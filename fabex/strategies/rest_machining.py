"""Rest Machining strategy for Fabex CNC.

Contains all logic specific to rest machining (clean-up / scallop-removal
passes): Z-map seeding, Z-map-based skip filtering, per-layer viewport
visualization, and the self-contained ``parallel`` entry point.

The standard parallel strategy (parallel.py) and core utilities
(chunk_utils.py) are deliberately left clean of any rest machining code.
"""

import bpy
from math import ceil

import numpy as np

from ..chunk_builder import CamPathChunk
from ..utilities.chunk_utils import (
    add_collections,
    chunks_to_mesh,
    sample_chunks,
)
from ..utilities.logging_utils import log
from ..utilities.operation_utils import get_layers
from ..utilities.strategy_utils import parallel_pattern

# ---------------------------------------------------------------------------
# Layer height suggestion
# ---------------------------------------------------------------------------


def suggest_layer_height(op):
    """Auto-suggest rest_layer_height based on cutter type and diameter.

    Called from update callbacks in operation_utils.py when the cutter
    changes and rest machining is active.
    """
    d = op.cutter_diameter
    if op.cutter_type in ["BALLNOSE", "BALLCONE"]:
        op.rest_layer_height = d * 0.25
    elif op.cutter_type == "VCARVE":
        op.rest_layer_height = d * 0.2
    else:
        op.rest_layer_height = d * 0.3


# ---------------------------------------------------------------------------
# Validation and Z-map seeding
# ---------------------------------------------------------------------------


async def validate_and_seed(o, scene):
    """Validate rest machining prerequisites and seed the Z-map.

    Returns:
        (True, None) on success.
        (False, error_message) when a prerequisite is not met.
    """
    prior_name = o.rest_machining_operation
    if not prior_name or prior_name == "NONE":
        return False, "Rest Machining is enabled but no previous operation is selected."
    if prior_name == o.name:
        return False, "Rest Machining: an operation cannot use its own path as roughing stock."
    if o.strategy != "PARALLEL":
        return False, "Rest Machining is only supported for the Parallel strategy."
    if o.optimisation.use_exact:
        return False, "Rest Machining requires image mode. Disable 'Use Exact Mode'."

    prior_op = scene.cam_operations.get(prior_name)
    if prior_op is None:
        return False, f"Rest Machining: operation '{prior_name}' not found in this scene."
    if not prior_op.path_object_name or prior_op.path_object_name not in bpy.data.objects:
        return False, (
            f"Rest Machining: '{prior_name}' has no calculated path. "
            "Calculate that operation first."
        )

    log.info(f"[Rest Machining] Seeding Z-map from roughing path '{prior_name}'")
    o.rest_zmap = None
    zmap = await seed_zmap(o)
    if zmap is not None:
        o.rest_zmap = zmap
        log.info(f"[Rest Machining] Z-map ready: shape={zmap.shape}")
    else:
        log.warning(f"[Rest Machining] Could not build Z-map from '{prior_name}'.")

    return True, None


async def seed_zmap(o):
    """Build a Z-map from the roughing operation's path object vertices.

    Traces every vertex of the roughing path through the roughing cutter
    footprint to produce a truthful record of what material was left behind.
    No simulation file is required.

    Returns:
        np.ndarray of shape (resx, resy), or None on failure.
    """
    from ..utilities.async_utils import progress_async
    from ..utilities.operation_utils import get_cutter_array
    from ..simulation import sim_cutter_spot

    scene = bpy.context.scene
    prior_op_name = o.rest_machining_operation
    if not prior_op_name or prior_op_name == "NONE":
        return None

    prior_op = scene.cam_operations.get(prior_op_name)
    if prior_op is None:
        return None

    if not prior_op.path_object_name or prior_op.path_object_name not in bpy.data.objects:
        log.warning(f"seed_zmap: roughing path object '{prior_op.path_object_name}' not found")
        return None

    await progress_async("Rest Machining: Building Z-map from roughing path", 0)

    curr_pixsize = o.optimisation.pixsize
    curr_borderwidth = o.borderwidth
    curr_minx, curr_miny = o.min.x, o.min.y
    curr_maxx, curr_maxy = o.max.x, o.max.y
    curr_resx = ceil((curr_maxx - curr_minx) / curr_pixsize) + 2 * curr_borderwidth
    curr_resy = ceil((curr_maxy - curr_miny) / curr_pixsize) + 2 * curr_borderwidth
    coordoffset = curr_borderwidth + curr_pixsize / 2.0

    stock_z = prior_op.max.z
    zmap = np.full((curr_resx, curr_resy), fill_value=stock_z, dtype=float)

    roughing_cutter = get_cutter_array(prior_op, curr_pixsize)

    path_obj = bpy.data.objects[prior_op.path_object_name]
    verts = path_obj.data.vertices
    num_verts = len(verts)

    log.info(
        f"seed_zmap: tracing {num_verts:,} roughing vertices "
        f"with {prior_op.cutter_type} Ø{prior_op.cutter_diameter * 1000:.2f} mm cutter, "
        f"stock_z={stock_z:.4f}, zmap shape={zmap.shape}"
    )

    report_step = max(1, num_verts // 100)
    for j, vert in enumerate(verts):
        co = vert.co
        xs = int(round((co.x - curr_minx) / curr_pixsize + coordoffset))
        ys = int(round((co.y - curr_miny) / curr_pixsize + coordoffset))
        sim_cutter_spot(xs, ys, co.z, roughing_cutter, zmap)
        if j % report_step == 0:
            pct = int(j / num_verts * 100)
            await progress_async(f"Rest Machining: Building Z-map {pct}%", pct)

    await progress_async("Rest Machining: Z-map ready", 100)

    cut_mask = zmap < stock_z - curr_pixsize
    cut_count = int(np.sum(cut_mask))
    at_stock = zmap.size - cut_count
    log.info(
        f"seed_zmap: pixels at stock={at_stock:,}, cut={cut_count:,} "
        f"({100 * cut_count / zmap.size:.1f}%), "
        f"z range=[{zmap.min():.4f}, {zmap.max():.4f}]"
    )
    return zmap


# ---------------------------------------------------------------------------
# Z-map skip filtering
# ---------------------------------------------------------------------------


def _filter_chunks_by_zmap(chunks, o):
    """Split chunks at points pre-cleared within one layer height by roughing.

    Works as a post-processing step on the output of sample_chunks: each
    chunk is scanned point-by-point against the Z-map.  Points that are
    already cleared (roughing left less than one layer height of stock) are
    skipped, creating new chunk boundaries at skip/cut transitions.

    Final-layer chunks (layer_index == num_layers - 1) are never skipped so
    the full surface always gets a finishing pass.

    Updates ``o.rest_stats_skipped`` and ``o.rest_stats_cut``.

    Returns:
        list of CamPathChunk, potentially more chunks than the input due to
        splitting at skip boundaries.
    """
    zmap = o.rest_zmap
    pixsize = o.optimisation.pixsize
    coordoff = o.borderwidth + pixsize / 2.0
    minx, miny = o.min.x, o.min.y
    layer_height = getattr(o, "rest_layer_height", 0.001)
    final_lh = getattr(o, "rest_final_layer_height", 0.0) or layer_height

    result = []
    skipped = cut = 0

    for chunk in chunks:
        pts = chunk.get_points()
        if not pts:
            continue

        layer_idx = getattr(chunk, "layer_index", -1)
        num_layers = getattr(chunk, "num_layers", 1)
        is_final = layer_idx < 0 or layer_idx == num_layers - 1
        # Use tighter threshold for the second-to-last layer (feeds final pass)
        threshold = final_lh if layer_idx == num_layers - 1 else layer_height

        current_pts = []
        for pt in pts:
            if is_final:
                current_pts.append(pt)
                cut += 1
                continue

            xi = int(round((pt[0] - minx) / pixsize + coordoff))
            yi = int(round((pt[1] - miny) / pixsize + coordoff))
            if 0 <= xi < zmap.shape[0] and 0 <= yi < zmap.shape[1]:
                remaining = zmap[xi, yi] - pt[2]
                if remaining <= threshold:
                    # Roughing pre-cleared this point — skip it
                    if current_pts:
                        sub = CamPathChunk(current_pts)
                        sub.layer_index = layer_idx
                        sub.num_layers = num_layers
                        result.append(sub)
                        current_pts = []
                    skipped += 1
                    continue

            current_pts.append(pt)
            cut += 1

        if current_pts:
            sub = CamPathChunk(current_pts)
            sub.layer_index = layer_idx
            sub.num_layers = num_layers
            result.append(sub)

    o.rest_stats_skipped = skipped
    o.rest_stats_cut = cut
    total = skipped + cut
    if total > 0:
        pct = 100.0 * skipped / total
        log.info(
            f"[Rest Machining] Points skipped: {skipped:,} ({pct:.1f}%), "
            f"cut: {cut:,} ({100 - pct:.1f}%)"
        )
    return result


# ---------------------------------------------------------------------------
# Per-layer viewport visualization
# ---------------------------------------------------------------------------


def _cleanup_vis_objects(path_name):
    """Remove stale layer visualization objects from a previous calculation."""
    layers_col = bpy.data.collections.get("Layers")
    if not layers_col:
        return
    prefix = f"{path_name}_L"
    stale = [obj for obj in list(layers_col.objects) if obj.name.startswith(prefix)]
    for obj in stale:
        mesh = obj.data
        layers_col.objects.unlink(obj)
        bpy.data.objects.remove(obj)
        if mesh and mesh.users == 0:
            bpy.data.meshes.remove(mesh)


def _make_emission_material(name, color):
    """Return a material with an Emission shader at the given RGBA color."""
    if name in bpy.data.materials:
        mat = bpy.data.materials[name]
    else:
        mat = bpy.data.materials.new(name)
    mat.diffuse_color = color
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()
    emit = nt.nodes.new("ShaderNodeEmission")
    emit.inputs[0].default_value = color
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    nt.links.new(emit.outputs[0], out.inputs[0])
    return mat


def _link_object_to_collection(obj, col):
    """Ensure obj is only in col (unlink from other collections)."""
    if obj.name not in [o.name for o in col.objects]:
        try:
            bpy.context.collection.objects.unlink(obj)
        except RuntimeError:
            pass
        col.objects.link(obj)


def _build_layer_vis_objects(chunks, path_name, scene):
    """Create per-layer visualization mesh objects in a 'Layers' sub-collection.

    One solid-color object per unique layer_index, plus red rapid-traversal
    objects.  Colors visible in both Solid and Material Preview viewport modes
    without relying on vertex-colour interpolation along edges.
    """
    _FINAL_LAYER_COLOR = (0.05, 0.8, 0.1, 1.0)
    _RAPID_COLOR = (1.0, 0.1, 0.1, 1.0)
    _LAYER_PALETTE = [
        (0.6, 0.0, 0.9, 1.0),  # purple
        (0.8, 0.7, 0.0, 1.0),  # dark yellow
        (0.1, 0.35, 0.95, 1.0),  # blue
        (0.0, 0.75, 0.75, 1.0),  # cyan
        (0.9, 0.4, 0.0, 1.0),  # orange
        (0.75, 0.0, 0.55, 1.0),  # magenta
    ]

    num_layers = max(
        (getattr(ch, "num_layers", 1) for ch in chunks if ch.get_points()),
        default=1,
    )

    def _layer_color(idx):
        if idx < 0 or num_layers <= 1 or idx == num_layers - 1:
            return _FINAL_LAYER_COLOR
        return _LAYER_PALETTE[idx % len(_LAYER_PALETTE)]

    # Ensure 'Layers' sub-collection exists inside 'Paths'
    collections = bpy.data.collections
    if "Paths" not in collections:
        add_collections()
    if "Layers" not in collections:
        layers_col = bpy.data.collections.new("Layers")
        collections["Paths"].children.link(layers_col)
    else:
        layers_col = collections["Layers"]

    # Group chunks by layer_index
    layer_chunks: dict = {}
    for ch in chunks:
        if ch.get_points():
            idx = getattr(ch, "layer_index", -1)
            layer_chunks.setdefault(idx, []).append(ch)

    if len(layer_chunks) <= 1:
        return

    for idx in sorted(layer_chunks.keys()):
        verts: list = []
        edges: list = []
        for ch in layer_chunks[idx]:
            pts = ch.get_points()
            if not pts:
                continue
            base = len(verts)
            verts.extend(pts)
            edges.extend((base + i, base + i + 1) for i in range(len(pts) - 1))

        if not verts:
            continue

        obj_name = f"{path_name}_L{idx:02d}"
        lmesh = bpy.data.meshes.new(obj_name)
        lmesh.from_pydata(verts, edges, [])

        if obj_name in bpy.data.objects:
            bpy.data.objects[obj_name].data = lmesh
            lobj = bpy.data.objects[obj_name]
        else:
            lobj = bpy.data.objects.new(obj_name, lmesh)
            layers_col.objects.link(lobj)

        lobj.location = (0, 0, 0)
        color = _layer_color(idx)
        lobj.color = color

        lmat = _make_emission_material(f"{obj_name}_mat", color)
        if lobj.data.materials:
            lobj.data.materials[0] = lmat
        else:
            lobj.data.materials.append(lmat)

        _link_object_to_collection(lobj, layers_col)

    log.info(f"[Color] Created {len(layer_chunks)} layer vis objects for {path_name}")


# ---------------------------------------------------------------------------
# Time savings logging
# ---------------------------------------------------------------------------


def _log_time_savings(o):
    """Log rest machining time savings after path calculation."""
    skipped = getattr(o, "rest_stats_skipped", 0)
    cut = getattr(o, "rest_stats_cut", 0)
    total = skipped + cut
    if total == 0:
        return

    pct_skip = 100.0 * skipped / total
    step = o.distance_between_paths
    f_cut = max(o.feedrate, 0.0001)
    machine = bpy.context.scene.cam_machine
    f_rapid = max(getattr(machine, "feedrate_rapid", f_cut * 5), f_cut)
    t_cut_everywhere = total * step / f_cut / 60.0
    t_rest = cut * step / f_cut / 60.0 + skipped * step / f_rapid / 60.0
    t_saved = t_cut_everywhere - t_rest

    log.info("-" * 60)
    log.info("[Rest Machining Results]")
    log.info(
        f"Points skipped : {skipped:>10,}  ({pct_skip:.1f}%)" f"  ~{skipped * step:.1f} m at rapid"
    )
    log.info(
        f"Points cut     : {cut:>10,}  ({100 - pct_skip:.1f}%)" f"  ~{cut * step:.1f} m at feedrate"
    )
    if t_cut_everywhere > 0:
        log.info(
            f"Est. time saved: {t_saved:.1f} min "
            f"(vs {t_cut_everywhere:.1f} min cut-everywhere = "
            f"{100 * t_saved / t_cut_everywhere:.0f}% reduction)"
        )
    log.info("-" * 60)


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------


async def parallel(o):
    """Rest machining parallel strategy.

    Replaces the standard parallel strategy when ``use_rest_machining`` is
    True.  Assumes ``validate_and_seed`` has already been called by
    path_ops._calc_path so that ``o.rest_zmap`` is populated.
    """
    log.info("~ Strategy: Rest Machining (Parallel) ~")

    pathSamples = parallel_pattern(o, o.parallel_angle)
    layers = get_layers(o, o.max_z, o.min.z)

    log.info(f"Sampling Object: {o.name}")
    chunks = await sample_chunks(o, pathSamples, layers)
    log.info(f"Sampling complete: {len(chunks)} raw chunks")

    # Filter: skip points already cleared by roughing
    if getattr(o, "rest_zmap", None) is not None:
        log.info("[Rest Machining] Applying Z-map skip filter")
        chunks = _filter_chunks_by_zmap(chunks, o)
        log.info(f"[Rest Machining] {len(chunks)} chunks after filtering")
    else:
        log.warning("[Rest Machining] No Z-map — running without skip logic")

    # Build the Blender path mesh (movement rules refined in next phase)
    scene = bpy.context.scene
    path_name = scene.cam_names.path_name_full
    _cleanup_vis_objects(path_name)
    chunks_to_mesh(chunks, o)

    # Per-layer visualization
    _build_layer_vis_objects(chunks, path_name, scene)

    _log_time_savings(o)
