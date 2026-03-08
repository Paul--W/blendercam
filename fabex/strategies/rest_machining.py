"""Rest Machining strategy for Fabex CNC.

Contains all logic specific to rest machining (clean-up / scallop-removal
passes): Z-map seeding, Z-map-based skip filtering, per-layer viewport
visualization, and the self-contained ``parallel`` entry point.

The standard parallel strategy (parallel.py) and core utilities
(chunk_utils.py) are deliberately left clean of any rest machining code.
"""

import time
from math import ceil

import bpy
from bpy_extras import object_utils
import numpy as np

from ..chunk_builder import CamPathChunk
from ..utilities.chunk_utils import (
    add_collections,
    get_operation_axes,
    optimize_chunk,
    sample_chunks,
)
from ..utilities.logging_utils import log
from ..utilities.operation_utils import get_layers
from ..utilities.simple_utils import activate, progress
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
# Gap terrain sampling
# ---------------------------------------------------------------------------


def _terrain_z_along_gap(zmap, ax, ay, bx, by, pixsize, minx, miny, coordoff):
    """Return the maximum Z-map value along the straight-line gap from A to B.

    PURPOSE
    -------
    Before the cutter traverses from the end of one machining segment to the
    start of the next, we need to know the highest remaining stock anywhere
    along that path so we can choose a safe traverse height.  This function
    scans the Z-map — a 2-D array of remaining stock heights built from the
    roughing operation — along the straight-line gap and returns the peak value.

    WHY PIXEL SPACE, NOT WORLD SPACE
    ---------------------------------
    The Z-map has a finite resolution of ``pixsize`` metres per pixel.  Sampling
    finer than one pixel per step yields no new information (the same pixel is
    read repeatedly).  Sampling coarser than one pixel per step risks skipping
    over a pixel that contains a terrain peak.  The correct sampling density is
    therefore exactly one Z-map pixel per step — which means the step count must
    be computed in pixel space, not world space.

    WHY THE BRESENHAM PIXEL COUNT
    ------------------------------
    Converting both endpoints to integer pixel coordinates and using:

        n = max(|x1 - x0|, |y1 - y0|) + 1

    is the Bresenham line-drawing criterion.  It guarantees that every pixel
    the straight line passes through is visited at least once: the larger of
    the two axis differences is advanced by exactly one pixel per step, while
    the smaller axis advances by ≤ 1 pixel per step.  No pixel is skipped;
    no pixel is sampled twice unnecessarily.

    WHY THIS MATTERS FOR REST MACHINING
    -------------------------------------
    The roughing and finishing operations are typically run in different
    directions — for example, roughing in the X direction and finishing at 45°.
    The gap between two adjacent finishing segments therefore runs perpendicular
    to the finishing direction (i.e. at 135°), crossing the roughing-direction
    passes at an angle.

    Wherever the gap path crosses the space *between* two roughing passes there
    is a scallop ridge — material the roughing cutter never reached.  These
    ridges repeat at the roughing ``distance_between_paths`` interval.  When
    the gap path crosses them at an angle the apparent ridge width along the
    gap direction is:

        apparent_width = scallop_width / sin(angle_between_directions)

    At 45° this is ≈ 1.4× the actual scallop width.  A coarse sampling step
    (e.g. only checking the two endpoints) can miss these ridges entirely,
    producing an unsafe traverse height that results in the finishing cutter
    hitting remaining roughing stock mid-traverse.  Pixel-resolution sampling
    eliminates this risk.

    IMPLEMENTATION
    --------------
    numpy ``linspace`` generates ``n`` evenly-spaced values between the two
    pixel-space endpoints, rounded to the nearest integer.  The resulting
    index arrays ``xs`` and ``ys`` are clipped to the valid Z-map extents to
    handle gap paths that reach or slightly exceed the map boundary (e.g. at
    the edge of the stock).  A single ``np.max`` call over the gathered pixels
    returns the peak terrain height with no Python-level loop.

    PERFORMANCE
    -----------
    For a typical short gap of 14 mm at pixsize = 0.1 mm, n ≈ 140 — a
    140-element numpy max, negligible cost.  For a long gap of 200 mm,
    n ≈ 2000 — still a single vectorised operation well under 1 ms.

    Args:
        zmap:      2-D numpy array (resx, resy) of remaining stock heights.
        ax, ay:    World-space XY of gap start (end of current segment).
        bx, by:    World-space XY of gap end   (start of next segment).
        pixsize:   Z-map pixel pitch in metres (o.optimisation.pixsize).
        minx, miny: World-space origin of Z-map (o.min.x, o.min.y).
        coordoff:  Pixel-coordinate offset (borderwidth + pixsize / 2).

    Returns:
        float — maximum terrain Z along the gap path.
    """
    x0 = int(round((ax - minx) / pixsize + coordoff))
    y0 = int(round((ay - miny) / pixsize + coordoff))
    x1 = int(round((bx - minx) / pixsize + coordoff))
    y1 = int(round((by - miny) / pixsize + coordoff))

    n = max(abs(x1 - x0), abs(y1 - y0)) + 1

    xs = np.round(np.linspace(x0, x1, n)).astype(int)
    ys = np.round(np.linspace(y0, y1, n)).astype(int)
    np.clip(xs, 0, zmap.shape[0] - 1, out=xs)
    np.clip(ys, 0, zmap.shape[1] - 1, out=ys)

    return float(np.max(zmap[xs, ys]))


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
# Rest path building
# ---------------------------------------------------------------------------


def _build_rest_path(chunks, o):
    """Build the Blender path mesh for rest machining using the movement rules.

    Replaces the generic chunks_to_mesh for rest machining.  Iterates over the
    filtered chunk list and inserts the correct intermediate waypoints between
    each consecutive pair of machining segments according to the movement rules:

        Rule 1 — Job start: rapid to above first segment, plunge to cut depth.
        Rule 2 — Milling: append all segment points at mill feedrate.
        Rule 3 — Gap, same/lower Z, terrain clear: direct connection, no lift.
                 Both short and long gaps stay at cut depth (mill feedrate).
                 For typical workpiece sizes this is faster than lift+rapid+drop
                 because the plunge-speed lift/drop cost dominates until gap
                 distances exceed ~600 mm.
        Rule 4 — Gap, higher Z, terrain clear: one waypoint at
                 (next_x, next_y, next_z + layer_height).  The following
                 vertical drop to next_chunk[0] is recognised by the G-code
                 exporter as a plunge (angle from downvector < plunge_limit).
        Rule 5 — Gap, terrain obstruction: two waypoints —
                 (last_x, last_y, traverse_z) vertical rise, then
                 (next_x, next_y, traverse_z) flat traverse.  The following
                 vertical drop to next_chunk[0] triggers plunge feedrate.
                 traverse_z = max(terrain_z, next_z) + terrain_clearance.

    Feedrate encoding via vertex Z position (G-code exporter convention):
        v.z >= free_height          → G00 rapid (freefeedrate)
        steep downward move vector  → G01 at plunge feedrate
        all other moves             → G01 at mill feedrate

    All plunge descents are encoded as purely vertical moves (same XY as the
    waypoint above) so the exporter's angle check always fires correctly.
    """
    t = time.time()
    scene = bpy.context.scene
    machine = scene.cam_machine

    free_height = o.movement.free_height
    layer_height = getattr(o, "rest_layer_height", 0.001)
    terrain_clearance = layer_height + 0.004  # layer_height + 4 mm
    cutter_diameter = o.cutter_diameter

    zmap = getattr(o, "rest_zmap", None)
    if zmap is not None:
        pixsize = o.optimisation.pixsize
        minx, miny = o.min.x, o.min.y
        coordoff = o.borderwidth + pixsize / 2.0

    three_axis, _, _, indexed_four_axis, indexed_five_axis = get_operation_axes(o)

    if machine.use_position_definitions:
        origin = (
            machine.starting_position.x,
            machine.starting_position.y,
            machine.starting_position.z,
        )
    else:
        origin = (0, 0, free_height)

    vertices = [origin] if three_axis else []
    vertices_rotations = [] if not three_axis else None

    progress("~ Building Rest Machining Paths ~")

    # Pre-filter and optionally optimise chunks once.
    active_chunks = []
    for chunk in chunks:
        if chunk.count() > 0:
            if o.optimisation.optimize:
                chunk = optimize_chunk(chunk, o)
            active_chunks.append(chunk)

    lifted = True  # True = cutter is at or above free_height

    for ci, chunk in enumerate(active_chunks):
        chunk_points = chunk.get_points()

        # --- Drop / approach ---
        # If lifted, add a vertex at free_height above the first cut point so
        # the exporter emits a rapid XY move followed by a plunge descent.
        if lifted:
            if three_axis or indexed_five_axis or indexed_four_axis:
                vertices.append((chunk_points[0][0], chunk_points[0][1], free_height))
            else:
                vertices.append(chunk.startpoints[0])
                vertices_rotations.append(chunk.rotations[0])

        # --- Rule 2: mill along segment ---
        vertices.extend(chunk_points)
        if not three_axis:
            vertices_rotations.extend(chunk.rotations)

        # --- Inter-chunk movement (Rules 3 / 4 / 5) ---
        lift = True  # default: safe retract to free_height

        if ci < len(active_chunks) - 1 and three_axis:
            next_chunk = active_chunks[ci + 1]
            next_pts = next_chunk.get_points()

            if next_pts:
                last_pt = chunk_points[-1]
                next_pt = next_pts[0]
                ax, ay, current_z = last_pt[0], last_pt[1], last_pt[2]
                bx, by, next_z = next_pt[0], next_pt[1], next_pt[2]

                if zmap is not None:
                    terrain_z = _terrain_z_along_gap(
                        zmap, ax, ay, bx, by, pixsize, minx, miny, coordoff
                    )
                else:
                    # No Z-map: conservative — assume worst-case obstruction.
                    terrain_z = max(current_z, next_z)

                min_z = min(current_z, next_z)

                if terrain_z <= min_z:
                    # Terrain is clear at or below the lower of the two endpoints.
                    if next_z <= current_z:
                        # Rule 3: same or lower Z — direct connection, no lift.
                        # The exporter assigns mill feedrate (horizontal/gentle
                        # angle) or plunge feedrate (steep descent) automatically.
                        lift = False
                    else:
                        # Rule 4: next segment is higher — diagonal waypoint above
                        # the destination, then a vertical drop that the exporter
                        # recognises as a plunge.
                        vertices.append((bx, by, next_z + layer_height))
                        lift = False
                else:
                    # Rule 5: terrain obstruction.
                    # traverse_z clears both the terrain peak and the destination.
                    traverse_z = max(terrain_z, next_z) + terrain_clearance
                    # Vertical rise at current XY.
                    vertices.append((ax, ay, traverse_z))
                    # Flat traverse to above next segment start.
                    vertices.append((bx, by, traverse_z))
                    # The vertical drop to next_chunk[0] (next_z) follows
                    # automatically and triggers plunge feedrate.
                    lift = False

        if lift:
            if three_axis or indexed_five_axis or indexed_four_axis:
                vertices.append((chunk_points[-1][0], chunk_points[-1][1], free_height))
            else:
                vertices.append(chunk.startpoints[-1])
                vertices_rotations.append(chunk.rotations[-1])

        lifted = lift

    log.info(
        f"[Rest] Path built: {len(vertices):,} vertices, "
        f"{len(active_chunks)} chunks, {time.time() - t:.2f}s"
    )

    # --- Create Blender path mesh object ---
    edges = [(a, a + 1) for a in range(len(vertices) - 1)]
    path_name = scene.cam_names.path_name_full
    mesh = bpy.data.meshes.new(path_name)
    mesh.name = path_name
    mesh.from_pydata(vertices, edges, [])

    if path_name in scene.objects:
        scene.objects[path_name].data = mesh
        ob = scene.objects[path_name]
    else:
        ob = object_utils.object_data_add(bpy.context, mesh, operator=None)

    if not three_axis:
        ob.shape_key_add()
        ob.shape_key_add()
        shapek = mesh.shape_keys.key_blocks[1]
        shapek.name = "rotations"
        for i, co in enumerate(vertices_rotations):
            shapek.data[i].co = co

    ob.location = (0, 0, 0)
    ob.color = machine.path_color
    o.path_object_name = path_name

    collections = bpy.data.collections
    if "Paths" not in collections:
        add_collections()
    bpy.context.collection.objects.unlink(ob)
    collections["Paths"].objects.link(ob)

    if (o.geometry_source == "OBJECT") and o.parent_path_to_object:
        activate(o.objects[0])
        ob.select_set(state=True, view_layer=None)
        bpy.ops.object.parent_set(type="OBJECT", keep_transform=True)
    else:
        ob.select_set(state=True, view_layer=None)


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

    # Build the Blender path mesh using rest machining movement rules.
    scene = bpy.context.scene
    path_name = scene.cam_names.path_name_full
    _cleanup_vis_objects(path_name)
    _build_rest_path(chunks, o)

    # Per-layer visualization
    _build_layer_vis_objects(chunks, path_name, scene)

    _log_time_savings(o)
