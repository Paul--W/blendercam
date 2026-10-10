"""Rest Machining strategy for Fabex CNC.

Contains all logic specific to rest machining (clean-up / scallop-removal
passes): Z-map seeding, Z-map-based skip filtering, per-layer viewport
visualization, and the self-contained ``parallel`` entry point.

The standard parallel strategy (parallel.py) and core utilities
(chunk_utils.py) are deliberately left clean of any rest machining code.
"""

import time
from math import ceil, sqrt, inf

import bpy
from bpy_extras import object_utils
import numpy as np

from ..chunk_builder import CamPathChunk
from ..utilities.chunk_utils import (
    chunks_to_mesh,
    sample_chunks,
)
from ..utilities.operation_utils import get_layers
from ..utilities.logging_utils import log
from ..utilities.strategy_utils import parallel_pattern

# ---------------------------------------------------------------------------
# Diagnostic flags — controlled by operation properties when available,
# with hardcoded fallbacks for development.
# ---------------------------------------------------------------------------

def _get_diag_flags(o):
    """Read diagnostic flags from machine verbose_logging property."""
    try:
        machine = bpy.context.scene.cam_machine
        verbose = getattr(machine, "verbose_logging", False)
    except Exception:
        verbose = False
    return (verbose, verbose)

# Minimum layers for a pixel to be considered part of a deep island.
_MIN_ISLAND_LAYERS = 2


# ---------------------------------------------------------------------------
# Island detection for detail passes (prior_skin = 0)
# ---------------------------------------------------------------------------


def _build_remaining_map(o):
    """Compute per-pixel remaining material (zmap - model surface)."""
    zmap = getattr(o, "rest_zmap", None)
    if zmap is None:
        return None, None

    stock_z = getattr(o, "rest_stock_z", 0.0)
    resx, resy = zmap.shape

    offimg = getattr(o, "offset_image", None)
    zbuf = getattr(o, "zbuffer_image", None)
    model = offimg if offimg is not None else zbuf
    if model is None:
        return None, None

    mx = min(resx, model.shape[0])
    my = min(resy, model.shape[1])

    remaining = np.zeros((resx, resy), dtype=np.float32)
    remaining[:mx, :my] = zmap[:mx, :my] - model[:mx, :my]

    valid = np.zeros((resx, resy), dtype=bool)
    valid[:mx, :my] = (
        (zmap[:mx, :my] < stock_z - 1e-9) &
        (model[:mx, :my] > -9.0)
    )
    return remaining, valid


def _label_islands(mask):
    """Label connected components in a boolean mask using BFS flood fill."""
    resx, resy = mask.shape
    labels = np.zeros((resx, resy), dtype=np.int32)
    visited = np.zeros_like(mask, dtype=bool)
    current_label = 0

    for sx in range(resx):
        for sy in range(resy):
            if mask[sx, sy] and not visited[sx, sy]:
                current_label += 1
                queue = [(sx, sy)]
                visited[sx, sy] = True
                while queue:
                    cx, cy = queue.pop()
                    labels[cx, cy] = current_label
                    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                        nx, ny = cx + dx, cy + dy
                        if (0 <= nx < resx and 0 <= ny < resy
                                and mask[nx, ny] and not visited[nx, ny]):
                            visited[nx, ny] = True
                            queue.append((nx, ny))
    return labels, current_label



async def _sort_islands_detail(upper_chunks, o):
    """Sort detail-pass rest chunks using spatial region grouping.

    Labels connected spatial regions from the needs_rest mask (not just deep
    pockets), assigns chunks to regions, and processes regions in nearest-
    neighbor order of centroids.  Within regions that have deep pockets
    (>= 2 layers), sorts layer-by-layer.  Shallow regions use nearest-
    neighbor.  Large regions are subdivided into grid cells to prevent
    long-distance jumps within a single connected area.

    Only used when prior_skin = 0 (detail pass after a finishing pass).
    Returns sorted chunk list, or None to fall back to standard sort.
    """
    from collections import defaultdict

    layer_height = getattr(o, "rest_layer_height", 0.001)
    prior_skin = getattr(o, "rest_prior_skin", 0.0)

    # Use the SAME threshold as _ordered_filter_chunks so every pixel
    # that produced a chunk is also marked needs_rest for spatial grouping.
    if o.cutter_type == "BALLCONE" and getattr(o, "ball_radius", 0) > 0:
        effective_diameter = o.ball_radius * 2.0
    else:
        effective_diameter = o.cutter_diameter
    noise_floor = effective_diameter * 0.05
    skip_threshold = prior_skin + noise_floor

    log.info("[Rest Islands] Building remaining-material map...")
    remaining, valid = _build_remaining_map(o)
    if remaining is None:
        return None

    resx, resy = remaining.shape
    pixsize = o.optimisation.pixsize
    minx, miny = o.min.x, o.min.y
    coordoff = o.borderwidth + pixsize / 2.0

    # Identify pixels that need rest machining
    needs_rest = (remaining > skip_threshold) & valid
    rest_px = int(needs_rest.sum())
    if rest_px == 0:
        log.info("[Rest Islands] No rest pixels found")
        return None

    # Layer count per pixel (for deep pocket detection)
    layer_count = np.zeros((resx, resy), dtype=np.float32)
    layer_count[needs_rest] = np.maximum(1.0, remaining[needs_rest] / layer_height)
    deep_mask = layer_count >= _MIN_ISLAND_LAYERS
    deep_px = int(deep_mask.sum())

    # Label SPATIAL regions using needs_rest — groups all rest-area chunks
    # by proximity, not just deep pockets.  This prevents the old problem
    # where shallow-area chunks became "contours" and got a single global
    # nearest-neighbor sort that hopped between distant areas.
    log.info("[Rest Islands] Labelling spatial regions...")
    labels, num_regions = _label_islands(needs_rest)
    log.info(
        f"[Rest Islands] {rest_px} rest pixels, {deep_px} deep, "
        f"{num_regions} spatial regions"
    )

    if num_regions == 0:
        return None

    # Region stats: pixel count, centroid, whether it has deep areas
    region_stats = {}
    for rid in range(1, num_regions + 1):
        mask_r = labels == rid
        px = int(mask_r.sum())
        xi_arr, yi_arr = np.where(mask_r)
        cx = float(minx + (xi_arr.mean() - coordoff) * pixsize)
        cy = float(miny + (yi_arr.mean() - coordoff) * pixsize)
        has_deep = bool(np.any(deep_mask[mask_r]))
        region_stats[rid] = (px, cx, cy, has_deep)

    # Sort regions by nearest-neighbor traversal of centroids
    region_order = _sort_regions_nearest(region_stats)

    for i, rid in enumerate(region_order[:5]):
        px, cx, cy, has_deep = region_stats[rid]
        log.debug(
            f"[Rest Islands] Region {rid}: {px} px, "
            f"centroid=({cx*1000:.1f},{cy*1000:.1f})mm, deep={has_deep}"
        )
    if num_regions > 5:
        log.debug(f"[Rest Islands] ... and {num_regions - 5} more regions")

    # Minimum pixels for per-region treatment
    min_region_px = max(10, int((o.cutter_diameter / pixsize) ** 2))

    # Assign chunks to spatial regions
    region_chunks = defaultdict(list)
    unassigned = []
    for ch in upper_chunks:
        pts = ch.get_points()
        if not pts:
            unassigned.append(ch)
            continue
        mid = pts[len(pts) // 2]
        xi = int(round((mid[0] - minx) / pixsize + coordoff))
        yi = int(round((mid[1] - miny) / pixsize + coordoff))
        rid = 0
        if 0 <= xi < resx and 0 <= yi < resy:
            rid = int(labels[xi, yi])
        if rid > 0:
            region_chunks[rid].append(ch)
        else:
            unassigned.append(ch)

    assigned = sum(len(v) for v in region_chunks.values())
    log.info(
        f"[Rest Islands] {assigned} chunks in {len(region_chunks)} regions, "
        f"{len(unassigned)} unassigned"
    )

    # Boost merge distance for processing
    _saved_merge = o.movement.merge_distance
    o.movement.merge_distance = max(_saved_merge, o.cutter_diameter * 3.0)

    # Grid cell size for subdividing large regions (in metres)
    _GRID_CELL = o.cutter_diameter * 20.0

    sorted_all = []
    pos = (0, 0, 0)
    regions_processed = 0

    for rid in region_order:
        chunks = region_chunks.get(rid, [])
        if not chunks:
            continue

        px, cx, cy, has_deep = region_stats[rid]

        # Small regions: batch with unassigned for one sort at end
        if px < min_region_px:
            unassigned.extend(chunks)
            continue

        # Sort this region's chunks
        try:
            sorted_region = await _sort_region_chunks(
                chunks, o, pos, has_deep, _GRID_CELL
            )
            if sorted_region:
                pos = sorted_region[-1].get_point(-1)
            sorted_all.extend(sorted_region)
            regions_processed += 1
        except Exception as e:
            log.warning(f"[Rest Islands] Region {rid} failed: {e}")
            sorted_region = await rest_sort_chunks(chunks, o, last_pos=pos)
            if sorted_region:
                pos = sorted_region[-1].get_point(-1)
            sorted_all.extend(sorted_region)

    o.movement.merge_distance = _saved_merge

    # Unassigned/small-region chunks — nearest-neighbor
    if unassigned:
        log.info(
            f"[Rest Islands] Unassigned: {len(unassigned)} chunks "
            "— nearest-neighbor"
        )
        sorted_un = await rest_sort_chunks(unassigned, o, last_pos=pos)
        if sorted_un:
            pos = sorted_un[-1].get_point(-1)
        sorted_all.extend(sorted_un)

    log.info(
        f"[Rest Islands] Complete: {len(sorted_all)} chunks, "
        f"{regions_processed} regions processed"
    )
    return sorted_all


def _sort_regions_nearest(region_stats):
    """Sort region IDs by nearest-neighbor traversal on centroids."""
    if not region_stats:
        return []
    remaining = set(region_stats.keys())
    current = min(remaining, key=lambda i: region_stats[i][1] ** 2 + region_stats[i][2] ** 2)
    remaining.remove(current)
    result = [current]
    while remaining:
        cx, cy = region_stats[current][1], region_stats[current][2]
        best = min(remaining, key=lambda i: (region_stats[i][1] - cx) ** 2 + (region_stats[i][2] - cy) ** 2)
        remaining.remove(best)
        result.append(best)
        current = best
    return result


async def _sort_region_chunks(chunks, o, last_pos, has_deep, grid_cell):
    """Sort chunks within a single spatial region.

    For regions with deep pockets, sorts layer-by-layer (shallowest first).
    For large regions (> 2000 chunks), subdivides into grid cells processed
    in meander order to maintain spatial locality.
    """
    from collections import defaultdict

    _GRID_THRESHOLD = 2000  # subdivide regions with more chunks than this

    if has_deep:
        # Layer-by-layer sort
        layer_groups = defaultdict(list)
        for ch in chunks:
            layer_groups[getattr(ch, "layer_index", 0)].append(ch)

        result = []
        pos = last_pos
        for li in sorted(layer_groups.keys()):
            layer_chunks = layer_groups[li]
            if len(layer_chunks) > _GRID_THRESHOLD:
                sorted_layer = await _grid_sort_chunks(
                    layer_chunks, o, pos, grid_cell
                )
            else:
                sorted_layer = await rest_sort_chunks(
                    layer_chunks, o, last_pos=pos
                )
            if sorted_layer:
                pos = sorted_layer[-1].get_point(-1)
            result.extend(sorted_layer)
        return result

    # Shallow region — nearest-neighbor, with grid for large regions
    if len(chunks) > _GRID_THRESHOLD:
        return await _grid_sort_chunks(chunks, o, last_pos, grid_cell)
    return await rest_sort_chunks(chunks, o, last_pos=last_pos)


async def _grid_sort_chunks(chunks, o, last_pos, grid_cell):
    """Sort chunks using a spatial grid to prevent long-distance jumps.

    Divides the chunk area into grid cells, processes cells in meander order
    (left-to-right then right-to-left per row), and uses nearest-neighbor
    within each cell.
    """
    from collections import defaultdict

    if not chunks:
        return []

    # Compute bounding box of chunk start points
    start_pts = []
    for ch in chunks:
        pts = ch.get_points()
        if pts:
            start_pts.append((pts[0][0], pts[0][1]))
    if not start_pts:
        return await rest_sort_chunks(chunks, o, last_pos=last_pos)

    xs = [p[0] for p in start_pts]
    ys = [p[1] for p in start_pts]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)

    # Grid dimensions
    nx = max(1, int(ceil((x_max - x_min) / grid_cell)))
    ny = max(1, int(ceil((y_max - y_min) / grid_cell)))

    # Assign chunks to grid cells
    cells = defaultdict(list)
    for ch in chunks:
        pts = ch.get_points()
        if not pts:
            cells[(0, 0)].append(ch)
            continue
        sx, sy = pts[0][0], pts[0][1]
        ci = min(nx - 1, int((sx - x_min) / grid_cell)) if grid_cell > 0 else 0
        cj = min(ny - 1, int((sy - y_min) / grid_cell)) if grid_cell > 0 else 0
        cells[(ci, cj)].append(ch)

    # Meander order: row by row, alternating direction
    result = []
    pos = last_pos
    for row in range(ny):
        cols = range(nx) if row % 2 == 0 else range(nx - 1, -1, -1)
        for col in cols:
            cell_chunks = cells.get((col, row))
            if not cell_chunks:
                continue
            sorted_cell = await rest_sort_chunks(
                cell_chunks, o, last_pos=pos
            )
            if sorted_cell:
                pos = sorted_cell[-1].get_point(-1)
            result.extend(sorted_cell)

    log.debug(
        f"[Rest Islands] Grid sort: {len(chunks)} chunks in "
        f"{nx}x{ny} grid ({len(cells)} occupied cells)"
    )
    return result


# ---------------------------------------------------------------------------
# Layer height suggestion
# ---------------------------------------------------------------------------


def suggest_layer_height(op, machine=None):
    """Auto-suggest rest_layer_height based on cutter and auto-calc.

    When auto-calculate is enabled, uses the same layer height as the
    calculation engine (matches Calculation Details display).  Otherwise
    falls back to a fraction of cutter diameter.

    Called from update callbacks in operation_utils.py when the cutter
    changes and rest machining is active.
    """
    if machine is not None and machine.auto_calc_layer_height:
        try:
            from ..utilities.cutting_params import auto_calculate_for_operation
            params = auto_calculate_for_operation(op, machine, write_back=False)
            if params and "layer_height_mm" in params:
                op.rest_layer_height = params["layer_height_mm"] * 0.001  # mm → m
                return
        except Exception:
            pass

    # Fallback: fraction of cutter diameter
    d = op.cutter_diameter
    if op.cutter_type in ["BALLNOSE", "BALLCONE"]:
        op.rest_layer_height = d * 0.33
    elif op.cutter_type == "VCARVE":
        op.rest_layer_height = d * 0.25
    else:
        op.rest_layer_height = d * 0.35


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


def _sweep_segment_to_zmap(zmap, ax, ay, az, bx, by, bz, cutter_r, stock_z, pixsize, minx, miny, coordoff):
    """Sweep a ball-nose path segment into the Z-map as a smooth furrow.

    Instead of stamping discrete circular dots along the segment, this function
    analytically computes the ball-nose depth profile for every Z-map pixel
    inside the swept capsule (segment body + hemispherical end-caps).

    The result is a smooth furrow:
      - Along the travel direction: uniform depth (z interpolated from endpoints)
      - Across the travel direction: ball-nose convex profile
        car(d_perp) = r - sqrt(r² - d_perp²)   →  0 at centreline, +r at edge

    This eliminates the wave / concave-circle artefacts produced by dot sampling.
    """
    from math import floor, ceil

    r = cutter_r
    r2 = r * r

    # Axis-aligned bounding box of the capsule (segment expanded by cutter radius).
    x_lo_w = min(ax, bx) - r
    x_hi_w = max(ax, bx) + r
    y_lo_w = min(ay, by) - r
    y_hi_w = max(ay, by) + r

    # Convert world-space bounds to Z-map pixel indices and clip to array extents.
    xi_lo = max(0, int(floor((x_lo_w - minx) / pixsize + coordoff)))
    xi_hi = min(zmap.shape[0] - 1, int(ceil((x_hi_w - minx) / pixsize + coordoff)))
    yi_lo = max(0, int(floor((y_lo_w - miny) / pixsize + coordoff)))
    yi_hi = min(zmap.shape[1] - 1, int(ceil((y_hi_w - miny) / pixsize + coordoff)))

    if xi_lo > xi_hi or yi_lo > yi_hi:
        return  # capsule entirely outside Z-map

    # Pixel-space coordinate grids for the bounding-box patch.
    xi = np.arange(xi_lo, xi_hi + 1, dtype=np.float64)
    yi = np.arange(yi_lo, yi_hi + 1, dtype=np.float64)
    xi_grid, yi_grid = np.meshgrid(xi, yi, indexing="ij")

    # World coordinates of each pixel centre.
    wx = minx + (xi_grid - coordoff) * pixsize
    wy = miny + (yi_grid - coordoff) * pixsize

    # Vector from segment start A to each pixel.
    relx = wx - ax
    rely = wy - ay

    dx, dy = bx - ax, by - ay
    L = (dx * dx + dy * dy) ** 0.5

    if L < 1e-12:
        # Degenerate point — fall back to circular stamp.
        d_eff = np.sqrt(relx ** 2 + rely ** 2)
        z_at_t = az
    else:
        ux, uy = dx / L, dy / L                    # unit vector along segment
        along = relx * ux + rely * uy               # signed projection onto segment
        perp  = -relx * uy + rely * ux              # signed perpendicular distance
        t_cl  = np.clip(along, 0.0, L)             # clamped position on segment
        d_cap = along - t_cl                        # 0 in body, nonzero beyond endpoints
        d_eff = np.sqrt(perp ** 2 + d_cap ** 2)    # 3D distance from nearest segment point
        z_at_t = az + (bz - az) * (t_cl / L)       # Z interpolated along segment

    # Ball-nose height offset: convex profile (0 at centreline, +r at edge).
    # car(d) = r - sqrt(r² - d²)
    d_eff_sq = d_eff ** 2
    car = np.where(d_eff_sq <= r2,
                   r - np.sqrt(np.maximum(r2 - d_eff_sq, 0.0)),
                   np.inf)  # np.inf outside cutter radius → np.minimum ignores

    z_cut = z_at_t + car

    # Write the deepest (minimum) cut into the Z-map patch.
    np.minimum(
        zmap[xi_lo:xi_hi + 1, yi_lo:yi_hi + 1],
        z_cut,
        out=zmap[xi_lo:xi_hi + 1, yi_lo:yi_hi + 1],
    )


async def seed_zmap(o):
    """Build a Z-map from the roughing operation's path object vertices.

    Traces every vertex of the roughing path through the roughing cutter
    footprint to produce a truthful record of what material was left behind.
    No simulation file is required.

    Returns:
        np.ndarray of shape (resx, resy), or None on failure.
    """
    from ..utilities.async_utils import progress_async

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

    path_obj = bpy.data.objects[prior_op.path_object_name]
    verts = path_obj.data.vertices
    num_verts = len(verts)

    # Use matrix_world to convert vertices from local to world space.
    # CAM path objects are created with ob.location=(0,0,0) but rotation and
    # scale are not explicitly reset; any non-identity transform would cause
    # vert.co (local) to differ from world coordinates, producing a
    # rotated/scaled Z-map that doesn't align with the finishing operation.
    matrix = path_obj.matrix_world
    mat_has_transform = matrix != matrix.Identity(4)
    log.debug(
        f"seed_zmap: finishing bounds: "
        f"x=[{curr_minx:.5f}, {curr_maxx:.5f}] y=[{curr_miny:.5f}, {curr_maxy:.5f}] "
        f"pixsize={curr_pixsize:.6f} border={curr_borderwidth} image={curr_resx}x{curr_resy}"
    )
    log.info(
        f"seed_zmap: tracing {num_verts:,} roughing vertices "
        f"with {prior_op.cutter_type} Ø{prior_op.cutter_diameter * 1000:.2f} mm cutter"
    )

    cutter_r = prior_op.cutter_diameter / 2

    prev_co = None
    vx_min, vx_max, vy_min, vy_max = np.inf, -np.inf, np.inf, -np.inf
    segs_swept = 0
    report_step = max(1, num_verts // 100)
    for j, vert in enumerate(verts):
        co = matrix @ vert.co  # world-space coordinates
        vx_min = min(vx_min, co.x)
        vx_max = max(vx_max, co.x)
        vy_min = min(vy_min, co.y)
        vy_max = max(vy_max, co.y)

        if prev_co is not None:
            # Skip rapid moves — at least one endpoint must be below stock.
            at_cut_depth = (prev_co.z < stock_z - curr_pixsize
                            or co.z < stock_z - curr_pixsize)
            if at_cut_depth:
                _sweep_segment_to_zmap(
                    zmap,
                    prev_co.x, prev_co.y, prev_co.z,
                    co.x, co.y, co.z,
                    cutter_r, stock_z,
                    curr_pixsize, curr_minx, curr_miny, coordoffset,
                )
                segs_swept += 1

        prev_co = co

        if j % report_step == 0:
            pct = int(j / num_verts * 100)
            await progress_async("Rest Machining: Building Z-map", pct)

    log.debug(
        f"seed_zmap: roughing vertex bounds: "
        f"x=[{vx_min:.5f}, {vx_max:.5f}] y=[{vy_min:.5f}, {vy_max:.5f}]"
    )
    log.debug(
        f"seed_zmap: swept {segs_swept:,} segments"
    )

    await progress_async("Rest Machining: Z-map ready", 100)

    cut_mask = zmap < stock_z - curr_pixsize
    cut_count = int(np.sum(cut_mask))
    at_stock = zmap.size - cut_count
    log.info(
        f"seed_zmap: pixels at stock={at_stock:,}, cut={cut_count:,} "
        f"({100 * cut_count / zmap.size:.1f}%), stock_z={stock_z:.6f}"
    )
    if getattr(o, "rest_create_diagnostic_images", False):
        _save_zmap_image(zmap, stock_z)
    o.rest_stock_z = stock_z
    o.rest_prior_skin = prior_op.skin  # roughing skin (metres) — used by filter threshold
    return zmap


# ---------------------------------------------------------------------------
# Z-map diagnostic image
# ---------------------------------------------------------------------------


def _save_zmap_image(zmap, stock_z):
    """Create a grayscale diagnostic image from the Z-map and save as PNG.

    Bright (white) = uncut material at stock surface.
    Dark (black) = deepest cuts by the roughing operation.

    The image is created in Blender's image data-block list so it can be
    viewed immediately in the Image Editor (switch any editor to Image
    Editor, open the image named 'rest_zmap_diag').  It is also saved as
    a PNG file to the fabex/logs/ directory.

    Call this after seed_zmap completes to verify roughing coverage before
    the finishing pass filter runs.
    """
    import os

    z_min = float(zmap.min())
    z_max = float(stock_z)
    z_rng = z_max - z_min
    if z_rng < 1e-9:
        normalized = np.ones(zmap.shape, dtype=np.float32)
    else:
        normalized = ((zmap - z_min) / z_rng).astype(np.float32)

    resx, resy = zmap.shape

    # Remove stale image from a previous run.
    if "rest_zmap_diag" in bpy.data.images:
        bpy.data.images.remove(bpy.data.images["rest_zmap_diag"])

    img = bpy.data.images.new("rest_zmap_diag", width=resx, height=resy, alpha=False)
    img.colorspace_settings.name = "Non-Color"

    # Blender image layout: flat RGBA, row 0 = bottom of screen = world miny.
    # zmap shape is (resx, resy) with axis-0 = X, axis-1 = Y.
    # Transpose to (resy, resx) so each row corresponds to a Y slice.
    grey = normalized.T  # shape (resy, resx)
    rgba = np.empty((resy, resx, 4), dtype=np.float32)
    rgba[:, :, 0] = grey
    rgba[:, :, 1] = grey
    rgba[:, :, 2] = grey
    rgba[:, :, 3] = 1.0
    img.pixels.foreach_set(rgba.ravel())
    img.update()

    log_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "logs")
    os.makedirs(log_dir, exist_ok=True)
    png_path = os.path.join(log_dir, "rest_zmap_diag.png")
    try:
        img.filepath_raw = png_path
        img.file_format = "PNG"
        img.save()
        log.info(f"[Diag] Z-map PNG saved → {png_path}")
    except Exception as exc:
        log.warning(f"[Diag] Z-map PNG save failed: {exc}")

    cut_px = int(np.sum(zmap < stock_z - 1e-9))
    log.info(
        f"[Diag] Z-map image 'rest_zmap_diag' ready in Blender Image Editor "
        f"({resx}×{resy}px, z=[{z_min:.4f} … {z_max:.4f}]). "
        f"Uncut={zmap.size - cut_px:,} ({100*(zmap.size-cut_px)/zmap.size:.1f}%) "
        f"Cut={cut_px:,} ({100*cut_px/zmap.size:.1f}%)"
    )



# ---------------------------------------------------------------------------
# Chunk sorting (rest-machining-local, scipy-free)
# ---------------------------------------------------------------------------


async def rest_sort_chunks(chunks, o, last_pos=None):
    """Sort chunks for rest machining using a numpy-vectorized nearest-neighbour.

    Equivalent to chunk_utils.sort_chunks but avoids the scipy dependency so it
    runs at full speed inside Blender's bundled Python.  Uses numpy vectorized
    distance calculations (O(n²) complexity but each step is a fast numpy op)
    which is typically 100-500× faster than the pure-Python O(n²) fallback.

    For chunk counts typical of rest machining (tens of thousands) this
    completes in a few seconds rather than 10+ minutes.
    """
    import sys
    from ..utilities.chunk_utils import connect_chunks_low
    from ..utilities.async_utils import progress_async

    log.info("-")
    _t0 = time.time()
    await progress_async("Sorting Paths")
    sys.setrecursionlimit(100000)

    total = len(chunks)
    log.debug(f"sort_chunks: {total} chunks, starting sort loop")
    last_progress_time = time.time()
    pos = (0, 0, 0) if last_pos is None else last_pos

    meander = o.movement.type == "MEANDER"

    # Pre-compute start (and optionally end) XY coords for all chunks.
    start_xy = np.empty((total, 2), dtype=np.float64)
    end_xy = np.empty((total, 2), dtype=np.float64) if meander else None
    valid = np.ones(total, dtype=bool)
    for ci, ch in enumerate(chunks):
        pts = ch.get_points()
        if not pts:
            valid[ci] = False
            continue
        start_xy[ci] = (pts[0][0], pts[0][1])
        if meander:
            end_xy[ci] = (pts[-1][0], pts[-1][1])

    remaining_mask = valid.copy()
    sortedchunks = []
    cur = np.array([pos[0], pos[1]], dtype=np.float64)

    while remaining_mask.any():
        ri = np.where(remaining_mask)[0]
        d_start = np.sum((start_xy[ri] - cur) ** 2, axis=1)
        if meander:
            d_end = np.sum((end_xy[ri] - cur) ** 2, axis=1)
            best_local = np.argmin(np.minimum(d_start, d_end))
        else:
            best_local = np.argmin(d_start)

        ci = ri[best_local]
        remaining_mask[ci] = False
        ch = chunks[ci]
        if not ch.sorted:
            ch.adapt_distance(pos, o)
            ch.sorted = True
        sortedchunks.append(ch)
        pts = ch.get_points()
        pos = pts[-1]
        cur[0] = pts[-1][0]
        cur[1] = pts[-1][1]

        if time.time() - last_progress_time > 0.1:
            await progress_async(
                "Sorting Paths", 100.0 * (total - int(remaining_mask.sum())) / total
            )
            last_progress_time = time.time()

    log.debug(
        f"sort_chunks: sort loop done in {time.time() - _t0:.3f}s, {len(sortedchunks)} sorted"
    )

    _t1 = time.time()
    sortedchunks = await connect_chunks_low(sortedchunks, o)
    log.debug(f"sort_chunks: connect_chunks_low done in {time.time() - _t1:.3f}s")

    sys.setrecursionlimit(1000)
    return sortedchunks


# ---------------------------------------------------------------------------
# Z-map filtering with correct layer/scan-line ordering
# ---------------------------------------------------------------------------


async def _ordered_filter_chunks(chunks, o, num_rest_layers):
    """Order chunks correctly and filter by Z-map inline.

    Chunks from sample_chunks arrive in (scan-line, layer) order.  This
    function groups them by layer_index — preserving the original scan-line
    order within each layer — and processes layers top-to-bottom.  Within each
    layer, alternate scan lines are reversed so the cutter follows a proper
    MEANDER (zigzag) pattern without any separate sort step.

    For each chunk, points are walked one-by-one against the Z-map.  Points
    already cleared by roughing are dropped; sub-chunk boundaries are created
    at cut/skip transitions.  Final-pass chunks (layer_index == num_rest_layers)
    are passed through unfiltered.

    Returns:
        list of CamPathChunk in (layer → scan-line → sub-chunk) order.
    """
    from collections import defaultdict
    from ..utilities.async_utils import progress_async

    zmap = getattr(o, "rest_zmap", None)          # 2-D numpy array of roughing stock heights, or None
    layer_height = getattr(o, "rest_layer_height", 0.001)  # depth per rest-machining layer (metres)
    prior_skin = getattr(o, "rest_prior_skin", 0.0)  # roughing skin (metres) — material intentionally left by rougher
    do_meander = o.movement.type == "MEANDER"      # True → alternate scan-line direction each pass

    # Skip threshold: prior skin + 5% of the rest cutter diameter.
    #
    # The noise floor (5% of Ø) filters out sampling artifacts — small
    # positive remaining values caused by pixel grid misalignment and
    # cutter geometry differences between the prior op and the rest op.
    # Real rest areas (corners the prior cutter couldn't reach) have
    # remaining well above this floor.
    #
    # This threshold is the SAME for all layers.  Previous versions used
    # (skin + layer_height) on upper layers, assuming deeper layers would
    # catch skipped points.  That assumption is wrong: sample_chunks only
    # includes a point in layers whose Z range contains the model surface.
    # A point whose model Z is above deeper layers is absent from them,
    # so if the upper layer skips it, nothing ever machines it.
    # For BALLCONE, use tip diameter (2×ball_radius) not body diameter
    if o.cutter_type == "BALLCONE" and getattr(o, "ball_radius", 0) > 0:
        effective_diameter = o.ball_radius * 2.0
    else:
        effective_diameter = o.cutter_diameter
    noise_floor = effective_diameter * 0.05  # 5% of effective cutter diameter
    skip_threshold = prior_skin + noise_floor

    # Minimum XY travel for a sub-chunk to be worth a retract-plunge cycle.
    # Must be small enough that angled scan lines crossing narrow rest
    # corridors aren't discarded.  Half the cutter diameter is roughly
    # the cutter radius — any segment shorter than that removes negligible
    # material.
    _min_cut_dist = effective_diameter * 0.5  # metres

    # Maximum consecutive skip points to bridge without splitting.
    # Short gaps (a few pixels) at the perimeter are bridged so adjacent
    # cut regions stay in one sub-chunk, avoiding retract-plunge chatter.
    _MAX_BRIDGE_PTS = 8

    stock_z = getattr(o, "rest_stock_z", 0.0)     # Z height of the stock surface (uncut pixels sit at this value)
    pixsize = minx = miny = coordoff = 0.0  # set below when zmap is available
    if zmap is not None:                           # zmap exists → set up pixel-space lookup params
        pixsize = o.optimisation.pixsize           # world-space size of one Z-map pixel (metres/pixel)
        minx, miny = o.min.x, o.min.y             # world-space origin of the Z-map (bottom-left corner)
        coordoff = o.borderwidth + pixsize / 2.0  # pixel offset: border width plus half-pixel centre shift

    # Group by layer_index, keeping original list order within each group so
    # that scan-line order from parallel_pattern / sample_chunks is preserved.
    by_layer: dict = defaultdict(list)
    for original_idx, ch in enumerate(chunks):     # walk every chunk, recording its position in the input list
        by_layer[getattr(ch, "layer_index", 0)].append((original_idx, ch))  # bucket by layer, store (original_position, chunk)

    _m2in = 39.3701  # metres to inches conversion

    skipped = cut = 0
    result = []

    num_layers = len(by_layer)
    for li, layer_idx in enumerate(sorted(by_layer.keys())):  # process layers in ascending order: layer 0 (shallowest) first
        pct = int(100 * li / max(num_layers, 1))
        await progress_async(f"Filtering Layer {li + 1}/{num_layers}", pct)
        layer_chunks = [ch for _, ch in sorted(by_layer[layer_idx])]  # restore scan-line order within this layer by sorting on original_position
        is_final_pass = (layer_idx == num_rest_layers)  # True → this is the unfiltered full-surface finishing pass
        is_deepest_rest = (layer_idx == num_rest_layers - 1)  # True → deepest rest clearing layer (no layer below)
        active_threshold = skip_threshold

        # Per-layer diagnostic counters
        _lyr_cut = 0          # points accepted as CUT
        _lyr_skip_thresh = 0  # skipped: remaining <= threshold
        _lyr_skip_bg = 0      # skipped: background (rougher never touched)
        _lyr_skip_oob = 0     # skipped: point outside Z-map bounds
        _lyr_skip_short = 0   # discarded: sub-chunk too short (< _min_cut_dist)
        _lyr_skip_gap = 0     # skipped: gap points (trailing or flushed)
        _lyr_final = 0        # unfiltered final-pass points
        _lyr_z_min = float('inf')   # track Z range of points in this layer
        _lyr_z_max = float('-inf')
        _lyr_rem_min = float('inf')  # track remaining range for filtered points
        _lyr_rem_max = float('-inf')
        _lyr_sub_chunks = 0   # sub-chunks emitted for this layer

        for scan_i, chunk in enumerate(layer_chunks):   # walk scan lines in their original parallel-pattern order
            pts = chunk.get_points()                     # returns list of (x, y, z) tuples for this scan line
            if not pts:                                  # chunk is empty (no geometry in this layer slice) → skip it
                continue

            # Reverse alternate scan lines for MEANDER direction.
            if do_meander and scan_i % 2 == 1:          # odd-numbered scan lines run in the opposite direction
                pts = list(reversed(pts))                # reverse the point list so the cutter zigzags back

            layer_idx_attr = getattr(chunk, "layer_index", 0)   # layer index to stamp onto any sub-chunks we create
            num_layers_attr = getattr(chunk, "num_layers", 1)   # total layer count to stamp onto any sub-chunks we create
            current_pts = []                             # accumulator for the current run of cut points
            gap_pts = []                                 # points in a skip gap (bridged if gap is short)

            for pt in pts:                               # walk every sample point along this scan line
                # Track Z range
                if pt[2] < _lyr_z_min:
                    _lyr_z_min = pt[2]
                if pt[2] > _lyr_z_max:
                    _lyr_z_max = pt[2]

                if is_final_pass or zmap is None:        # final pass cuts everywhere; no Z-map means cut everywhere
                    current_pts.append(pt)               # accept point unconditionally
                    cut += 1
                    _lyr_final += 1
                    continue

                xi = int(round((pt[0] - minx) / pixsize + coordoff))  # convert world X to Z-map column index
                yi = int(round((pt[1] - miny) / pixsize + coordoff))  # convert world Y to Z-map row index

                should_skip = False
                _skip_reason = None  # track reason for diagnostics
                if 0 <= xi < zmap.shape[0] and 0 <= yi < zmap.shape[1]:  # point falls inside the Z-map bounds
                    remaining = zmap[xi, yi] - pt[2]    # stock height at this XY minus finishing cut depth = material left to remove
                    if remaining < _lyr_rem_min:
                        _lyr_rem_min = remaining
                    if remaining > _lyr_rem_max:
                        _lyr_rem_max = remaining
                    # Background check: rougher never touched this pixel (zmap
                    # at stock height).  On upper layers this prevents plunging
                    # through full stock in one pass — the layers above will
                    # step down first.  On the deepest rest layer the upper
                    # layers have already cleared the stock, but the zmap
                    # doesn't reflect rest-layer cuts, so we must NOT skip
                    # background pixels here.
                    is_background = (
                        not is_deepest_rest
                        and zmap[xi, yi] >= stock_z - 1e-9
                        and pt[2] < stock_z - active_threshold
                    )
                    if remaining <= active_threshold:
                        should_skip = True
                        _skip_reason = "thresh"
                    elif is_background:
                        should_skip = True
                        _skip_reason = "bg"
                else:
                    should_skip = True
                    _skip_reason = "oob"

                if should_skip:
                    if _skip_reason == "thresh":
                        _lyr_skip_thresh += 1
                    elif _skip_reason == "bg":
                        _lyr_skip_bg += 1
                    elif _skip_reason == "oob":
                        _lyr_skip_oob += 1

                    if current_pts:
                        # We're in a cut run — buffer skip points as a gap.
                        # If the gap stays short we'll bridge it instead of
                        # splitting into two sub-chunks with a retract.
                        gap_pts.append(pt)
                        if len(gap_pts) > _MAX_BRIDGE_PTS:
                            # Gap too long — flush the sub-chunk.
                            _xy_dist = sum(
                                sqrt((current_pts[i+1][0]-current_pts[i][0])**2 +
                                     (current_pts[i+1][1]-current_pts[i][1])**2)
                                for i in range(len(current_pts)-1)
                            ) if len(current_pts) > 1 else 0.0
                            if _xy_dist >= _min_cut_dist:
                                sub = CamPathChunk(current_pts)
                                sub.layer_index = layer_idx_attr
                                sub.num_layers = num_layers_attr
                                result.append(sub)
                                _lyr_sub_chunks += 1
                            else:
                                _lyr_skip_short += len(current_pts)
                                skipped += len(current_pts)
                            current_pts = []
                            _lyr_skip_gap += len(gap_pts)
                            skipped += len(gap_pts)
                            gap_pts = []
                    else:
                        skipped += 1
                    continue

                # This is a CUT point.
                if gap_pts:
                    # Short gap followed by another cut — bridge it by
                    # including the gap points in the current run.
                    current_pts.extend(gap_pts)
                    cut += len(gap_pts)  # re-classify bridged points as cut
                    _lyr_cut += len(gap_pts)
                    gap_pts = []
                current_pts.append(pt)
                cut += 1
                _lyr_cut += 1

            # End of scan line — discard trailing gap, flush remaining points.
            _lyr_skip_gap += len(gap_pts)
            skipped += len(gap_pts)
            gap_pts = []
            _xy_dist = sum(
                sqrt((current_pts[i+1][0]-current_pts[i][0])**2 +
                     (current_pts[i+1][1]-current_pts[i][1])**2)
                for i in range(len(current_pts)-1)
            ) if len(current_pts) > 1 else 0.0
            if _xy_dist >= _min_cut_dist:
                sub = CamPathChunk(current_pts)
                sub.layer_index = layer_idx_attr
                sub.num_layers = num_layers_attr
                result.append(sub)
                _lyr_sub_chunks += 1
            elif current_pts:
                _lyr_skip_short += len(current_pts)
                skipped += len(current_pts)

        # --- Per-layer summary log ---
        _lyr_total = _lyr_cut + _lyr_skip_thresh + _lyr_skip_bg + _lyr_skip_oob + _lyr_skip_gap + _lyr_skip_short + _lyr_final
        _tag = "DEEPEST" if is_deepest_rest else ("FINAL" if is_final_pass else f"upper")
        log.info(
            f"[Filter] Layer {layer_idx} ({_tag}): "
            f"{_lyr_total:,} pts, {_lyr_sub_chunks} sub-chunks, "
            f"threshold={active_threshold * _m2in:.5f}\""
        )
        log.info(
            f"[Filter]   CUT: {_lyr_cut:,}  |  SKIP: "
            f"threshold={_lyr_skip_thresh:,}, bg={_lyr_skip_bg:,}, "
            f"short_chunk={_lyr_skip_short:,}, gap={_lyr_skip_gap:,}, oob={_lyr_skip_oob:,}"
            + (f", final_pass={_lyr_final:,}" if _lyr_final else "")
        )
        if _lyr_z_min <= _lyr_z_max:
            log.info(
                f"[Filter]   Z range: {_lyr_z_min * _m2in:.5f}\" .. {_lyr_z_max * _m2in:.5f}\"  |  "
                f"remaining range: {_lyr_rem_min * _m2in:.5f}\" .. {_lyr_rem_max * _m2in:.5f}\""
                if _lyr_rem_min <= _lyr_rem_max else
                f"[Filter]   Z range: {_lyr_z_min * _m2in:.5f}\" .. {_lyr_z_max * _m2in:.5f}\""
            )

    o.rest_stats_skipped = skipped   # store final skip count on the operation (used by _log_time_savings)
    o.rest_stats_cut = cut           # store final cut count on the operation (used by _log_time_savings)
    total = skipped + cut
    if total > 0:                    # avoid divide-by-zero if no points were processed
        pct = 100.0 * skipped / total
        log.info(
            f"[Filter] Threshold: {skip_threshold * _m2in:.5f}\" "
            f"(skin={prior_skin * _m2in:.5f}\" + 5%Ø={noise_floor * _m2in:.5f}\"), "
            f"layer_height={layer_height * _m2in:.5f}\""
        )
        log.info(
            f"[Filter] Total: {skipped:,} skipped ({pct:.1f}%), "
            f"{cut:,} cut ({100 - pct:.1f}%), "
            f"num_rest_layers={num_rest_layers}, total_layers={len(by_layer)}"
        )
    return result


# ---------------------------------------------------------------------------
# Silhouette boundary clipping
# ---------------------------------------------------------------------------


def _clip_chunks_to_silhouette(chunks, o):
    """Clip chunks to the part silhouette boundary.

    Removes points that fall outside the actual part outline (buffered by
    the cutter radius so the cutter can fully reach the part edge).  This
    prevents rest machining from cutting in stock-only areas beyond the
    part boundary where scallops serve no purpose.

    Chunks are split at boundary crossings and layer_index / num_layers
    attributes are preserved on the resulting sub-chunks.
    """
    from shapely import contains as shp_contains, points as shp_points
    from ..utilities.silhouette_utils import get_operation_silhouette

    silhouette = get_operation_silhouette(o)
    if silhouette is None or silhouette.is_empty:
        log.debug("[Rest Machining] No silhouette available — skipping boundary clip")
        return chunks

    # Buffer outward by the cutter radius so the cutter centre can reach
    # the very edge of the part (the cutter body extends beyond centre).
    buffer_dist = o.cutter_diameter / 2.0
    boundary = silhouette.buffer(
        buffer_dist, resolution=getattr(o.optimisation, "circle_detail", 64)
    )

    clipped = []
    removed = 0

    for ch in chunks:
        pts = ch.get_points_np()
        if len(pts) == 0:
            continue

        # Vectorised containment test (shapely 2.0)
        inside = shp_contains(boundary, shp_points(pts[:, 0:2]))

        # Walk points, splitting at inside/outside transitions.
        layer_idx = getattr(ch, "layer_index", 0)
        num_layers = getattr(ch, "num_layers", 1)
        current_pts = []

        for pt, is_inside in zip(pts, inside):
            if is_inside:
                current_pts.append(pt)
            else:
                removed += 1
                if current_pts:
                    sub = CamPathChunk(current_pts)
                    sub.layer_index = layer_idx
                    sub.num_layers = num_layers
                    clipped.append(sub)
                    current_pts = []

        if current_pts:
            sub = CamPathChunk(current_pts)
            sub.layer_index = layer_idx
            sub.num_layers = num_layers
            clipped.append(sub)

    if removed > 0:
        log.info(
            f"[Rest Machining] Silhouette clip: removed {removed:,} points "
            f"outside part boundary"
        )
    else:
        log.info("[Rest Machining] Silhouette clip: all points within part boundary")

    return clipped



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
# Terrain-aware rapid traverses
# ---------------------------------------------------------------------------
# Replaces full safe-height retracts between cuts with low-clearance
# traverses that fly just above the stock surface.  Uses the rest Z-map
# (seeded from the roughing pass) with a dilation/max filter to guarantee
# clearance even over scallops between roughing passes.
#
# The post-processor walks the finished mesh from chunks_to_mesh and
# rewrites only the rapid-traverse waypoints — cutting geometry is never
# touched.
#
# Enable via Custom Settings → Terrain Aware Rapids checkbox.
# ---------------------------------------------------------------------------


def _build_dilated_zmap(zmap, cutter_diameter, pixsize):
    """Apply a max-filter (dilation) to the Z-map.

    The raw Z-map records the deepest point reached by the rougher at each
    pixel.  Between roughing passes, scallops may be slightly higher than
    the deepest point.  The dilated Z-map records the HIGHEST stock surface
    in a neighbourhood around each pixel, giving a safe worst-case terrain
    height for rapid traverse clearance.

    The dilation radius is half the roughing cutter diameter (the maximum
    scallop distance from the roughing pass centre).

    Returns:
        numpy 2-D array same shape as zmap, with each pixel holding the
        max value in its neighbourhood.
    """
    t0 = time.time()

    # Dilation radius in pixels — half the cutter diameter
    radius_px = max(1, int(round(cutter_diameter / (2.0 * pixsize))))
    kernel = 2 * radius_px + 1

    # Use scipy if available (fast C implementation)
    try:
        from scipy.ndimage import maximum_filter
        dilated = maximum_filter(zmap, size=kernel)
        method = "scipy"
    except ImportError:
        # Separable max filter: two 1D passes (rows then columns).
        # Each 1D pass uses numpy stride_tricks for a sliding window view,
        # reducing O(n × kernel²) to O(n × 2) array operations.
        rows, cols = zmap.shape

        # Pass 1: max along axis 0 (rows) with padding
        pad0 = np.pad(zmap, ((radius_px, radius_px), (0, 0)),
                       mode='edge')
        # Sliding window view: shape (rows, cols, kernel)
        strides0 = pad0.strides
        view0 = np.lib.stride_tricks.as_strided(
            pad0,
            shape=(rows, cols, kernel),
            strides=(strides0[0], strides0[1], strides0[0]),
        )
        temp = view0.max(axis=2)

        # Pass 2: max along axis 1 (columns) with padding
        pad1 = np.pad(temp, ((0, 0), (radius_px, radius_px)),
                       mode='edge')
        strides1 = pad1.strides
        view1 = np.lib.stride_tricks.as_strided(
            pad1,
            shape=(rows, cols, kernel),
            strides=(strides1[0], strides1[1], strides1[1]),
        )
        dilated = view1.max(axis=2)
        method = "numpy-separable"

    elapsed = time.time() - t0
    log.info(
        f"[Terrain] Dilated Z-map: radius={radius_px}px, kernel={kernel}, "
        f"shape={dilated.shape}, method={method}, {elapsed * 1000:.1f}ms"
    )
    return dilated


def _max_terrain_z_along_line(dilated_zmap, ax, ay, bx, by,
                               pixsize, minx, miny, coordoff):
    """Sample the dilated Z-map along a straight line from A to B.

    Returns the maximum terrain Z value along the path, or -inf if
    no valid pixels are encountered.
    """
    resx, resy = dilated_zmap.shape

    # Number of samples: at least 1 per pixel along the longer axis
    dx = abs(bx - ax)
    dy = abs(by - ay)
    dist_px = max(dx, dy) / pixsize if pixsize > 0 else 1
    n_samples = max(2, int(dist_px) + 1)

    t = np.linspace(0.0, 1.0, n_samples)
    wx = ax + t * (bx - ax)
    wy = ay + t * (by - ay)

    xi = np.rint((wx - minx) / pixsize + coordoff).astype(np.int32)
    yi = np.rint((wy - miny) / pixsize + coordoff).astype(np.int32)

    valid = (xi >= 0) & (xi < resx) & (yi >= 0) & (yi < resy)
    if not valid.any():
        return -inf

    return float(np.max(dilated_zmap[xi[valid], yi[valid]]))


def _linearize_segment(p1, p2, step_mm=0.5):
    """Break a 3D line segment into short steps for smooth motion.

    Returns list of (x, y, z) tuples including p1 and p2.
    step_mm is in METRES (matching Blender internal units).
    """
    step = step_mm * 0.001  # mm to metres
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    dist = sqrt(dx * dx + dy * dy + dz * dz)
    if dist <= step:
        return [p1, p2]

    n = max(2, int(dist / step) + 1)
    pts = []
    for i in range(n + 1):
        t = i / n
        pts.append((
            p1[0] + t * dx,
            p1[1] + t * dy,
            p1[2] + t * dz,
        ))
    return pts


def _check_terrain_clear(dilated_zmap, ax, ay, az_start, az_end, bx, by,
                          pixsize, minx, miny, coordoff, clearance):
    """Check if a diagonal move from (ax,ay,az_start) to (bx,by,az_end)
    stays above terrain + clearance at all points along the path.

    The move is linear in XYZ — Z interpolates linearly between az_start
    and az_end while XY interpolates from A to B.
    """
    resx, resy = dilated_zmap.shape

    dx = abs(bx - ax)
    dy = abs(by - ay)
    dist_px = max(dx, dy) / pixsize if pixsize > 0 else 1
    n_samples = max(2, int(dist_px) + 1)

    t = np.linspace(0.0, 1.0, n_samples)
    wx = ax + t * (bx - ax)
    wy = ay + t * (by - ay)
    wz = az_start + t * (az_end - az_start)

    xi = np.rint((wx - minx) / pixsize + coordoff).astype(np.int32)
    yi = np.rint((wy - miny) / pixsize + coordoff).astype(np.int32)

    valid = (xi >= 0) & (xi < resx) & (yi >= 0) & (yi < resy)
    if not valid.any():
        return True  # no data — assume safe

    terrain_z = dilated_zmap[xi[valid], yi[valid]]
    cutter_z = wz[valid]

    # Check: cutter must be above terrain + clearance at every sample
    return bool(np.all(cutter_z >= terrain_z + clearance))


def terrain_rapids_postprocess(o):
    """Rewrite rapid traverses in the rest machining path mesh.

    Walks the mesh vertices and identifies lift/traverse/drop sequences
    (consecutive vertices at free_height).  For each traverse, queries the
    dilated Z-map to find terrain height and replaces the free_height
    waypoints with terrain-following waypoints:

        1. 45° angled climb from cut end to traverse height
           (terrain-checked — falls back to vertical if terrain blocks)
        2. Horizontal traverse at terrain_z + clearance
        3. 45° angled descent toward next cut start
           (terrain-checked — falls back to vertical if terrain blocks)
        4. Final 1mm descent at plunge speed (not rapid)

    The mesh is rebuilt with the new vertices.  A per-vertex 'rapid'
    attribute is stamped: 1 for rapid moves, 0 for plunge/cut moves.
    """
    import traceback as _tb
    t0 = time.time()
    try:
        return _terrain_rapids_impl(o)
    except Exception as e:
        log.warning(f"[Terrain] Post-processor CRASHED: {e}")
        log.warning(_tb.format_exc())


def _terrain_rapids_impl(o):
    """Internal implementation — separated so exceptions are caught and logged."""
    t0 = time.time()
    scene = bpy.context.scene
    machine = scene.cam_machine

    clearance = machine.terrain_clearance * 0.001  # mm → metres
    free_height = o.movement.free_height
    log.info(f"[Terrain] Starting: clearance={clearance*1000:.1f}mm, free_height={free_height*1000:.1f}mm")

    zmap = getattr(o, "rest_zmap", None)
    if zmap is None:
        log.warning("[Terrain] No Z-map available — skipping terrain rapids")
        return
    log.info(f"[Terrain] Z-map shape={zmap.shape}")

    pixsize = o.optimisation.pixsize
    minx, miny = o.min.x, o.min.y
    coordoff = o.borderwidth + pixsize / 2.0

    # Get the prior op's cutter diameter for dilation radius
    prior_op_name = getattr(o, "rest_machining_operation", "")
    prior_cutter_d = o.cutter_diameter  # fallback
    if prior_op_name:
        for op in scene.cam_operations:
            if op.name == prior_op_name:
                prior_cutter_d = op.cutter_diameter
                break

    # Build dilated Z-map
    dilated = _build_dilated_zmap(zmap, prior_cutter_d, pixsize)

    # Read the current mesh
    path_name = scene.cam_names.path_name_full
    mesh_obj = bpy.data.objects.get(path_name)
    if mesh_obj is None:
        log.warning("[Terrain] No path mesh found — skipping terrain rapids")
        return

    mesh = mesh_obj.data
    n_verts = len(mesh.vertices)
    if n_verts < 3:
        return

    # Bulk-read vertex positions
    coords = np.zeros(n_verts * 3, dtype=np.float64)
    mesh.vertices.foreach_get("co", coords)
    coords = coords.reshape(n_verts, 3)

    # Find vertices at free_height (rapid traverse waypoints)
    fh_eps = free_height * 0.001  # tolerance for float comparison
    at_fh = np.abs(coords[:, 2] - free_height) < fh_eps
    n_at_fh = int(at_fh.sum())
    log.info(f"[Terrain] Mesh: {n_verts} verts, {n_at_fh} at free_height ({free_height:.4f}m)")

    # Walk vertices, build new vertex list
    new_verts = []
    rapid_flags = []
    traverses_replaced = 0
    traverses_fallback = 0
    traverses_direct = 0   # short same-level moves with no obstruction
    traverses_minihop = 0  # short moves with small terrain ridge

    # Plunge transition: last 1mm of descent at plunge speed
    _PLUNGE_ZONE = 0.001  # 1mm in metres
    # Direct connection: skip lift entirely for short moves on same level
    _DIRECT_MAX_DIST = 0.005  # 5mm in metres
    _DIRECT_Z_TOL = 0.0005   # 0.5mm Z tolerance for "same level"
    # Mini-hop: short moves with terrain slightly above cut line
    _MINIHOP_MAX_DIST = 0.005  # 5mm in metres

    i = 0
    while i < n_verts:
        if not at_fh[i]:
            # Cutting point — pass through unchanged
            new_verts.append(tuple(coords[i]))
            rapid_flags.append(0)
            i += 1
            continue

        # Found a free_height vertex — collect the full traverse sequence
        # Pattern: ... cut_end, lift, [traverse...], drop, cut_start ...
        # lift/traverse/drop are all at free_height
        traverse_start = i
        while i < n_verts and at_fh[i]:
            i += 1
        traverse_end = i  # exclusive

        # We need the cut point before and after the traverse
        if traverse_start == 0 or traverse_end >= n_verts:
            # First or last segment — keep as free_height (origin/return)
            for j in range(traverse_start, traverse_end):
                new_verts.append(tuple(coords[j]))
                rapid_flags.append(1)
            continue

        cut_end = coords[traverse_start - 1]   # last cutting point
        cut_start = coords[traverse_end]        # next cutting point

        # Query terrain along the traverse XY path
        terrain_z = _max_terrain_z_along_line(
            dilated, cut_end[0], cut_end[1], cut_start[0], cut_start[1],
            pixsize, minx, miny, coordoff
        )

        if terrain_z == -inf:
            # No terrain data — keep free_height
            for j in range(traverse_start, traverse_end):
                new_verts.append(tuple(coords[j]))
                rapid_flags.append(1)
            traverses_fallback += 1
            continue

        # Direct connection: short move, same Z level, no terrain above
        # the higher cut point → skip lift entirely, connect at rapid.
        # The cutter is already at cutting depth; if terrain along the
        # path is below the cutter, there's nothing to collide with.
        dx = cut_start[0] - cut_end[0]
        dy = cut_start[1] - cut_end[1]
        xy_dist = sqrt(dx * dx + dy * dy)
        same_level = abs(cut_start[2] - cut_end[2]) < _DIRECT_Z_TOL
        higher_z = max(cut_end[2], cut_start[2])
        terrain_clear = higher_z >= terrain_z + clearance

        if xy_dist < _DIRECT_MAX_DIST and same_level and terrain_clear:
            # Direct rapid move — no lift needed
            new_verts.append(tuple(cut_start))
            rapid_flags.append(1)
            traverses_direct += 1
            continue

        traverse_z = terrain_z + clearance

        # Mini-hop: short move with terrain ridge between cut points.
        # Instead of the full 45° climb/traverse/descent, just do:
        #   up to traverse_z → across to above cut_start → down
        # Three rapid points, no linearization.
        if xy_dist < _MINIHOP_MAX_DIST:
            # Lift to terrain clearance
            new_verts.append((cut_end[0], cut_end[1], traverse_z))
            rapid_flags.append(1)
            # Move to above next cut point
            new_verts.append((cut_start[0], cut_start[1], traverse_z))
            rapid_flags.append(1)
            # Plunge to cut depth
            new_verts.append(tuple(cut_start))
            rapid_flags.append(0)  # plunge speed
            traverses_minihop += 1
            continue

        # If terrain traverse is not significantly lower than free_height,
        # not worth the complexity — keep free_height
        if traverse_z >= free_height - clearance:
            for j in range(traverse_start, traverse_end):
                new_verts.append(tuple(coords[j]))
                rapid_flags.append(1)
            traverses_fallback += 1
            continue

        # --- Build terrain-aware waypoints ---

        # Climb: 45° angle from cut_end up to traverse_z
        # Check terrain along the 45° climb path
        dz_climb = traverse_z - cut_end[2]
        climb_xy_dist = abs(dz_climb)  # 45° means equal XY and Z distance
        if climb_xy_dist > 0:
            # Direction from cut_end toward cut_start
            full_dx = cut_start[0] - cut_end[0]
            full_dy = cut_start[1] - cut_end[1]
            full_dist = sqrt(full_dx * full_dx + full_dy * full_dy)
            if full_dist > 0:
                dir_x = full_dx / full_dist
                dir_y = full_dy / full_dist
            else:
                dir_x, dir_y = 0.0, 0.0

            # Climb endpoint
            climb_end_x = cut_end[0] + dir_x * climb_xy_dist
            climb_end_y = cut_end[1] + dir_y * climb_xy_dist

            # Check if 45° climb is terrain-safe
            climb_safe = _check_terrain_clear(
                dilated, cut_end[0], cut_end[1], cut_end[2], traverse_z,
                climb_end_x, climb_end_y,
                pixsize, minx, miny, coordoff, clearance
            )
        else:
            climb_safe = True
            climb_end_x = cut_end[0]
            climb_end_y = cut_end[1]

        if not climb_safe:
            # Vertical climb instead
            climb_end_x = cut_end[0]
            climb_end_y = cut_end[1]

        # Linearize climb
        climb_pts = _linearize_segment(
            (cut_end[0], cut_end[1], cut_end[2]),
            (climb_end_x, climb_end_y, traverse_z),
            step_mm=1.0
        )
        # Skip first point (it's cut_end, already in the list)
        for pt in climb_pts[1:]:
            new_verts.append(pt)
            rapid_flags.append(1)

        # Horizontal traverse at traverse_z
        # Descent will start from near cut_start — compute descent endpoint
        dz_descent = traverse_z - cut_start[2]
        descent_xy_dist = abs(dz_descent)  # 45° descent
        if descent_xy_dist > 0 and full_dist > 0:
            # Descent start: back from cut_start by descent_xy_dist
            descent_start_x = cut_start[0] - dir_x * descent_xy_dist
            descent_start_y = cut_start[1] - dir_y * descent_xy_dist

            # Check if 45° descent is terrain-safe
            descent_safe = _check_terrain_clear(
                dilated, descent_start_x, descent_start_y, traverse_z,
                cut_start[2],
                cut_start[0], cut_start[1],
                pixsize, minx, miny, coordoff, clearance
            )
        else:
            descent_safe = True
            descent_start_x = cut_start[0]
            descent_start_y = cut_start[1]

        if not descent_safe:
            # Vertical descent instead
            descent_start_x = cut_start[0]
            descent_start_y = cut_start[1]

        # Horizontal traverse: from climb end to descent start
        # No linearization needed — cutter is at safe clearance, just
        # emit the endpoint (start point is already the climb end).
        horiz_end = (descent_start_x, descent_start_y, traverse_z)
        new_verts.append(horiz_end)
        rapid_flags.append(1)

        # Descent: 45° angle from traverse_z down to plunge zone
        plunge_start_z = cut_start[2] + _PLUNGE_ZONE
        if plunge_start_z >= traverse_z:
            plunge_start_z = cut_start[2]  # already close enough

        # Rapid descent to plunge zone
        descent_pts = _linearize_segment(
            (descent_start_x, descent_start_y, traverse_z),
            (cut_start[0], cut_start[1], plunge_start_z),
            step_mm=1.0
        )
        for pt in descent_pts[1:]:
            new_verts.append(pt)
            rapid_flags.append(1)

        # Final plunge: plunge_start_z → cut_start at plunge speed
        if plunge_start_z > cut_start[2] + 1e-6:
            plunge_pts = _linearize_segment(
                (cut_start[0], cut_start[1], plunge_start_z),
                (cut_start[0], cut_start[1], cut_start[2]),
                step_mm=0.25
            )
            for pt in plunge_pts[1:]:
                new_verts.append(pt)
                rapid_flags.append(0)  # plunge speed, not rapid

        traverses_replaced += 1

    # --- Rebuild the mesh ---
    n_new = len(new_verts)
    if n_new == 0:
        return

    new_mesh = bpy.data.meshes.new(f"{path_name}_terrain")
    edges = [(i, i + 1) for i in range(n_new - 1)]
    new_mesh.from_pydata(new_verts, edges, [])

    # Stamp rapid attribute
    attr = new_mesh.attributes.new("rapid", "INT", "POINT")
    attr.data.foreach_set("value", rapid_flags)

    # Replace the mesh on the path object
    old_mesh = mesh_obj.data
    mesh_obj.data = new_mesh
    bpy.data.meshes.remove(old_mesh)

    elapsed = time.time() - t0
    log.info(
        f"[Terrain] Rapids post-processed in {elapsed * 1000:.1f}ms: "
        f"{traverses_direct} direct, {traverses_minihop} mini-hop, "
        f"{traverses_replaced} terrain, {traverses_fallback} fallback, "
        f"{n_verts} → {n_new} vertices, "
        f"clearance={clearance * 1000:.1f}mm"
    )


# ---------------------------------------------------------------------------
# Rest machining G-code exporter (forked from gcode_export.py)
# ---------------------------------------------------------------------------


def rest_export_gcode_path(filename, vertslist, operations):
    """v6 legacy — Export G-code for rest machining paths.

    Forked from ``gcode_export.export_gcode_path`` to use the per-vertex
    ``rapid`` attribute on the path mesh instead of the ``v.z >= free_height``
    heuristic.  All path points are kept (no geometry removed) — the attribute
    controls whether each segment is emitted as G0 (rapid) or G1 (feed).
    """
    from importlib import import_module
    from math import pi

    from mathutils import Euler, Vector

    from .. import __package__ as base_package
    from ..constants import IMPERIAL_CORRECTION, METRIC_CORRECTION, ROTATION_CORRECTION
    from ..post_processors import iso
    from ..utilities.compare_utils import point_on_line
    from ..utilities.simple_utils import safe_filename, unit_value_to_string

    t = time.time()
    s = bpy.context.scene
    m = s.cam_machine

    enable_dust = False
    enable_hold = False
    enable_mist = False
    split = False
    totops = 0
    findex = 0

    if m.eval_splitting:
        for mesh in vertslist:
            totops += len(mesh.vertices)
        if totops > m.split_limit:
            split = True

    if s.cam_names.default_export_location == "":
        filepath = bpy.data.filepath
        basename = bpy.path.basename
        basefilename = filepath[: -len(basename(filepath))] + safe_filename(filename)
    else:
        basefilename = s.cam_names.default_export_location + safe_filename(filename)

    processor_extension = {
        "ANILAM": ("anilam_crusader_m", ".tap"),
        "CENTROID": ("centroid1", ".tap"),
        "EMC": ("emc2b", ".ngc"),
        "FADAL": ("fadal", ".tap"),
        "GRAVOS": ("gravos", ".nc"),
        "GRBL": ("grbl", ".gcode"),
        "HM50": ("hm50", ".tap"),
        "HEIDENHAIN": ("heiden", ".H"),
        "HEIDENHAIN530": ("heiden530", ".H"),
        "ISO": ("iso", ".tap"),
        "LYNX_OTTER_O": ("lynx_otter_o", ".nc"),
        "MACH3": ("mach3", ".tap"),
        "SHOPBOT MTC": ("shopbot_mtc", ".sbp"),
        "SIEGKX1": ("siegkx1", ".tap"),
        "TNC151": ("tnc151", ".tap"),
        "USER": ("user", ".gcode"),
        "WIN-PC": ("winpc", ".din"),
    }

    module = f".post_processors.{processor_extension[m.post_processor][0]}"
    postprocessor = import_module(module, base_package)
    extension = processor_extension[m.post_processor][1]

    unit_system = s.unit_settings.system
    unitcorr = (
        METRIC_CORRECTION if unit_system == "METRIC"
        else IMPERIAL_CORRECTION if unit_system == "IMPERIAL"
        else 1
    )
    rotcorr = ROTATION_CORRECTION

    def start_new_file():
        fileindex = "_" + str(findex) if split else ""
        fname = basefilename + fileindex + extension
        log.info(f"[Rest Export] Writing: {fname}")
        c = postprocessor.Creator()
        if isinstance(c, iso.Creator):
            c.output_block_numbers = m.output_block_numbers
            c.start_block_number = m.start_block_number
            c.block_number_increment = m.block_number_increment
        c.output_tool_definitions = m.output_tool_definitions
        c.output_tool_change = m.output_tool_change
        c.output_G43_on_tool_change_line = m.output_G43_on_tool_change
        c.file_open(fname)
        if unit_system == "METRIC":
            c.metric()
        elif unit_system == "IMPERIAL":
            c.imperial()
        c.program_begin(0, fname)
        c.flush_nc()
        c.comment("G-code Generated with Fabex and NC library (Rest Machining)")
        frf = round(m.feedrate_rapid * unitcorr, 2)
        c.comment(f"Rapids: {frf}/min")
        c.absolute()
        c.set_plane(0)
        c.flush_nc()
        return c

    c = start_new_file()
    last_cutter = None
    processedops = 0
    last = Vector((0, 0, 0))
    cut_distance = 0
    mill_distance = 0
    mill_duration = 0

    for i, o in enumerate(operations):
        if o.output_header:
            for aline in o.gcode_header.split(";"):
                c.write(aline + "\n")

        free_height = o.movement.free_height
        if o.movement.useG64:
            c.set_path_control_mode(2, round(o.movement.G64 * 1000, 5), 0)

        mesh = vertslist[i]
        verts = mesh.vertices[:]

        # Bulk-read the rapid attribute into a numpy array for O(1) lookup.
        n_verts = len(verts)
        has_rapid_attr = "rapid" in mesh.attributes
        if has_rapid_attr:
            rapid = np.zeros(n_verts, dtype=np.int32)
            mesh.attributes["rapid"].data.foreach_get("value", rapid)
            log.debug(
                f"[Rest Export] Rapid attribute: {np.count_nonzero(rapid):,} of "
                f"{n_verts:,} vertices flagged as rapid"
            )
        else:
            rapid = np.zeros(n_verts, dtype=np.int32)
            log.warning("[Rest Export] No 'rapid' attribute — all moves will be G1")

        if o.machine_axes != "3":
            rots = mesh.shape_keys.key_blocks["rotations"].data

        spdir_clockwise = o.movement.spindle_rotation == "CW"

        if m.output_tool_change and last_cutter != [
            o.cutter_id, o.cutter_diameter, o.cutter_type, o.cutter_flutes,
        ]:
            c.tool_change(o.cutter_id)
        if m.output_tool_definitions:
            c.comment(
                "Tool: D = %s type %s flutes %s"
                % (unit_value_to_string(o.cutter_diameter, 4), o.cutter_type, o.cutter_flutes)
            )
        c.flush_nc()
        last_cutter = [o.cutter_id, o.cutter_diameter, o.cutter_type, o.cutter_flutes]

        fmh = round(free_height * unitcorr, 2)

        if o.cutter_type not in ["LASER", "PLASMA"]:
            frf = round(m.feedrate_rapid * unitcorr, 2)
            if m.use_job_start_height:
                jsh = round(m.job_start_height * unitcorr, 2)
                c.write(f"G00 Z{jsh} F{frf}\n")
            if o.enable_hold:
                c.write("(Hold Down)\n")
                for aline in o.gcode_start_hold_cmd.split(";"):
                    c.write(aline + "\n")
                enable_hold = True
                stop_hold = o.gcode_stop_hold_cmd
            if o.enable_mist:
                c.write("(Mist)\n")
                for aline in o.gcode_start_mist_cmd.split(";"):
                    c.write(aline + "\n")
                enable_mist = True
                stop_mist = o.gcode_stop_mist_cmd
            c.spindle(o.spindle_rpm, spdir_clockwise)
            c.write_spindle()
            c.flush_nc()
            c.write("\n")
            if o.enable_dust:
                c.write("(Dust Collector)\n")
                for aline in o.gcode_start_dust_cmd.split(";"):
                    c.write(aline + "\n")
                enable_dust = True
                stop_dust = o.gcode_stop_dust_cmd

        if m.spindle_start_time > 0:
            c.dwell(m.spindle_start_time)

        if o.cutter_type not in ["LASER", "PLASMA"] and not m.use_job_start_height:
            c.write(f"G00 Z{fmh} F{frf}\n")

        if o.enable_a_axis:
            if o.rotation_a == 0:
                o.rotation_a = 0.0001
            c.rapid(a=o.rotation_a * 180 / pi)
        if o.enable_b_axis:
            if o.rotation_b == 0:
                o.rotation_b = 0.0001
            c.rapid(a=o.rotation_b * 180 / pi)

        c.write("\n")
        c.flush_nc()

        m = bpy.context.scene.cam_machine
        # o.feedrate has unit="LENGTH" → stored in m/min by Blender.
        # m.feedrate_max/min have NO unit="LENGTH" → stored in display units
        # (in/min or mm/min).  Convert o.feedrate to display units for
        # comparison, then apply unitcorr at the end.
        o_feedrate_display = o.feedrate * unitcorr  # m/min → in/min or mm/min
        millfeedrate = min(o_feedrate_display, m.feedrate_max)
        millfeedrate = max(millfeedrate, m.feedrate_min)
        plungefeedrate = millfeedrate * o.plunge_feedrate / 100
        freefeedrate = m.feedrate_max

        fadjust = False
        if (
            o.do_simulation_feedrate
            and mesh.shape_keys is not None
            and mesh.shape_keys.key_blocks.find("feedrates") != -1
        ):
            shapek = mesh.shape_keys.key_blocks["feedrates"]
            fadjust = True

        if m.use_position_definitions:
            last = Vector((m.starting_position.x, m.starting_position.y, m.starting_position.z))

        lastrot = Euler((0, 0, 0))
        duration = 0.0
        f = 0.1123456
        fadjustval = 1
        downvector = Vector((0, 0, -1))
        plungelimit = pi / 2 - o.plunge_angle
        scale_graph = 0.05
        job_start_pending = m.use_job_start_height and o.cutter_type not in ["LASER", "PLASMA"]

        for vi, vert in enumerate(verts):
            if i > 0 and vi == 0:
                continue
            if job_start_pending and vi == 0:
                continue
            v = vert.co

            if o.remove_redundant_points and o.strategy != "DRILL":
                # Redundant point removal is intentionally DISABLED for rest
                # machining — every point must be kept to avoid crashes.
                pass

            if o.machine_axes != "3":
                v = v.copy()
                r = Euler(rots[vi].co)
                rcompensate = r.copy()
                rcompensate.x = -r.x
                rcompensate.y = -r.y
                rcompensate.z = -r.z
                v.rotate(rcompensate)
                ra = None if r.x == lastrot.x else r.x * rotcorr
                rb = None if r.y == lastrot.y else r.y * rotcorr

            vx = None if vi > 0 and v.x == last.x else v.x * unitcorr
            vy = None if vi > 0 and v.y == last.y else v.y * unitcorr
            vz = None if vi > 0 and v.z == last.z else v.z * unitcorr

            if fadjust:
                fadjustval = shapek.data[vi].co.z / scale_graph

            vect = v - last
            l = vect.length

            # --- Feedrate decision (rest machining version) ---
            # Priority 1: plunge detection (steep downward move)
            if vi > 0 and l > 0 and downvector.angle(vect) < plungelimit:
                if f != plungefeedrate or (fadjust and fadjustval != 1):
                    f = plungefeedrate * fadjustval
                    c.feedrate(f)
                if o.machine_axes == "3":
                    c.feed(x=vx, y=vy, z=vz)
                else:
                    c.feed(x=vx, y=vy, z=vz, a=ra, b=rb)

            # Priority 2: rapid flag from rest machining Z-map analysis
            elif rapid[vi]:
                if f != freefeedrate:
                    f = freefeedrate
                    c.feedrate(f)
                if o.machine_axes == "3":
                    c.rapid(x=vx, y=vy, z=vz)
                else:
                    c.rapid(x=vx, y=vy, z=vz, a=ra, b=rb)

            # Priority 3: normal mill feed
            else:
                if f != millfeedrate or (fadjust and fadjustval != 1):
                    f = millfeedrate * fadjustval
                    c.feedrate(f)
                if o.machine_axes == "3":
                    c.feed(x=vx, y=vy, z=vz)
                else:
                    c.feed(x=vx, y=vy, z=vz, a=ra, b=rb)

            d = vect.length * unitcorr
            cut_distance += d
            duration += vect.length / f if f > 0 else 0
            if not rapid[vi]:
                mill_distance += d
                mill_duration += vect.length / f if f > 0 else 0
            last = v

            if o.machine_axes != "3":
                lastrot = r

            processedops += 1
            if split and processedops > m.split_limit:
                c.rapid(x=last.x * unitcorr, y=last.y * unitcorr, z=free_height * unitcorr)
                c.program_end()
                findex += 1
                c.file_close()
                c = start_new_file()
                c.flush_nc()
                c.comment(
                    f"Tool change - D = {unit_value_to_string(o.cutter_diameter, 4)} "
                    f"type {o.cutter_type} flutes {o.cutter_flutes}"
                )
                c.tool_change(o.cutter_id)
                c.spindle(o.spindle_rpm, spdir_clockwise)
                c.write_spindle()
                c.flush_nc()
                if m.spindle_start_time > 0:
                    c.dwell(m.spindle_start_time)
                    c.flush_nc()
                c.feedrate(unitcorr * o.feedrate)
                c.rapid(x=last.x * unitcorr, y=last.y * unitcorr, z=free_height * unitcorr)
                c.rapid(x=last.x * unitcorr, y=last.y * unitcorr, z=last.z * unitcorr)
                processedops = 0

        c.feedrate(unitcorr * o.feedrate)

        if o.output_trailer:
            for aline in o.gcode_trailer.split(";"):
                c.write(aline + "\n")

    o.info.duration = duration * unitcorr
    total_min = duration * unitcorr
    rapid_distance = cut_distance - mill_distance
    rapid_pct = (100 * rapid_distance / cut_distance) if cut_distance > 0 else 0
    log.info(
        f"[Rest Export] Est. machining time: {round(total_min * 60)}s ({total_min:.1f} min)"
    )
    log.info(
        f"[Rest Export] Distance: {round(mill_distance, 1)} cut + "
        f"{round(rapid_distance, 1)} rapid ({rapid_pct:.1f}%) = "
        f"{round(cut_distance, 1)} total (inches)"
    )

    if enable_dust:
        c.write(stop_dust + "\n")
    if enable_hold:
        c.write(stop_hold + "\n")
    if enable_mist:
        c.write(stop_mist + "\n")

    c.program_end()
    c.file_close()
    log.info(f"[Rest Export] Done in {time.time() - t:.2f}s")


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------


_REST_MACHINING_VERSION = "2026-03-28-v8"


async def parallel(o):
    """Rest machining parallel strategy (v7 — filter-first, two-phase).

    Phase 1 — Rest clearing (upper layers):
        Z-map filtered sub-chunks, nearest-neighbor sorted for speed.
        Clears material the roughing pass left behind on intermediate layers.

    Phase 2 — Finishing pass (final layer):
        Full unfiltered parallel pass, standard sort with connect_chunks_low
        merging scan lines into continuous meander — identical to the standard
        parallel strategy.  This is the surface the user sees.

    Both phases are concatenated into a single mesh/G-code file with a
    safe-height lift separating them.

    Assumes ``validate_and_seed`` has already been called by
    path_ops._calc_path so that ``o.rest_zmap`` is populated.
    """
    from ..bridges import use_bridges
    from ..utilities.chunk_utils import sort_chunks

    log.info(f"~ Strategy: Rest Machining (Parallel) v{_REST_MACHINING_VERSION} ~")

    # Read diagnostic flags from operation properties
    diag_images, diag_verbose = _get_diag_flags(o)

    # Force layers on using rest_layer_height so sample_chunks generates
    # layered chunks with correct layer_index attributes.
    saved_use_layers = o.use_layers
    saved_stepdown = o.stepdown
    o.use_layers = True
    o.stepdown = getattr(o, "rest_layer_height", o.stepdown)

    pathSamples = parallel_pattern(o, o.parallel_angle)
    layers = get_layers(o, o.max_z, o.min.z)

    log.info(f"Sampling Object: {o.name}")
    chunks = list(await sample_chunks(o, pathSamples, layers))
    log.info(f"Sampling Finished: {len(chunks)} chunks from {len(layers)} layers")

    # --- v7: Filter out cleared points BEFORE sorting ---
    # When prior_skin=0 (detail pass), Phase 2 is skipped, so ALL layers
    # must be rest layers — otherwise the deepest rest layer's bottom is
    # one layer_height above min.z, and points clamped to that bottom
    # never reach the actual model surface.  With prior_skin>0, Phase 2
    # generates a fresh finishing pass that covers the full depth, so the
    # last layer can safely be treated as a discardable final pass.
    prior_skin = getattr(o, "rest_prior_skin", 0.0)
    if prior_skin <= 1e-9:
        num_rest_layers = len(layers)      # all layers are rest layers
    else:
        num_rest_layers = len(layers) - 1  # last layer = final pass (discarded)
    chunks = await _ordered_filter_chunks(chunks, o, num_rest_layers)
    log.info(f"After Z-map filtering: {len(chunks)} sub-chunks")

    # Clip to part silhouette — prevents machining outside the actual part
    # boundary (eliminates scallops in stock-only areas).
    chunks = _clip_chunks_to_silhouette(chunks, o)
    log.info(f"After silhouette clip: {len(chunks)} sub-chunks")

    # --- v7: Rest clearing uses only the upper (filtered) layers ---
    # The final layer from rest sampling is discarded; Phase 2 generates
    # a fresh standard parallel pass at full depth instead.
    upper = [ch for ch in chunks if getattr(ch, "layer_index", 0) < num_rest_layers]

    # Phase 1: Rest clearing
    if upper:
        log.info(f"Phase 1 — Rest clearing: {len(upper)} sub-chunks")

        # For detail passes (prior skin=0), use island-aware sorting
        # to reduce lifting on multi-layer pockets.
        prior_skin = getattr(o, "rest_prior_skin", 0.0)
        _used_islands = False
        if prior_skin <= 1e-9:
            try:
                log.info("Phase 1 — Island-aware detail sort")
                _island_result = await _sort_islands_detail(upper, o)
                if _island_result is not None:
                    upper = _island_result
                    _used_islands = True
            except Exception as e:
                log.warning(f"[Rest Islands] Failed, falling back to standard: {e}")

        if not _used_islands:
            # Standard layer-by-layer sort (shallowest first),
            # nearest-neighbor within each layer.
            from collections import defaultdict as _defaultdict
            _layer_groups = _defaultdict(list)
            for ch in upper:
                _layer_groups[getattr(ch, "layer_index", 0)].append(ch)
            upper_sorted = []
            _pos = (0, 0, 0)
            _saved_merge = o.movement.merge_distance
            o.movement.merge_distance = max(
                _saved_merge, o.cutter_diameter * 3.0
            )
            for _li in sorted(_layer_groups.keys()):
                _layer_chunks = _layer_groups[_li]
                log.debug(f"  Sorting layer {_li}: {len(_layer_chunks)} sub-chunks")
                _sorted = await rest_sort_chunks(_layer_chunks, o, last_pos=_pos)
                if _sorted:
                    _pos = _sorted[-1].get_point(-1)
                upper_sorted.extend(_sorted)
            o.movement.merge_distance = _saved_merge
            upper = upper_sorted
    else:
        log.info("Phase 1 — No upper-layer material to clear")

    # Phase 2: Standard parallel finishing pass — single layer at full depth.
    # Skip if the prior operation had no skin (skin=0), meaning it was already
    # a finishing pass.  In that case, only the rest clearing (Phase 1) is
    # needed — the surface is already finished everywhere the prior cutter
    # could reach.  This enables detail-pass chaining: rough → finish → fine
    # detail, where each pass only cuts the shrinking areas the previous
    # cutter couldn't physically fit.
    prior_skin = getattr(o, "rest_prior_skin", 0.0)
    final = []
    if prior_skin > 1e-9:
        o.use_layers = False
        log.info("Phase 2 — Generating standard parallel finishing pass (single layer)")
        finish_samples = parallel_pattern(o, o.parallel_angle)
        finish_layers = get_layers(o, o.max_z, o.min.z)
        final = list(await sample_chunks(o, finish_samples, finish_layers))
        log.info(f"Phase 2 — Sampled {len(final)} finishing chunks from {len(finish_layers)} layers")
        final = _clip_chunks_to_silhouette(final, o)
        final = await sort_chunks(final, o)
    else:
        log.info(
            "Phase 2 — Skipped (prior op skin=0, already a finishing pass — "
            "rest clearing only)"
        )

    log.info(
        f"After sorting: {len(upper)} upper + {len(final)} final "
        f"= {len(upper) + len(final)} total chunks"
    )

    # Concatenate: rest clearing first, then finishing pass
    chunks = upper + final

    if o.movement.ramp:
        for ch in chunks:
            ch.ramp_zig_zag(ch.zstart, None, o)

    if o.use_bridges:
        for bridge_chunk in chunks:
            use_bridges(bridge_chunk, o)

    chunks_to_mesh(chunks, o)

    # Terrain-aware rapids — post-process the mesh to replace full
    # safe-height retracts with low-clearance traverses.
    try:
        machine = bpy.context.scene.cam_machine
        if getattr(machine, "terrain_aware_rapids", False):
            log.info("[Terrain] Terrain-aware rapids enabled — running post-processor")
            terrain_rapids_postprocess(o)
        else:
            log.info("[Terrain] Terrain-aware rapids disabled — using standard free_height")
    except Exception as e:
        import traceback
        log.warning(f"[Terrain] Post-processing failed: {e}")
        log.warning(traceback.format_exc())

    # Restore original operation properties
    o.use_layers = saved_use_layers
    o.stepdown = saved_stepdown
