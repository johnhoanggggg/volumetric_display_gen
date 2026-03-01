import bpy
import math
import random
from mathutils import Vector
from mathutils.bvhtree import BVHTree

# ===================================================================
# PROJECTOR FOV CALIBRATION & ALIGNMENT SCRIPT
# ===================================================================
# For Nebra AnyBeam 720p laser scanning projector
#
# TWO-PART TOOL:
#   Part 1 — FOV RULER: Measures projector HFOV/VFOV to 0.1 deg.
#   Part 2 — ALIGNMENT PATTERN: Aligns projector pose to Blender camera.
#
# ADDITIONAL FEATURES:
#   Depth Probes      — Off-plane points to verify FOV, not just position.
#   Vol Test Regions   — Volumetric display test patches across the FOV.
#   Cluster Tests      — Different point clustering densities per voxel
#                        (per Nayar & Anand, Columbia CUCS-030-06, 2006).
#
# -------------------------------------------------------------------
# PART 1: FOV RULER
#
#   Two modes controlled by CALIB_WINDOW_FRACTION:
#
#   FULL FRAME (CALIB_WINDOW_FRACTION = 1.0):
#     Project full-white 1280x720. Ticks are at the projector's cone
#     edge. Glass must be large enough to intercept edge rays.
#
#   WINDOWED (CALIB_WINDOW_FRACTION < 1.0, e.g. 0.3):
#     Project a generated calibration PNG (white rectangle on black).
#     The IMAGE edge creates the lit/dark boundary — not the projector
#     cone. Works with any block size at any distance.
#     The script generates the PNG automatically.
#
#   Both modes: tick labels are real HFOV/VFOV values. Read them the
#   same way — find the last glowing tick, that's your FOV.
#
#   SETUP: Set Blender camera FOV WIDER than the projector's max
#   possible FOV. If you think HFOV is 37.5-39.1, set camera to ~42 deg.
#
#   TICK HEIGHT ENCODING:
#     .0 (whole degree) : 40px tall  — tallest landmark
#     .5 (half degree)  : 28px tall  — mid-point marker
#     other tenths      : 14px tall  — short ticks
#
#     .0   .1  .2  .3  .4  .5   .6  .7  .8  .9  next .0
#     TALL  s   s   s   s  MED   s   s   s   s   TALL
#
#   Max 4 short ticks from any landmark. Last lit = 2 past TALL → x.2.
#
#   AnyBeam laser scanning gives crisp edge cutoff (no vignetting).
#
# -------------------------------------------------------------------
# PART 2: ALIGNMENT PATTERN
#
#   SETUP: Blender camera can stay wider than projector FOV.
#   The alignment border maps to the projector's edge pixels at
#   PROJECTOR_HFOV_DEG, not the Blender camera edges.
#
#   FEATURES:
#     Corner dots    — Verify X/Y position + FOV at projector edges.
#     Center cross   — Verify pointing direction (yaw + pitch).
#     Edge midpoints — Verify no roll. All 4 should be symmetric.
#     Border frame   — With gaps where the FOV ruler crosses.
#     Sparse grid    — Interior dots. Catch distortion or local error.
#     Depth probes   — Off-plane points between grid dots. Only the
#                      correct FOV will illuminate all probes + grid
#                      simultaneously (parallax disambiguation).
#
# -------------------------------------------------------------------
# DEPTH PROBES — FOV VERIFICATION
#
#   If all calibration points are coplanar, any FOV will work if you
#   position the projector at the right distance. But depth-varied
#   points introduce parallax: a wrong-FOV projector can match the
#   on-plane grid dots by adjusting distance, but the off-plane probes
#   will shift laterally. Only the correct FOV illuminates everything.
#
#   Probes are placed at pixel locations between alignment grid dots,
#   at alternating front/back depths (checkerboard pattern).
#
# -------------------------------------------------------------------
# VOLUMETRIC TEST REGIONS
#
#   Larger volumetric display test patches positioned above the bottom
#   VFOV ruler, spread across the projector's horizontal FOV. Each
#   region uses a different HFOV (from VOL_TEST_FOV_MIN to
#   VOL_TEST_FOV_MAX) so you can see which FOV assumption produces
#   correct alignment. Uses the projector's actual pixel step and
#   1 point per ray at random depth.
#
# -------------------------------------------------------------------
# CLUSTER TESTS — NUCLEUS/ELECTRON MODEL (Nayar & Anand, 2006)
#
#   Each test "atom" has a nucleus point (single fracture at mid-depth
#   on the projector ray) surrounded by electron points distributed on
#   a sphere shell around it.  Tests whether more micro-fractures per
#   voxel increase perceived brightness of the holographic point.
#   Compare brightness vs. diffusion tradeoffs empirically.
#
# -------------------------------------------------------------------
# WORKFLOW:
#   1. Set Blender camera to ~42 deg HFOV (wider than projector)
#   2. Set CALIB_WINDOW_FRACTION:
#        Large block (covers full projection) → 1.0
#        Small block (cheaper) → 0.3 to 0.5
#   3. Run with GENERATE_FOV_RULER=True, GENERATE_ALIGNMENT=False
#   4. Etch the DXF into glass
#   5. If windowed: project the generated CalibrationImage.png
#      If full frame: project full white 1280x720
#   6. Read the last glowing tick → that is your HFOV/VFOV
#   7. Set PROJECTOR_HFOV_DEG to measured value (e.g. 37.6)
#      (Blender camera can stay wider — alignment maps to projector FOV)
#   8. Run with GENERATE_ALIGNMENT=True (and depth probes, clusters, etc.)
#   9. Etch → align projector using test images
#
# ===================================================================

# -------------------------------------------------------------------
# CONFIGURATION
# -------------------------------------------------------------------
EXPORT_PATH   = "C:/Users/johnh/Downloads/CalibrationOutput.dxf"
DO_EXPORT     = True

# --- Nebra AnyBeam 720p specs ---
RES_X         = 1280
RES_Y         = 720

# --- Scene objects ---
CUBE_NAME     = "Cube"

# --- Glass block dimensions (mm) ---
# Physical size of your glass block. The script resizes the Blender
# cube to match these at startup.  Set any to None to skip resizing
# and use the existing cube geometry as-is.
BLOCK_X_MM    = 50.0       # Width  (X axis)
BLOCK_Y_MM    = 50.0       # Height (Y axis)
BLOCK_Z_MM    = 80.0       # Depth  (Z axis)
BLOCK_UNIT_SCALE = 0.001   # mm → Blender scene units (0.001 = meters)

# --- Which patterns to generate ---
GENERATE_FOV_RULER  = True
GENERATE_ALIGNMENT  = True

# -------------------------------------------------------------------
# FOV RULER CONFIG
# -------------------------------------------------------------------
HFOV_MIN_DEG  = 37.0
HFOV_MAX_DEG  = 39.5
HFOV_STEP_DEG = 0.1

# Vertical FOV range (16:9: HFOV 37.6 → VFOV ~21.7)
VFOV_MIN_DEG  = 20.0
VFOV_MAX_DEG  = 23.0
VFOV_STEP_DEG = 0.1

# Calibration window — fraction of the projector frame to illuminate.
CALIB_WINDOW_FRACTION = 1.0        # 1.0 = full frame, 0.5 = center 50%
CALIB_IMAGE_PATH = "C:/Users/johnh/Downloads/CalibrationImage.png"

# Tick heights — the visual hierarchy
TICK_HEIGHT_WHOLE = 40    # .0 degrees (tallest landmark)
TICK_HEIGHT_HALF  = 28    # .5 degrees (mid landmark)
TICK_HEIGHT_TENTH = 14    # other tenths (short)

# Center reference crosshair half-arm length
CENTER_CROSS_LEN  = 25

# Ruler tick pixel step — trace every Nth pixel along each tick line.
# 1 = every pixel (dense), 3 = every 3rd pixel (sparse).
RULER_TICK_STEP   = 3

# Ruler tick depth range — ticks sweep from front to back so that
# projector positioning errors don't affect FOV calibration.
# If all ticks are coplanar, a lateral shift or tilt in the projector
# can mimic a different FOV. With depth variation, each tick is a 3D
# line: the "last glowing tick" is determined purely by cone angle.
RULER_DEPTH_FRONT = 0.20   # Depth at top/left end of tick (0=front face)
RULER_DEPTH_BACK  = 0.80   # Depth at bottom/right end of tick (1=back face)

# -------------------------------------------------------------------
# ALIGNMENT PATTERN CONFIG
# -------------------------------------------------------------------
# The measured projector HFOV. The alignment border maps to the
# projector's edge pixels at this FOV, NOT the Blender camera edges.
# VFOV is derived from the 16:9 aspect ratio automatically.
PROJECTOR_HFOV_DEG = 37.6
CORNER_RADIUS_PX   = 5
CROSSHAIR_LEN_PX   = 40
EDGE_TICK_LEN_PX   = 15
ALIGNMENT_INSET_PX = 2

# Interior alignment grid
GRID_ENABLED       = True
GRID_COLS          = 8
GRID_ROWS          = 5

# Margin (in pixels) around the FOV ruler where the alignment border
# leaves gaps so the ruler ticks remain visible.
RULER_GAP_MARGIN   = 5

# Border pixel step — trace every Nth projector pixel along each edge.
# 1280//5 = 256 pts per horizontal edge, 720//5 = 144 per vertical edge.
BORDER_PIXEL_STEP  = 5

# -------------------------------------------------------------------
# DEPTH VERIFICATION PROBES CONFIG
# -------------------------------------------------------------------
GENERATE_DEPTH_PROBES = True
DEPTH_FRONT           = 0.15   # Depth factor near front of safe zone (0=front)
DEPTH_BACK            = 0.85   # Depth factor near back of safe zone (1=back)

# -------------------------------------------------------------------
# VOLUMETRIC TEST REGIONS CONFIG
# -------------------------------------------------------------------
# Volumetric display test patches across the projector's horizontal
# FOV, positioned above the bottom VFOV ruler. Uses the projector's
# actual pixel step and 1 point per ray at random depth for a
# realistic volumetric display sanity check.
GENERATE_VOL_TEST      = True
VOL_TEST_SIZE_PX       = 15    # Half-size of each test region in pixels
VOL_TEST_PPR           = 1     # Points per ray (1 = single random depth)
VOL_TEST_PIXEL_STEP    = 5     # Trace every Nth pixel (projector renders 1 per 5)
VOL_TEST_N_REGIONS     = 5     # Number of test regions across H FOV
VOL_TEST_GAP_PX        = 10    # Gap above the bottom VFOV ruler
VOL_TEST_FOV_MIN       = 37.0  # HFOV (deg) for leftmost region
VOL_TEST_FOV_MAX       = 39.0  # HFOV (deg) for rightmost region

# -------------------------------------------------------------------
# POINT CLUSTERING TEST CONFIG — NUCLEUS/ELECTRON MODEL
# -------------------------------------------------------------------
# Each test "atom" has a nucleus point (single fracture at mid-depth on
# the projector ray) surrounded by electron points on a sphere shell.
# Tests whether more micro-fractures per voxel increase perceived
# brightness of the holographic point (Nayar & Anand, 2006).
GENERATE_CLUSTER_TEST   = True
CLUSTER_TEST_ATOM_SPACING = 8   # Pixels between nucleus centers within a patch
CLUSTER_TEST_PATCH_ATOMS  = 3   # NxN grid of atoms per config patch
CLUSTER_TEST_SPACING      = 18  # Pixels between patch centers
CLUSTER_CONFIGS = [
    # (label, n_electrons, orbit_depth_spread, orbit_pixel_radius)
    # orbit_depth_spread: fraction of ray segment for orbital depth radius
    # orbit_pixel_radius: lateral radius in pixels for electron positions
    ("nucleus",     0,  0.00, 0.0),  # Nucleus only (baseline)
    ("1+1_tight",   1,  0.02, 0.3),  # 1 electron, tight orbit
    ("1+2_tight",   2,  0.02, 0.3),  # 2 electrons, tight orbit
    ("1+3_tight",   3,  0.03, 0.5),  # 3 electrons, tight orbit
    ("1+5_tight",   5,  0.03, 0.5),  # 5 electrons, tight orbit
    ("1+5_wide",    5,  0.08, 1.5),  # 5 electrons, wider orbit
    ("1+8_tight",   8,  0.03, 0.5),  # 8 electrons, tight orbit
    ("1+8_wide",    8,  0.08, 1.5),  # 8 electrons, wider orbit
    ("1+12_tight", 12,  0.04, 0.8),  # 12 electrons, tight orbit
    ("1+12_wide",  12,  0.10, 2.0),  # 12 electrons, wide orbit
]

# -------------------------------------------------------------------
# FOV RULER EXTENSION
# -------------------------------------------------------------------
# How many pixels past the projector resolution bounds the ruler ticks
# extend in the perpendicular direction.  Makes ruler ticks visible
# outside the projected area so you can read angles beyond the cone.
RULER_EXTEND_PX = 30

# -------------------------------------------------------------------
# 2D-3D MAPPING EXPORT
# -------------------------------------------------------------------
MAPPING_EXPORT_PATH = "C:/Users/johnh/Downloads/projector_mapping.json"

# -------------------------------------------------------------------
# CALIBRATION VERIFICATION IMAGE
# -------------------------------------------------------------------
# Generates a projector image showing all active pixel positions and
# adds it to the Blender scene as a camera background for verification.
GENERATE_CALIB_VERIFY_IMAGE = True
CALIB_VERIFY_IMAGE_PATH = "C:/Users/johnh/Downloads/CalibrationVerify.png"

# -------------------------------------------------------------------
# GLASS / OPTICS
# -------------------------------------------------------------------
SAFE_ZONE_MARGIN = 0.90    # Inner 90% of block on each axis (avoids edge damage)
IOR_OUTSIDE      = 1.00
IOR_INSIDE       = 1.50
RAY_MAX_DIST     = 100000.0
POINT_RADIUS     = 0.00005

# -------------------------------------------------------------------
# OUTPUT OBJECT NAMES
# -------------------------------------------------------------------
FOV_RULER_NAME      = "FOV_Ruler"
ALIGNMENT_NAME      = "AlignmentPattern"
ALIGN_BORDER_NAME   = "AlignmentBorder"
DEPTH_PROBES_NAME   = "DepthProbes"
VOL_TEST_NAME       = "VolTestRegions"
CLUSTER_TEST_NAME   = "ClusterTests"

# -------------------------------------------------------------------
# DXF WRITER
# -------------------------------------------------------------------
def write_dxf_points(filepath, points):
    try:
        with open(filepath, 'w') as f:
            f.write("0\nSECTION\n2\nENTITIES\n")
            for p in points:
                f.write("0\nPOINT\n8\n0\n")
                f.write(f"10\n{p.x:.6f}\n")
                f.write(f"20\n{p.y:.6f}\n")
                f.write(f"30\n{p.z:.6f}\n")
            f.write("0\nENDSEC\n0\nEOF\n")
        print(f"SUCCESS: Exported {len(points)} points to {filepath}")
    except Exception as e:
        print(f"ERROR: Could not write DXF file: {e}")

# -------------------------------------------------------------------
# GEOMETRY & OPTICS
# -------------------------------------------------------------------
def intersect_aabb(ray_origin, ray_dir, box_min, box_max):
    tmin = -1e9
    tmax = 1e9
    for i in range(3):
        if abs(ray_dir[i]) < 1e-9:
            if ray_origin[i] < box_min[i] or ray_origin[i] > box_max[i]:
                return None
        else:
            inv_d = 1.0 / ray_dir[i]
            t1 = (box_min[i] - ray_origin[i]) * inv_d
            t2 = (box_max[i] - ray_origin[i]) * inv_d
            if t1 > t2: t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax: return None
    if tmax < 0: return None
    tmin = max(tmin, 0)
    return tmin, tmax

def refract(I, N, n1, n2):
    I = I.normalized()
    N = N.normalized()
    eta = n1 / n2
    cosi = -max(-1.0, min(1.0, I.dot(N)))
    k = 1.0 - eta * eta * (1.0 - cosi * cosi)
    if k < 0.0: return None
    cost = math.sqrt(k)
    return (eta * I + (eta * cosi - cost) * N).normalized()

# -------------------------------------------------------------------
# BLENDER SETUP
# -------------------------------------------------------------------
def create_emission_material(strength=5.0, color=(1.0, 1.0, 1.0, 1.0)):
    name = f"CalibGlow_{strength}_{color[0]:.2f}_{color[1]:.2f}_{color[2]:.2f}"
    mat = bpy.data.materials.get(name)
    if not mat:
        mat = bpy.data.materials.new(name)
        mat.use_nodes = True
        tree = mat.node_tree
        tree.nodes.clear()
        out = tree.nodes.new('ShaderNodeOutputMaterial')
        out.location = (400, 0)
        emission = tree.nodes.new('ShaderNodeEmission')
        emission.location = (0, 0)
        emission.inputs['Strength'].default_value = strength
        emission.inputs['Color'].default_value = color
        tree.links.new(emission.outputs['Emission'], out.inputs['Surface'])
    return mat

def setup_point_rendering(obj, radius=0.005, color=(1.0, 1.0, 1.0, 1.0)):
    mod = obj.modifiers.get("PointViz")
    if not mod: mod = obj.modifiers.new("PointViz", 'NODES')
    tree = mod.node_group
    if not tree:
        tree = bpy.data.node_groups.new(f"Viz_{obj.name}", 'GeometryNodeTree')
        mod.node_group = tree
    if hasattr(tree, 'interface'):
        tree.interface.clear()
        tree.interface.new_socket(name="Geometry", in_out='INPUT',
                                  socket_type='NodeSocketGeometry')
        tree.interface.new_socket(name="Geometry", in_out='OUTPUT',
                                  socket_type='NodeSocketGeometry')
    else:
        tree.inputs.clear()
        tree.outputs.clear()
        tree.inputs.new('NodeSocketGeometry', 'Geometry')
        tree.outputs.new('NodeSocketGeometry', 'Geometry')
    tree.nodes.clear()
    in_node = tree.nodes.new('NodeGroupInput')
    in_node.location = (-400, 0)
    mesh_to_points = tree.nodes.new('GeometryNodeMeshToPoints')
    mesh_to_points.location = (-200, 0)
    mesh_to_points.inputs['Radius'].default_value = radius
    set_mat = tree.nodes.new('GeometryNodeSetMaterial')
    set_mat.location = (0, 0)
    set_mat.inputs['Material'].default_value = create_emission_material(10.0, color)
    out_node = tree.nodes.new('NodeGroupOutput')
    out_node.location = (200, 0)
    tree.links.new(in_node.outputs[0], mesh_to_points.inputs['Mesh'])
    tree.links.new(mesh_to_points.outputs[0], set_mat.inputs['Geometry'])
    tree.links.new(set_mat.outputs[0], out_node.inputs[0])

def create_obj_from_points(name, points, color=(1.0, 1.0, 1.0, 1.0)):
    mesh = bpy.data.meshes.get(name)
    if not mesh: mesh = bpy.data.meshes.new(name)
    else: mesh.clear_geometry()
    mesh.from_pydata(points, [], [])
    obj = bpy.data.objects.get(name)
    if not obj:
        obj = bpy.data.objects.new(name, mesh)
        bpy.context.collection.objects.link(obj)
    else:
        obj.data = mesh
    setup_point_rendering(obj, radius=POINT_RADIUS * 1.5, color=color)
    return obj

def get_camera_vectors(scene, camera):
    frame = camera.data.view_frame(scene=scene)
    mat = camera.matrix_world
    world_frame = [mat @ v for v in frame]
    mat_inv = mat.inverted()
    local_frame = [mat_inv @ v for v in world_frame]
    sorted_y = sorted(zip(local_frame, world_frame),
                      key=lambda x: x[0].y, reverse=True)
    top_set = sorted_y[:2]
    bot_set = sorted_y[2:]
    top_set.sort(key=lambda x: x[0].x)
    bot_set.sort(key=lambda x: x[0].x)
    return (mat.translation,
            top_set[0][1], top_set[1][1],
            bot_set[0][1], bot_set[1][1])

# -------------------------------------------------------------------
# CAMERA FOV
# -------------------------------------------------------------------
def get_camera_hfov(scene, camera):
    render = scene.render
    sensor_fit = camera.data.sensor_fit
    focal_length = camera.data.lens
    aspect_x = render.resolution_x * render.pixel_aspect_x
    aspect_y = render.resolution_y * render.pixel_aspect_y
    if sensor_fit == 'HORIZONTAL' or (sensor_fit == 'AUTO' and aspect_x >= aspect_y):
        sensor_width = camera.data.sensor_width
    else:
        sensor_width = camera.data.sensor_height * (aspect_x / aspect_y)
    return math.degrees(2.0 * math.atan(sensor_width / (2.0 * focal_length)))

def get_camera_vfov(scene, camera):
    render = scene.render
    sensor_fit = camera.data.sensor_fit
    focal_length = camera.data.lens
    aspect_x = render.resolution_x * render.pixel_aspect_x
    aspect_y = render.resolution_y * render.pixel_aspect_y
    if sensor_fit == 'VERTICAL' or (sensor_fit == 'AUTO' and aspect_y > aspect_x):
        sensor_height = camera.data.sensor_height
    else:
        sensor_height = camera.data.sensor_width * (aspect_y / aspect_x)
    return math.degrees(2.0 * math.atan(sensor_height / (2.0 * focal_length)))

# -------------------------------------------------------------------
# CORE RAY PIPELINE
# -------------------------------------------------------------------
def build_ray_tracer(scene, camera, cube):
    """Returns trace(pix_x, pix_y, depth_factor) -> world point or None.

    Traces ray from camera through pixel (pix_x, pix_y) at 1280x720,
    refracts at glass surface, returns point at depth_factor (0=front,
    1=back) within the inner safe zone.  pix_x/pix_y may be floats
    for sub-pixel addressing.
    """
    for poly in cube.data.polygons:
        poly.use_smooth = False
    cube.data.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    bvh = BVHTree.FromObject(cube, depsgraph)

    cube_mat = cube.matrix_world
    cube_mat_inv = cube_mat.inverted()
    normal_mat = cube_mat_inv.transposed().to_3x3()

    # Compute safe zone from actual mesh bounding box (handles non-cubic blocks)
    local_bb = [Vector(corner) for corner in cube.bound_box]
    bb_min = Vector((min(v[i] for v in local_bb) for i in range(3)))
    bb_max = Vector((max(v[i] for v in local_bb) for i in range(3)))
    center = (bb_min + bb_max) * 0.5
    half = (bb_max - bb_min) * 0.5
    box_min = center - half * SAFE_ZONE_MARGIN
    box_max = center + half * SAFE_ZONE_MARGIN

    cam_origin, tl, tr, bl, br = get_camera_vectors(scene, camera)

    def trace(pix_x, pix_y, depth_factor=0.5):
        u = (pix_x + 0.5) / RES_X
        v = (pix_y + 0.5) / RES_Y

        vec_top = tl.lerp(tr, u)
        vec_bot = bl.lerp(br, u)
        target_pt = vec_bot.lerp(vec_top, v)

        ray_dir_world = (target_pt - cam_origin).normalized()
        ray_origin_local = cube_mat_inv @ cam_origin
        ray_dir_local = (cube_mat_inv.to_3x3() @ ray_dir_world).normalized()

        hit = bvh.ray_cast(ray_origin_local, ray_dir_local, RAY_MAX_DIST)
        if not hit[0]:
            return None

        loc_local, no_local = hit[0], hit[1]
        entry_point = cube_mat @ loc_local
        entry_normal = (normal_mat @ no_local).normalized()

        if entry_normal.dot(ray_dir_world) > -0.1:
            return None

        I_inside = refract(ray_dir_world, entry_normal, IOR_OUTSIDE, IOR_INSIDE)
        if not I_inside:
            return None

        ray_origin_local_2 = cube_mat_inv @ entry_point
        ray_dir_local_2 = (cube_mat_inv.to_3x3() @ I_inside).normalized()

        intervals = intersect_aabb(ray_origin_local_2, ray_dir_local_2,
                                   box_min, box_max)
        if not intervals:
            return None

        t_enter, t_exit = intervals
        if t_exit <= 0 or t_exit <= t_enter:
            return None

        t_enter = max(0.0, t_enter)
        t_val = t_enter + (t_exit - t_enter) * depth_factor
        p_local = ray_origin_local_2 + ray_dir_local_2 * t_val
        return cube_mat @ p_local

    return trace

# -------------------------------------------------------------------
# FOV-TO-PIXEL CONVERSION
# -------------------------------------------------------------------
def hfov_to_pixel_x(hfov_deg, camera_hfov_deg, window_px=None):
    """Pixel column in camera space where the measurement edge falls."""
    if window_px is None:
        window_px = RES_X
    half_angle = math.radians(hfov_deg / 2.0)
    edge_half = math.atan(math.tan(half_angle) * window_px / RES_X)
    cam_half = math.radians(camera_hfov_deg / 2.0)
    pixel_offset = (math.tan(edge_half) / math.tan(cam_half)) * (RES_X / 2.0)
    return RES_X / 2.0 + pixel_offset

def vfov_to_pixel_y(vfov_deg, camera_vfov_deg, window_py=None):
    """Same as hfov_to_pixel_x but vertical."""
    if window_py is None:
        window_py = RES_Y
    half_angle = math.radians(vfov_deg / 2.0)
    edge_half = math.atan(math.tan(half_angle) * window_py / RES_Y)
    cam_half = math.radians(camera_vfov_deg / 2.0)
    pixel_offset = (math.tan(edge_half) / math.tan(cam_half)) * (RES_Y / 2.0)
    return RES_Y / 2.0 + pixel_offset

# -------------------------------------------------------------------
# PROJECTOR FOV HELPERS
# -------------------------------------------------------------------
def projector_vfov_from_hfov(hfov_deg):
    """Compute projector VFOV from HFOV assuming native resolution aspect ratio."""
    return 2.0 * math.degrees(math.atan(
        math.tan(math.radians(hfov_deg / 2.0)) * RES_Y / RES_X))

def get_projector_pixel_bounds(cam_hfov, cam_vfov):
    """Map projector FOV edge pixels into camera pixel coordinates.

    Returns (px_left, px_right, py_top, py_bottom) in camera pixel space.
    When the Blender camera FOV equals PROJECTOR_HFOV_DEG, these return
    (0, RES_X, 0, RES_Y) — i.e. the full frame.
    """
    proj_vfov = projector_vfov_from_hfov(PROJECTOR_HFOV_DEG)
    px_right = hfov_to_pixel_x(PROJECTOR_HFOV_DEG, cam_hfov)
    px_left = RES_X - px_right
    py_bottom = vfov_to_pixel_y(proj_vfov, cam_vfov)
    py_top = RES_Y - py_bottom
    return px_left, px_right, py_top, py_bottom

def proj_to_cam(proj_px, proj_py, px_left, px_right, py_top, py_bottom):
    """Convert projector pixel coordinates to camera pixel coordinates.

    Projector pixel (0, 0) maps to camera pixel (px_left, py_top).
    Projector pixel (RES_X-1, RES_Y-1) maps to (px_right, py_bottom).
    Accepts float inputs for sub-pixel addressing.
    """
    cam_x = px_left + (px_right - px_left) * proj_px / (RES_X - 1)
    cam_y = py_top + (py_bottom - py_top) * proj_py / (RES_Y - 1)
    return cam_x, cam_y

# -------------------------------------------------------------------
# PART 1: FOV MEASUREMENT RULER
# -------------------------------------------------------------------
def get_glass_angular_extent(camera, cube):
    """Compute the full H and V FOV the glass block subtends from the camera."""
    cam_inv = camera.matrix_world.inverted()
    bb_cam = [cam_inv @ (cube.matrix_world @ Vector(c)) for c in cube.bound_box]

    max_htan = 0.0
    max_vtan = 0.0
    min_depth = float('inf')
    for v in bb_cam:
        depth = -v.z
        if depth > 0.001:
            max_htan = max(max_htan, abs(v.x) / depth)
            max_vtan = max(max_vtan, abs(v.y) / depth)
            min_depth = min(min_depth, depth)

    hfov = math.degrees(2.0 * math.atan(max_htan)) if max_htan > 0 else 0.0
    vfov = math.degrees(2.0 * math.atan(max_vtan)) if max_vtan > 0 else 0.0
    return hfov, vfov, min_depth

def generate_calibration_image(filepath, window_x, window_y):
    """Generate a calibration PNG: centered white rectangle on black."""
    name = "FOV_CalibImage"
    img = bpy.data.images.get(name)
    if img:
        bpy.data.images.remove(img)
    img = bpy.data.images.new(name, RES_X, RES_Y, alpha=False)

    pixels = [0.0, 0.0, 0.0, 1.0] * (RES_X * RES_Y)

    x_start = (RES_X - window_x) // 2
    x_end = x_start + window_x
    y_start = (RES_Y - window_y) // 2
    y_end = y_start + window_y

    for y in range(y_start, y_end):
        for x in range(x_start, x_end):
            idx = (y * RES_X + x) * 4
            pixels[idx]     = 1.0
            pixels[idx + 1] = 1.0
            pixels[idx + 2] = 1.0

    img.pixels = pixels
    img.filepath_raw = filepath
    img.file_format = 'PNG'
    img.save()
    bpy.data.images.remove(img)
    print(f"Calibration image saved: {filepath}")
    print(f"  White window: {window_x}x{window_y} centered in {RES_X}x{RES_Y}")

def get_tick_height(fov_rounded):
    """Tick height based on sub-degree value."""
    tenths = round((fov_rounded - int(fov_rounded)) * 10) % 10
    if tenths == 0:
        return TICK_HEIGHT_WHOLE
    elif tenths == 5:
        return TICK_HEIGHT_HALF
    else:
        return TICK_HEIGHT_TENTH

def generate_fov_ruler(trace):
    """Generate height-encoded tick marks for FOV measurement.

    Each tick sweeps depth from RULER_DEPTH_FRONT to RULER_DEPTH_BACK
    along its length, making it a 3D line rather than a flat mark.
    This ensures projector positioning errors (distance, tilt) don't
    corrupt the FOV reading — the last glowing tick is determined
    purely by the projector's cone angle, not its position.
    """
    scene = bpy.context.scene
    cam = scene.camera
    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)

    wf = CALIB_WINDOW_FRACTION
    win_px = int(round(RES_X * wf))
    win_py = int(round(RES_Y * wf))
    windowed = wf < 1.0

    if windowed:
        print(f"Window mode: {win_px}x{win_py} px "
              f"({wf*100:.0f}% of {RES_X}x{RES_Y})")
    else:
        print("Full-frame mode")

    # Validate camera is wide enough for the tick positions
    max_h_pixel = hfov_to_pixel_x(HFOV_MAX_DEG, cam_hfov, win_px)
    max_v_pixel = vfov_to_pixel_y(VFOV_MAX_DEG, cam_vfov, win_py)
    if max_h_pixel >= RES_X:
        print(f"WARNING: Camera HFOV ({cam_hfov:.1f} deg) too narrow "
              f"for H ruler max pixel {max_h_pixel:.0f}")
    if max_v_pixel >= RES_Y:
        print(f"WARNING: Camera VFOV ({cam_vfov:.1f} deg) too narrow "
              f"for V ruler max pixel {max_v_pixel:.0f}")

    # Validate glass subtends enough angle
    cube = bpy.data.objects.get(CUBE_NAME)
    glass_hfov, glass_vfov, glass_dist = get_glass_angular_extent(cam, cube)
    print(f"Camera: HFOV={cam_hfov:.2f} deg, VFOV={cam_vfov:.2f} deg")
    print(f"Glass subtends {glass_hfov:.1f} x {glass_vfov:.1f} deg "
          f"({glass_dist/BLOCK_UNIT_SCALE:.1f}mm away)")

    h_edge_max = math.degrees(2.0 * math.atan(
        math.tan(math.radians(HFOV_MAX_DEG / 2.0)) * win_px / RES_X))
    v_edge_max = math.degrees(2.0 * math.atan(
        math.tan(math.radians(VFOV_MAX_DEG / 2.0)) * win_py / RES_Y))
    print(f"Ticks span: H={h_edge_max:.1f} deg, V={v_edge_max:.1f} deg "
          f"(window-adjusted)")
    print(f"Ruler: HFOV {HFOV_MIN_DEG}-{HFOV_MAX_DEG} deg | "
          f"VFOV {VFOV_MIN_DEG}-{VFOV_MAX_DEG} deg")

    if h_edge_max > glass_hfov:
        print(f"\n  H ticks need {h_edge_max:.1f} deg but glass covers "
              f"{glass_hfov:.1f} deg.")
        if windowed:
            safe_frac = glass_hfov / HFOV_MAX_DEG
            print(f"  Try CALIB_WINDOW_FRACTION = {safe_frac:.2f} or smaller")
        else:
            dims = cube.dimensions
            half_w = dims.x / 2.0
            need_dist = half_w / math.tan(math.radians(HFOV_MAX_DEG / 2.0))
            print(f"  Move camera within {need_dist/BLOCK_UNIT_SCALE:.0f}mm, "
                  f"or use CALIB_WINDOW_FRACTION < 1.0")

    if v_edge_max > glass_vfov:
        print(f"\n  V ticks need {v_edge_max:.1f} deg but glass covers "
              f"{glass_vfov:.1f} deg.")
        if windowed:
            safe_frac = glass_vfov / VFOV_MAX_DEG
            print(f"  Try CALIB_WINDOW_FRACTION = {safe_frac:.2f} or smaller")
        else:
            dims = cube.dimensions
            half_h = dims.y / 2.0
            need_dist = half_h / math.tan(math.radians(VFOV_MAX_DEG / 2.0))
            print(f"  Move camera within {need_dist/BLOCK_UNIT_SCALE:.0f}mm, "
                  f"or use CALIB_WINDOW_FRACTION < 1.0")

    points = []
    y_center = RES_Y // 2
    x_center = RES_X // 2

    # === HORIZONTAL FOV RULER (vertical ticks at Y midline) ===
    # Height-encoded ticks centered at midline. The ruler naturally
    # extends past the projector resolution because HFOV_MAX_DEG >
    # PROJECTOR_HFOV_DEG — ticks beyond the projector's cone angle
    # exist but won't glow, which is how you read the FOV.
    fov = HFOV_MIN_DEG
    while fov <= HFOV_MAX_DEG + 0.001:
        fov_rounded = round(fov, 1)
        tick_h = get_tick_height(fov_rounded)
        half_h = tick_h // 2

        px_right = hfov_to_pixel_x(fov_rounded, cam_hfov, win_px)
        px_left = RES_X - px_right

        y_start = max(0, y_center - half_h)
        y_end = min(RES_Y - 1, y_center + half_h)

        ix_r = int(round(px_right))
        if 0 <= ix_r < RES_X:
            for y in range(y_start, y_end + 1, RULER_TICK_STEP):
                t = (y - y_start) / max(1, y_end - y_start)
                d = RULER_DEPTH_FRONT + (RULER_DEPTH_BACK - RULER_DEPTH_FRONT) * t
                pt = trace(ix_r, y, d)
                if pt:
                    points.append(pt)

        ix_l = int(round(px_left))
        if 0 <= ix_l < RES_X:
            for y in range(y_start, y_end + 1, RULER_TICK_STEP):
                t = (y - y_start) / max(1, y_end - y_start)
                d = RULER_DEPTH_FRONT + (RULER_DEPTH_BACK - RULER_DEPTH_FRONT) * t
                pt = trace(ix_l, y, d)
                if pt:
                    points.append(pt)

        fov = round(fov + HFOV_STEP_DEG, 1)

    # === VERTICAL FOV RULER (horizontal ticks at X midline) ===
    fov = VFOV_MIN_DEG
    while fov <= VFOV_MAX_DEG + 0.001:
        fov_rounded = round(fov, 1)
        tick_w = get_tick_height(fov_rounded)
        half_w = tick_w // 2

        py_bottom = vfov_to_pixel_y(fov_rounded, cam_vfov, win_py)
        py_top = RES_Y - py_bottom

        x_start = max(0, x_center - half_w)
        x_end = min(RES_X - 1, x_center + half_w)

        iy_b = int(round(py_bottom))
        if 0 <= iy_b < RES_Y:
            for x in range(x_start, x_end + 1, RULER_TICK_STEP):
                t = (x - x_start) / max(1, x_end - x_start)
                d = RULER_DEPTH_FRONT + (RULER_DEPTH_BACK - RULER_DEPTH_FRONT) * t
                pt = trace(x, iy_b, d)
                if pt:
                    points.append(pt)

        iy_t = int(round(py_top))
        if 0 <= iy_t < RES_Y:
            for x in range(x_start, x_end + 1, RULER_TICK_STEP):
                t = (x - x_start) / max(1, x_end - x_start)
                d = RULER_DEPTH_FRONT + (RULER_DEPTH_BACK - RULER_DEPTH_FRONT) * t
                pt = trace(x, iy_t, d)
                if pt:
                    points.append(pt)

        fov = round(fov + VFOV_STEP_DEG, 1)

    # === CENTER REFERENCE CROSSHAIR (with depth sweep) ===
    ch_y_start = y_center - CENTER_CROSS_LEN
    ch_y_end = y_center + CENTER_CROSS_LEN
    for y in range(ch_y_start, ch_y_end + 1, RULER_TICK_STEP):
        if 0 <= y < RES_Y:
            t = (y - ch_y_start) / max(1, ch_y_end - ch_y_start)
            d = RULER_DEPTH_FRONT + (RULER_DEPTH_BACK - RULER_DEPTH_FRONT) * t
            pt = trace(x_center, y, d)
            if pt:
                points.append(pt)
    ch_x_start = x_center - CENTER_CROSS_LEN
    ch_x_end = x_center + CENTER_CROSS_LEN
    for x in range(ch_x_start, ch_x_end + 1, RULER_TICK_STEP):
        if 0 <= x < RES_X:
            t = (x - ch_x_start) / max(1, ch_x_end - ch_x_start)
            d = RULER_DEPTH_FRONT + (RULER_DEPTH_BACK - RULER_DEPTH_FRONT) * t
            pt = trace(x, y_center, d)
            if pt:
                points.append(pt)

    print(f"FOV ruler: {len(points)} points")
    return points

# -------------------------------------------------------------------
# PART 2: ALIGNMENT CALIBRATION
# -------------------------------------------------------------------
def generate_alignment_pattern(trace):
    """Generate alignment calibration pattern mapped to projector FOV.

    Border and all features map to the projector's actual edge pixels at
    PROJECTOR_HFOV_DEG, not the Blender camera edges. The border leaves
    gaps where the FOV ruler ticks cross so the ruler stays visible.
    """
    scene = bpy.context.scene
    cam = scene.camera
    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)
    proj_vfov = projector_vfov_from_hfov(PROJECTOR_HFOV_DEG)

    px_left, px_right, py_top, py_bottom = get_projector_pixel_bounds(
        cam_hfov, cam_vfov)

    print(f"Projector FOV: {PROJECTOR_HFOV_DEG:.1f} deg H x "
          f"{proj_vfov:.1f} deg V")
    print(f"Projector edges in camera pixels: "
          f"X=[{px_left:.1f}, {px_right:.1f}] "
          f"Y=[{py_top:.1f}, {py_bottom:.1f}]")

    if cam_hfov < PROJECTOR_HFOV_DEG:
        print(f"WARNING: Camera HFOV ({cam_hfov:.1f}) narrower than "
              f"projector ({PROJECTOR_HFOV_DEG:.1f}). "
              f"Alignment border will be clipped.")

    points = []
    border_points = []
    inset = ALIGNMENT_INSET_PX
    d = 0.5  # Single depth for all on-plane features

    # All features use projector pixel coordinates via proj_to_cam().
    # This ensures every fracture point maps to an integer projector pixel.
    p2c = lambda ppx, ppy: proj_to_cam(ppx, ppy, px_left, px_right,
                                        py_top, py_bottom)

    # Projector pixel bounds (with inset)
    proj_x_min = inset
    proj_x_max = RES_X - 1 - inset
    proj_y_min = inset
    proj_y_max = RES_Y - 1 - inset
    proj_cx = RES_X // 2
    proj_cy = RES_Y // 2

    # === 1. CORNER POINTS (projector pixel space) ===
    corners = [
        (proj_x_min, proj_y_min),
        (proj_x_max, proj_y_min),
        (proj_x_min, proj_y_max),
        (proj_x_max, proj_y_max),
    ]
    for corner_px, corner_py in corners:
        if 0 <= corner_px < RES_X and 0 <= corner_py < RES_Y:
            cam_x, cam_y = p2c(corner_px, corner_py)
            pt = trace(cam_x, cam_y, d)
            if pt:
                points.append(pt)

    # === 2. CENTER CROSSHAIR (projector pixel space) ===
    for ppy in range(proj_cy - CROSSHAIR_LEN_PX,
                     proj_cy + CROSSHAIR_LEN_PX + 1):
        if 0 <= ppy < RES_Y:
            cam_x, cam_y = p2c(proj_cx, ppy)
            pt = trace(cam_x, cam_y, d)
            if pt:
                points.append(pt)
    for ppx in range(proj_cx - CROSSHAIR_LEN_PX,
                     proj_cx + CROSSHAIR_LEN_PX + 1):
        if 0 <= ppx < RES_X:
            cam_x, cam_y = p2c(ppx, proj_cy)
            pt = trace(cam_x, cam_y, d)
            if pt:
                points.append(pt)

    # === 3. EDGE MIDPOINT MARKERS (projector pixel space) ===
    for ppy in range(proj_y_min, proj_y_min + EDGE_TICK_LEN_PX):
        cam_x, cam_y = p2c(proj_cx, ppy)
        pt = trace(cam_x, cam_y, d)
        if pt: points.append(pt)
    for ppy in range(proj_y_max - EDGE_TICK_LEN_PX + 1, proj_y_max + 1):
        cam_x, cam_y = p2c(proj_cx, ppy)
        pt = trace(cam_x, cam_y, d)
        if pt: points.append(pt)
    for ppx in range(proj_x_min, proj_x_min + EDGE_TICK_LEN_PX):
        cam_x, cam_y = p2c(ppx, proj_cy)
        pt = trace(cam_x, cam_y, d)
        if pt: points.append(pt)
    for ppx in range(proj_x_max - EDGE_TICK_LEN_PX + 1, proj_x_max + 1):
        cam_x, cam_y = p2c(ppx, proj_cy)
        pt = trace(cam_x, cam_y, d)
        if pt: points.append(pt)

    # === 4. BORDER RECTANGLE (projector pixel space, with ruler gaps) ===
    # Every BORDER_PIXEL_STEP-th projector pixel along each edge.
    gap = RULER_GAP_MARGIN

    # Ruler exclusion zones in projector pixel space
    ruler_py_top = proj_cy - TICK_HEIGHT_WHOLE // 2 - gap
    ruler_py_bot = proj_cy + TICK_HEIGHT_WHOLE // 2 + gap
    ruler_px_left = proj_cx - TICK_HEIGHT_WHOLE // 2 - gap
    ruler_px_right = proj_cx + TICK_HEIGHT_WHOLE // 2 + gap

    # Top edge
    for ppx in range(proj_x_min, proj_x_max + 1, BORDER_PIXEL_STEP):
        if ruler_px_left <= ppx <= ruler_px_right:
            continue
        cam_x, cam_y = p2c(ppx, proj_y_min)
        pt = trace(cam_x, cam_y, d)
        if pt: border_points.append(pt)

    # Bottom edge
    for ppx in range(proj_x_min, proj_x_max + 1, BORDER_PIXEL_STEP):
        if ruler_px_left <= ppx <= ruler_px_right:
            continue
        cam_x, cam_y = p2c(ppx, proj_y_max)
        pt = trace(cam_x, cam_y, d)
        if pt: border_points.append(pt)

    # Left edge
    for ppy in range(proj_y_min, proj_y_max + 1, BORDER_PIXEL_STEP):
        if ruler_py_top <= ppy <= ruler_py_bot:
            continue
        cam_x, cam_y = p2c(proj_x_min, ppy)
        pt = trace(cam_x, cam_y, d)
        if pt: border_points.append(pt)

    # Right edge
    for ppy in range(proj_y_min, proj_y_max + 1, BORDER_PIXEL_STEP):
        if ruler_py_top <= ppy <= ruler_py_bot:
            continue
        cam_x, cam_y = p2c(proj_x_max, ppy)
        pt = trace(cam_x, cam_y, d)
        if pt: border_points.append(pt)

    # === 5. SPARSE ALIGNMENT GRID (projector pixel space) ===
    if GRID_ENABLED:
        for col in range(1, GRID_COLS):
            for row in range(1, GRID_ROWS):
                proj_gx = int(RES_X * col / GRID_COLS)
                proj_gy = int(RES_Y * row / GRID_ROWS)
                if 0 <= proj_gx < RES_X and 0 <= proj_gy < RES_Y:
                    cam_x, cam_y = p2c(proj_gx, proj_gy)
                    pt = trace(cam_x, cam_y, d)
                    if pt:
                        points.append(pt)

    print(f"Alignment: {len(points)} feature pts + "
          f"{len(border_points)} border pts")
    return points, border_points

# -------------------------------------------------------------------
# DEPTH VERIFICATION PROBES
# -------------------------------------------------------------------
def generate_depth_probes(trace):
    """Generate off-plane verification points for FOV confirmation.

    All probes use projector pixel coordinates so every fracture point
    maps to a specific projector pixel.  Probes are placed between
    alignment grid dots at alternating front/back depths.
    """
    scene = bpy.context.scene
    cam = scene.camera
    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)

    px_left, px_right, py_top, py_bottom = get_projector_pixel_bounds(
        cam_hfov, cam_vfov)
    p2c = lambda ppx, ppy: proj_to_cam(ppx, ppy, px_left, px_right,
                                        py_top, py_bottom)
    inset = ALIGNMENT_INSET_PX

    points = []
    probe_idx = 0

    if not GRID_ENABLED:
        print("Depth probes require GRID_ENABLED=True — skipping")
        return points

    # Grid positions in projector pixel space (same as alignment)
    grid_pxs = [int(RES_X * col / GRID_COLS) for col in range(1, GRID_COLS)]
    grid_pys = [int(RES_Y * row / GRID_ROWS) for row in range(1, GRID_ROWS)]

    # --- Probes between horizontally adjacent grid dots ---
    for gpy in grid_pys:
        for col_idx in range(len(grid_pxs) - 1):
            mid_px = (grid_pxs[col_idx] + grid_pxs[col_idx + 1]) // 2
            depth = DEPTH_FRONT if (probe_idx % 2 == 0) else DEPTH_BACK
            probe_idx += 1
            if 0 <= mid_px < RES_X and 0 <= gpy < RES_Y:
                cam_x, cam_y = p2c(mid_px, gpy)
                pt = trace(cam_x, cam_y, depth)
                if pt: points.append(pt)

    # --- Probes between vertically adjacent grid dots ---
    for gpx in grid_pxs:
        for row_idx in range(len(grid_pys) - 1):
            mid_py = (grid_pys[row_idx] + grid_pys[row_idx + 1]) // 2
            depth = DEPTH_BACK if (probe_idx % 2 == 0) else DEPTH_FRONT
            probe_idx += 1
            if 0 <= gpx < RES_X and 0 <= mid_py < RES_Y:
                cam_x, cam_y = p2c(gpx, mid_py)
                pt = trace(cam_x, cam_y, depth)
                if pt: points.append(pt)

    # --- Corner depth probes (projector pixel space) ---
    corner_offset = CORNER_RADIUS_PX + 8
    corner_probes = [
        (inset + corner_offset, inset + corner_offset, DEPTH_FRONT),
        (RES_X - 1 - inset - corner_offset, inset + corner_offset, DEPTH_BACK),
        (inset + corner_offset, RES_Y - 1 - inset - corner_offset, DEPTH_BACK),
        (RES_X - 1 - inset - corner_offset,
         RES_Y - 1 - inset - corner_offset, DEPTH_FRONT),
    ]
    for cpx, cpy, depth in corner_probes:
        probe_idx += 1
        if 0 <= cpx < RES_X and 0 <= cpy < RES_Y:
            cam_x, cam_y = p2c(cpx, cpy)
            pt = trace(cam_x, cam_y, depth)
            if pt: points.append(pt)

    # --- Edge midpoint depth probes (projector pixel space) ---
    pcx = RES_X // 2
    pcy = RES_Y // 2
    edge_offset = EDGE_TICK_LEN_PX + 5
    edge_probes = [
        (pcx, inset + edge_offset, DEPTH_FRONT),
        (pcx, RES_Y - 1 - inset - edge_offset, DEPTH_BACK),
        (inset + edge_offset, pcy, DEPTH_FRONT),
        (RES_X - 1 - inset - edge_offset, pcy, DEPTH_BACK),
    ]
    for epx, epy, depth in edge_probes:
        probe_idx += 1
        if 0 <= epx < RES_X and 0 <= epy < RES_Y:
            cam_x, cam_y = p2c(epx, epy)
            pt = trace(cam_x, cam_y, depth)
            if pt: points.append(pt)

    print(f"Depth probes: {len(points)} pts across {probe_idx} probes "
          f"(d={DEPTH_FRONT} front, d={DEPTH_BACK} back)")
    return points

# -------------------------------------------------------------------
# VOLUMETRIC TEST REGIONS
# -------------------------------------------------------------------
def generate_vol_test_regions(trace):
    """Generate volumetric display test patches across the projector's H FOV.

    Each region uses a different HFOV (linearly interpolated from
    VOL_TEST_FOV_MIN to VOL_TEST_FOV_MAX) for its projector-to-camera
    pixel mapping.  This lets you visually compare which FOV assumption
    produces correct alignment — the region that looks best tells you
    your actual projector HFOV.

    Regions span from VOL_TEST_GAP_PX above the bottom VFOV ruler to
    VOL_TEST_GAP_PX above the top VFOV ruler.  Each region is
    VOL_TEST_SIZE_PX wide, traced at every VOL_TEST_PIXEL_STEP pixels
    with VOL_TEST_PPR points per ray at random depth.

    Returns (points, inter_points, mapping) where mapping is the full
    projector-pixel to world-point dictionary.
    """
    scene = bpy.context.scene
    cam = scene.camera
    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)

    # Use the main PROJECTOR_HFOV_DEG for layout (region positions, Y bounds)
    px_left, px_right, py_top, py_bottom = get_projector_pixel_bounds(
        cam_hfov, cam_vfov)
    p2c_layout = lambda ppx, ppy: proj_to_cam(ppx, ppy, px_left, px_right,
                                               py_top, py_bottom)

    # Compute vertical extent in camera pixel space, then convert
    # back to projector pixel bounds.
    wf = CALIB_WINDOW_FRACTION
    win_py = int(round(RES_Y * wf))

    # Bottom VFOV ruler innermost tick (closest to center, below center)
    bottom_ruler_cam_y = int(vfov_to_pixel_y(VFOV_MIN_DEG, cam_vfov, win_py))
    # Top VFOV ruler innermost tick (closest to center, above center)
    top_ruler_cam_y = int(RES_Y - vfov_to_pixel_y(VFOV_MIN_DEG, cam_vfov, win_py))

    # Convert camera Y to approximate projector Y
    def cam_y_to_proj_y(cam_y):
        if abs(py_bottom - py_top) < 0.001:
            return RES_Y // 2
        return int((cam_y - py_top) / (py_bottom - py_top) * (RES_Y - 1))

    # Region projector Y bounds: GAP above each ruler
    proj_y_bottom = cam_y_to_proj_y(bottom_ruler_cam_y) - VOL_TEST_GAP_PX
    proj_y_top = cam_y_to_proj_y(top_ruler_cam_y) - VOL_TEST_GAP_PX
    proj_y_top = max(0, proj_y_top)
    proj_y_bottom = min(RES_Y - 1, proj_y_bottom)

    sz = VOL_TEST_SIZE_PX
    step = VOL_TEST_PIXEL_STEP
    n_regions = VOL_TEST_N_REGIONS
    points = []
    inter_points = []
    mapping = {}

    # Build per-region info with linearly interpolated FOV
    region_infos = []
    for i in range(n_regions):
        x_frac = (i + 1.0) / (n_regions + 1.0)
        proj_cx = int(RES_X * x_frac)

        # Per-region FOV: linearly interpolate across regions
        fov_t = i / max(1, n_regions - 1)
        region_fov = VOL_TEST_FOV_MIN + (VOL_TEST_FOV_MAX - VOL_TEST_FOV_MIN) * fov_t

        # Build per-region pixel mapping using this region's FOV
        region_vfov = projector_vfov_from_hfov(region_fov)
        rpx_right = hfov_to_pixel_x(region_fov, cam_hfov)
        rpx_left = RES_X - rpx_right
        rpy_bottom = vfov_to_pixel_y(region_vfov, cam_vfov)
        rpy_top = RES_Y - rpy_bottom

        region_infos.append((proj_cx, region_fov,
                             rpx_left, rpx_right, rpy_top, rpy_bottom))

    height = proj_y_bottom - proj_y_top + 1
    print(f"Vol test: {n_regions} regions, {2*sz+1}px wide x {height}px tall, "
          f"step={step}, {VOL_TEST_PPR} PPR")
    print(f"  Projector Y range: {proj_y_top} to {proj_y_bottom}")
    print(f"  Per-region FOV: {VOL_TEST_FOV_MIN:.1f} to {VOL_TEST_FOV_MAX:.1f} deg")

    for i, (proj_cx, region_fov,
            rpx_left, rpx_right, rpy_top, rpy_bottom) in enumerate(region_infos):
        region_count = 0

        # Per-region proj_to_cam using this region's FOV bounds
        rp2c = lambda ppx, ppy, l=rpx_left, r=rpx_right, t=rpy_top, b=rpy_bottom: \
            proj_to_cam(ppx, ppy, l, r, t, b)

        for ppx in range(proj_cx - sz, proj_cx + sz + 1, step):
            for ppy in range(proj_y_top, proj_y_bottom + 1, step):
                if not (0 <= ppx < RES_X and 0 <= ppy < RES_Y):
                    continue
                for _ in range(VOL_TEST_PPR):
                    d = random.uniform(0.05, 0.95)
                    cam_x, cam_y = rp2c(ppx, ppy)
                    pt = trace(cam_x, cam_y, d)
                    if pt:
                        points.append(pt)
                        mapping[(ppx, ppy)] = (pt.x, pt.y, pt.z)
                        region_count += 1

        # --- FOV label: tick marks above region ---
        # Region number indicator: i+1 horizontal dots above the region
        label_y = proj_y_top - 3
        if label_y >= 0:
            for dot in range(i + 1):
                lx = proj_cx - (i) + dot * 2
                if 0 <= lx < RES_X:
                    cam_x, cam_y = p2c_layout(lx, label_y)
                    pt = trace(cam_x, cam_y, 0.5)
                    if pt: inter_points.append(pt)

        # Vertical connector line from region top to H ruler center
        # (dashed: every 3rd pixel)
        ruler_proj_y = RES_Y // 2
        conn_start = min(proj_y_top, ruler_proj_y)
        conn_end = max(proj_y_top, ruler_proj_y)
        for ppy in range(conn_start, conn_end + 1, 3):
            if 0 <= ppy < RES_Y and 0 <= proj_cx < RES_X:
                cam_x, cam_y = p2c_layout(proj_cx, ppy)
                pt = trace(cam_x, cam_y, 0.5)
                if pt: inter_points.append(pt)

        print(f"  Region {i+1}/{n_regions}: proj_x={proj_cx}, "
              f"FOV={region_fov:.1f} deg, {region_count} pts")

    # --- Alignment dots and depth probes between adjacent regions ---
    for i in range(n_regions - 1):
        cx_a = region_infos[i][0]
        cx_b = region_infos[i + 1][0]
        mid_px = (cx_a + cx_b) // 2
        mid_py = (proj_y_top + proj_y_bottom) // 2

        if 0 <= mid_px < RES_X and 0 <= mid_py < RES_Y:
            cam_x, cam_y = p2c_layout(mid_px, mid_py)
            pt = trace(cam_x, cam_y, 0.5)
            if pt: inter_points.append(pt)

        probe_offset = 15
        for probe_py, depth in [(mid_py - probe_offset, DEPTH_FRONT),
                                (mid_py + probe_offset, DEPTH_BACK)]:
            if 0 <= mid_px < RES_X and 0 <= probe_py < RES_Y:
                cam_x, cam_y = p2c_layout(mid_px, probe_py)
                pt = trace(cam_x, cam_y, depth)
                if pt: inter_points.append(pt)

    print(f"Vol test regions: {n_regions} patches, {len(points)} vol pts, "
          f"{len(inter_points)} inter-region pts, "
          f"{len(mapping)} mapping entries")
    return points, inter_points, mapping

# -------------------------------------------------------------------
# POINT CLUSTERING TESTS
# -------------------------------------------------------------------
def generate_cluster_tests(trace):
    """Generate nucleus/electron cluster test patches.

    Each test "atom" has a nucleus point (single fracture at mid-depth on
    the projector ray) surrounded by electron points on a sphere shell.
    This tests whether clustering more micro-fractures around a single
    voxel increases its perceived brightness when illuminated by the
    projector (per Nayar & Anand, Columbia CUCS-030-06, 2006).

    Each config gets an NxN grid of atoms (CLUSTER_TEST_PATCH_ATOMS^2)
    spaced CLUSTER_TEST_ATOM_SPACING pixels apart. Atoms within a patch
    are far enough apart that their electron shells don't overlap.

    Patches are arranged in a row in the lower portion of the projector
    FOV. Each patch has a separator tick above it and an electron-count
    indicator below.
    """
    scene = bpy.context.scene
    cam = scene.camera
    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)

    px_left, px_right, py_top, py_bottom = get_projector_pixel_bounds(
        cam_hfov, cam_vfov)
    inset = ALIGNMENT_INSET_PX
    x_min = max(0, int(round(px_left)) + inset)
    x_max = min(RES_X - 1, int(round(px_right)) - inset)
    y_min = max(0, int(round(py_top)) + inset)
    y_max = min(RES_Y - 1, int(round(py_bottom)) - inset)

    points = []
    n_patches = len(CLUSTER_CONFIGS)
    atom_n = CLUSTER_TEST_PATCH_ATOMS
    atom_sp = CLUSTER_TEST_ATOM_SPACING
    spacing = CLUSTER_TEST_SPACING

    # Patch footprint: NxN atoms spaced atom_sp pixels apart
    patch_width = (atom_n - 1) * atom_sp
    total_width = n_patches * patch_width + (n_patches - 1) * spacing

    # Center the row horizontally within projector bounds
    cx = (x_min + x_max) // 2
    start_x = cx - total_width // 2

    # Place in lower quarter of projector area
    cy_cluster = y_min + int((y_max - y_min) * 0.75)

    for cfg_idx, (label, n_electrons, orbit_depth, orbit_px) in enumerate(
            CLUSTER_CONFIGS):
        # Patch top-left corner in camera pixel space
        patch_origin_x = start_x + cfg_idx * (patch_width + spacing)
        patch_origin_y = cy_cluster - patch_width // 2

        patch_nuclei = 0
        patch_electrons = 0

        for ax in range(atom_n):
            for ay in range(atom_n):
                # Nucleus pixel position
                nuc_px = patch_origin_x + ax * atom_sp
                nuc_py = patch_origin_y + ay * atom_sp
                if not (0 <= nuc_px < RES_X and 0 <= nuc_py < RES_Y):
                    continue

                # --- Nucleus: single point at mid-depth ---
                pt = trace(nuc_px, nuc_py, 0.5)
                if pt:
                    points.append(pt)
                    patch_nuclei += 1

                # --- Electrons: distributed on sphere shell around nucleus ---
                for j in range(n_electrons):
                    # Uniform random point on sphere surface
                    theta = random.uniform(0, 2 * math.pi)
                    cos_phi = random.uniform(-1, 1)
                    sin_phi = math.sqrt(1 - cos_phi * cos_phi)
                    ex = sin_phi * math.cos(theta)
                    ey = sin_phi * math.sin(theta)
                    ez = cos_phi

                    # Scale to pixel and depth units
                    target_px = nuc_px + ex * orbit_px
                    target_py = nuc_py + ey * orbit_px
                    d = 0.5 + ez * orbit_depth / 2.0
                    d = max(0.01, min(0.99, d))
                    pt = trace(target_px, target_py, d)
                    if pt:
                        points.append(pt)
                        patch_electrons += 1

        # Separator tick above patch for visual identification
        patch_cx = patch_origin_x + patch_width // 2
        tick_top = patch_origin_y - patch_width // 2
        for tick_dy in range(-6, -2):
            py = tick_top + tick_dy
            if 0 <= py < RES_Y and 0 <= patch_cx < RES_X:
                pt = trace(patch_cx, py, 0.5)
                if pt:
                    points.append(pt)

        # Electron-count indicator below patch (n_electrons horizontal dots)
        indicator_count = max(1, min(n_electrons, 12))
        indicator_y = patch_origin_y + patch_width + 4
        for indicator in range(indicator_count):
            ix = patch_cx - indicator_count // 2 + indicator * 2
            if 0 <= ix < RES_X and 0 <= indicator_y < RES_Y:
                pt = trace(ix, indicator_y, 0.5)
                if pt:
                    points.append(pt)

        total_per_atom = 1 + n_electrons
        print(f"  Cluster '{label}': 1+{n_electrons} per atom, "
              f"orbit=({orbit_depth:.2f}d, {orbit_px:.1f}px), "
              f"{patch_nuclei} nuclei + {patch_electrons} electrons")

    print(f"Cluster tests: {len(points)} total pts "
          f"across {n_patches} configs ({atom_n}x{atom_n} atoms each)")
    return points

# -------------------------------------------------------------------
# MAIN
# -------------------------------------------------------------------
def generate_calibration():
    print("=" * 55)
    print("  NEBRA ANYBEAM FOV CALIBRATION & ALIGNMENT")
    print("=" * 55)

    scene = bpy.context.scene
    cube = bpy.data.objects.get(CUBE_NAME)
    cam = scene.camera
    if not cam or not cube:
        return print("ERROR: Need active Camera + object named "
                     f"'{CUBE_NAME}' in scene.")

    # Apply block dimensions to the Blender cube if specified
    if BLOCK_X_MM is not None and BLOCK_Y_MM is not None and BLOCK_Z_MM is not None:
        s = BLOCK_UNIT_SCALE
        cube.dimensions = Vector((BLOCK_X_MM * s, BLOCK_Y_MM * s, BLOCK_Z_MM * s))
        bpy.context.view_layer.update()
        print(f"Glass block: {BLOCK_X_MM} x {BLOCK_Y_MM} x {BLOCK_Z_MM} mm")
    else:
        dims = cube.dimensions
        print(f"Glass block: using existing cube "
              f"({dims.x/BLOCK_UNIT_SCALE:.1f} x "
              f"{dims.y/BLOCK_UNIT_SCALE:.1f} x "
              f"{dims.z/BLOCK_UNIT_SCALE:.1f} mm)")
    print(f"Safe zone: inner {SAFE_ZONE_MARGIN*100:.0f}% on each axis")

    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)
    proj_vfov = projector_vfov_from_hfov(PROJECTOR_HFOV_DEG)
    print(f"Camera: HFOV={cam_hfov:.2f} deg  VFOV={cam_vfov:.2f} deg")
    print(f"Projector: {RES_X}x{RES_Y} (Nebra AnyBeam)")
    print(f"Projector FOV: {PROJECTOR_HFOV_DEG:.1f} deg H x "
          f"{proj_vfov:.1f} deg V")

    trace = build_ray_tracer(scene, cam, cube)
    all_points = []

    if GENERATE_FOV_RULER:
        print("\n--- Part 1: FOV Ruler ---")
        ruler_pts = generate_fov_ruler(trace)
        create_obj_from_points(FOV_RULER_NAME, ruler_pts,
                               color=(0.0, 1.0, 0.0, 1.0))
        all_points.extend(ruler_pts)

        # Generate calibration image for window mode
        if CALIB_WINDOW_FRACTION < 1.0:
            win_px = int(round(RES_X * CALIB_WINDOW_FRACTION))
            win_py = int(round(RES_Y * CALIB_WINDOW_FRACTION))
            generate_calibration_image(CALIB_IMAGE_PATH, win_px, win_py)
            print(f"\n  PROJECT THIS IMAGE (not full white): {CALIB_IMAGE_PATH}")
            print(f"  Read the last glowing tick -> that IS your HFOV/VFOV.")
        else:
            print("\n  Project a full-white 1280x720 image.")
            print("  Read the last glowing tick -> that IS your HFOV/VFOV.")

    if GENERATE_ALIGNMENT:
        print("\n--- Part 2: Alignment Pattern ---")
        align_pts, border_pts = generate_alignment_pattern(trace)
        create_obj_from_points(ALIGNMENT_NAME, align_pts,
                               color=(1.0, 1.0, 0.0, 1.0))
        create_obj_from_points(ALIGN_BORDER_NAME, border_pts,
                               color=(0.0, 0.5, 1.0, 1.0))
        all_points.extend(align_pts)
        all_points.extend(border_pts)

    if GENERATE_DEPTH_PROBES:
        print("\n--- Depth Verification Probes ---")
        probe_pts = generate_depth_probes(trace)
        create_obj_from_points(DEPTH_PROBES_NAME, probe_pts,
                               color=(1.0, 0.0, 0.8, 1.0))  # Magenta
        all_points.extend(probe_pts)

    vol_mapping = {}
    if GENERATE_VOL_TEST:
        print("\n--- Volumetric Test Regions ---")
        vol_pts, inter_pts, vol_mapping = generate_vol_test_regions(trace)
        create_obj_from_points(VOL_TEST_NAME, vol_pts,
                               color=(0.0, 1.0, 1.0, 1.0))  # Cyan
        create_obj_from_points("VolTestInterRegion", inter_pts,
                               color=(1.0, 0.5, 0.0, 1.0))  # Orange
        all_points.extend(vol_pts)
        all_points.extend(inter_pts)

    if GENERATE_CLUSTER_TEST:
        print("\n--- Point Clustering Tests ---")
        print("  (Nayar & Anand, Columbia CUCS-030-06, 2006)")
        cluster_pts = generate_cluster_tests(trace)
        create_obj_from_points(CLUSTER_TEST_NAME, cluster_pts,
                               color=(1.0, 1.0, 1.0, 1.0))  # White
        all_points.extend(cluster_pts)

    # --- Export 2D->3D mapping as JSON ---
    if vol_mapping:
        import json
        mapping_out = {}
        for (ppx, ppy), (wx, wy, wz) in vol_mapping.items():
            mapping_out[f"{ppx},{ppy}"] = [wx, wy, wz]
        try:
            with open(MAPPING_EXPORT_PATH, 'w') as f:
                json.dump({
                    "res_x": RES_X,
                    "res_y": RES_Y,
                    "projector_hfov_deg": PROJECTOR_HFOV_DEG,
                    "ior_outside": IOR_OUTSIDE,
                    "ior_inside": IOR_INSIDE,
                    "safe_zone_margin": SAFE_ZONE_MARGIN,
                    "points": mapping_out,
                }, f, indent=2)
            print(f"\n2D->3D mapping exported: {MAPPING_EXPORT_PATH} "
                  f"({len(mapping_out)} entries)")
        except Exception as e:
            print(f"WARNING: Could not write mapping JSON: {e}")

    # --- Generate calibration verification image ---
    if GENERATE_CALIB_VERIFY_IMAGE:
        print("\n--- Calibration Verification Image ---")
        generate_calib_verify_image(trace, vol_mapping)

    if DO_EXPORT and all_points:
        write_dxf_points(EXPORT_PATH, all_points)

    print(f"\nTotal: {len(all_points)} calibration points")
    print("DONE")


def generate_calib_verify_image(trace, vol_mapping):
    """Generate a projector image and add it to the Blender scene.

    Creates a 1280x720 PNG showing all active projector pixel positions
    as white pixels on black.  Also loads it as a camera background
    image in Blender for visual alignment verification.
    """
    scene = bpy.context.scene
    cam = scene.camera
    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)

    px_left, px_right, py_top, py_bottom = get_projector_pixel_bounds(
        cam_hfov, cam_vfov)

    name = "CalibVerify"
    img = bpy.data.images.get(name)
    if img:
        bpy.data.images.remove(img)
    img = bpy.data.images.new(name, RES_X, RES_Y, alpha=False)

    pixels = [0.0, 0.0, 0.0, 1.0] * (RES_X * RES_Y)

    # Mark all mapped projector pixels as white
    active_count = 0
    for key_str in vol_mapping:
        if isinstance(key_str, tuple):
            ppx, ppy = key_str
        else:
            ppx, ppy = key_str
        if 0 <= ppx < RES_X and 0 <= ppy < RES_Y:
            # PNG pixel (0,0) is bottom-left; projector pixel (0,0) is top-left
            flipped_y = (RES_Y - 1) - ppy
            idx = (flipped_y * RES_X + ppx) * 4
            pixels[idx]     = 1.0
            pixels[idx + 1] = 1.0
            pixels[idx + 2] = 1.0
            active_count += 1

    # Also mark alignment features (border, grid, corners) as white
    # by tracing the projector pixel grid for the alignment border
    inset = ALIGNMENT_INSET_PX
    for ppx in range(inset, RES_X - inset, BORDER_PIXEL_STEP):
        for ppy in [inset, RES_Y - 1 - inset]:
            flipped_y = (RES_Y - 1) - ppy
            idx = (flipped_y * RES_X + ppx) * 4
            pixels[idx] = 0.0; pixels[idx+1] = 0.5; pixels[idx+2] = 1.0
            active_count += 1
    for ppy in range(inset, RES_Y - inset, BORDER_PIXEL_STEP):
        for ppx in [inset, RES_X - 1 - inset]:
            flipped_y = (RES_Y - 1) - ppy
            idx = (flipped_y * RES_X + ppx) * 4
            pixels[idx] = 0.0; pixels[idx+1] = 0.5; pixels[idx+2] = 1.0
            active_count += 1

    # Grid dots
    if GRID_ENABLED:
        for col in range(1, GRID_COLS):
            for row in range(1, GRID_ROWS):
                gx = int(RES_X * col / GRID_COLS)
                gy = int(RES_Y * row / GRID_ROWS)
                if 0 <= gx < RES_X and 0 <= gy < RES_Y:
                    flipped_y = (RES_Y - 1) - gy
                    idx = (flipped_y * RES_X + gx) * 4
                    pixels[idx] = 1.0
                    pixels[idx+1] = 1.0
                    pixels[idx+2] = 0.0
                    active_count += 1

    img.pixels = pixels
    try:
        img.filepath_raw = CALIB_VERIFY_IMAGE_PATH
        img.file_format = 'PNG'
        img.save()
        print(f"Calibration image saved: {CALIB_VERIFY_IMAGE_PATH}")
    except Exception as e:
        print(f"WARNING: Could not save calibration image: {e}")
        print("  (Image still available in Blender as 'CalibVerify')")

    # Add as camera background image for visual verification
    cam.data.show_background_images = True
    # Remove existing CalibVerify backgrounds
    for bg in list(cam.data.background_images):
        if bg.image and bg.image.name == name:
            cam.data.background_images.remove(bg)
    bg = cam.data.background_images.new()
    bg.image = img
    bg.alpha = 0.5
    bg.display_depth = 'FRONT'

    print(f"Calibration image added to camera background ({active_count} "
          f"active pixels)")


# --- Run from Blender scripting play button ---
generate_calibration()
