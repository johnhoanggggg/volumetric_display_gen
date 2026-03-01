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
#   Each region assumes a different projector HFOV (from VOL_TEST_HFOV_MIN
#   to VOL_TEST_HFOV_MAX). Regions are spread across the projector's
#   horizontal pixel grid and remap projector pixels to camera pixels
#   through their assumed HFOV. When illuminated by the real projector,
#   the region whose assumed HFOV matches the actual HFOV will display
#   correctly; others will show misaligned points.
#
# -------------------------------------------------------------------
# CLUSTER TESTS (Nayar & Anand, 2006)
#
#   Random seed points are scattered within each test patch, and each
#   seed gets a cloud of companion points in a sphere around it.
#   Different configs vary the number of companions and cloud radius
#   to evaluate light scattering brightness vs. spatial diffusion.
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
GRID_DOT_RADIUS_PX = 2

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
DEPTH_PROBE_RADIUS_PX = 2
DEPTH_FRONT           = 0.15   # Depth factor near front of safe zone (0=front)
DEPTH_BACK            = 0.85   # Depth factor near back of safe zone (1=back)

# -------------------------------------------------------------------
# VOLUMETRIC TEST REGIONS CONFIG
# -------------------------------------------------------------------
# Each region assumes a different projector HFOV (evenly spaced from
# VOL_TEST_HFOV_MIN to VOL_TEST_HFOV_MAX). Regions are spread across
# the projector's horizontal pixel grid. The region whose HFOV matches
# the real projector will display correctly when illuminated.
GENERATE_VOL_TEST      = True
VOL_TEST_SIZE_PX       = 15    # Half-size of each test region in pixels
VOL_TEST_PPR           = 1     # Points per ray (1 = single random depth)
VOL_TEST_PIXEL_STEP    = 5     # Trace every Nth pixel
VOL_TEST_N_REGIONS     = 5     # Number of test regions (one per HFOV value)
VOL_TEST_GAP_PX        = 10    # Gap above the bottom VFOV ruler
VOL_TEST_HFOV_MIN      = 37.0  # Minimum test HFOV (degrees)
VOL_TEST_HFOV_MAX      = 39.0  # Maximum test HFOV (degrees)

# -------------------------------------------------------------------
# POINT CLUSTERING TEST CONFIG
# -------------------------------------------------------------------
# Random seed points are scattered within each test patch. Each seed
# gets a cloud of companion points in a sphere around it (rejection
# sampling). Per Nayar & Anand (Columbia CUCS-030-06, 2006), multiple
# micro-fractures per voxel increase scattering. Configs vary companion
# count and cloud radius to compare brightness vs. diffusion.
GENERATE_CLUSTER_TEST    = True
CLUSTER_TEST_SIZE_PX     = 4     # Half-size of each cluster test patch
CLUSTER_TEST_SPACING     = 18    # Pixels between test patch centers
CLUSTER_TEST_N_SEEDS     = 20    # Random seed points per patch
CLUSTER_CONFIGS = [
    # (label, companions_per_seed, depth_radius, lateral_radius_px)
    # depth_radius: fraction of safe-zone ray segment
    # lateral_radius_px: pixel radius for companion spread
    ("bare",     0,  0.00, 0.0),  # Seeds only, no cloud (baseline)
    ("1c_tight", 1,  0.02, 0.3),  # 1 companion, very tight
    ("2c_tight", 2,  0.02, 0.5),  # 2 companions, tight cloud
    ("2c_wide",  2,  0.08, 2.0),  # 2 companions, wider cloud
    ("4c_tight", 4,  0.03, 0.8),  # 4 companions, tight
    ("4c_wide",  4,  0.08, 2.5),  # 4 companions, wider
    ("8c_tight", 8,  0.03, 1.0),  # 8 companions, tight cloud
    ("8c_wide",  8,  0.10, 3.0),  # 8 companions, wider cloud
    ("16c",      16, 0.08, 2.0),  # 16 companions, medium cloud
]

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

def remap_projector_to_camera(proj_px, proj_py, proj_hfov, cam_hfov, cam_vfov):
    """Map a projector pixel to a camera pixel given the assumed projector HFOV.

    The Blender camera has a wider FOV than the projector.  Given a
    projector pixel and the projector's assumed HFOV, compute the
    camera pixel that shares the same angular direction.  VFOV is
    derived from HFOV via the native 16:9 aspect ratio.
    """
    proj_vfov = 2.0 * math.degrees(math.atan(
        math.tan(math.radians(proj_hfov / 2.0)) * RES_Y / RES_X))

    # Projector pixel -> tangent-space direction
    u = (proj_px + 0.5) / RES_X
    v = (proj_py + 0.5) / RES_Y
    tan_h = math.tan(math.radians(proj_hfov / 2.0)) * (2.0 * u - 1.0)
    tan_v = math.tan(math.radians(proj_vfov / 2.0)) * (2.0 * v - 1.0)

    # Tangent-space direction -> camera pixel
    cam_u = 0.5 + 0.5 * tan_h / math.tan(math.radians(cam_hfov / 2.0))
    cam_v = 0.5 + 0.5 * tan_v / math.tan(math.radians(cam_vfov / 2.0))

    return cam_u * RES_X - 0.5, cam_v * RES_Y - 0.5

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
            for y in range(y_start, y_end + 1):
                # Depth sweeps front→back along tick height
                t = (y - y_start) / max(1, y_end - y_start)
                d = RULER_DEPTH_FRONT + (RULER_DEPTH_BACK - RULER_DEPTH_FRONT) * t
                pt = trace(ix_r, y, d)
                if pt:
                    points.append(pt)

        ix_l = int(round(px_left))
        if 0 <= ix_l < RES_X:
            for y in range(y_start, y_end + 1):
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
            for x in range(x_start, x_end + 1):
                # Depth sweeps front→back along tick width
                t = (x - x_start) / max(1, x_end - x_start)
                d = RULER_DEPTH_FRONT + (RULER_DEPTH_BACK - RULER_DEPTH_FRONT) * t
                pt = trace(x, iy_b, d)
                if pt:
                    points.append(pt)

        iy_t = int(round(py_top))
        if 0 <= iy_t < RES_Y:
            for x in range(x_start, x_end + 1):
                t = (x - x_start) / max(1, x_end - x_start)
                d = RULER_DEPTH_FRONT + (RULER_DEPTH_BACK - RULER_DEPTH_FRONT) * t
                pt = trace(x, iy_t, d)
                if pt:
                    points.append(pt)

        fov = round(fov + VFOV_STEP_DEG, 1)

    # === CENTER REFERENCE CROSSHAIR (with depth sweep) ===
    ch_y_start = y_center - CENTER_CROSS_LEN
    ch_y_end = y_center + CENTER_CROSS_LEN
    for y in range(ch_y_start, ch_y_end + 1):
        if 0 <= y < RES_Y:
            t = (y - ch_y_start) / max(1, ch_y_end - ch_y_start)
            d = RULER_DEPTH_FRONT + (RULER_DEPTH_BACK - RULER_DEPTH_FRONT) * t
            pt = trace(x_center, y, d)
            if pt:
                points.append(pt)
    ch_x_start = x_center - CENTER_CROSS_LEN
    ch_x_end = x_center + CENTER_CROSS_LEN
    for x in range(ch_x_start, ch_x_end + 1):
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

    # Projector FOV edge pixel bounds in camera space (with inset)
    x_min = max(0, int(round(px_left)) + inset)
    x_max = min(RES_X - 1, int(round(px_right)) - inset)
    y_min = max(0, int(round(py_top)) + inset)
    y_max = min(RES_Y - 1, int(round(py_bottom)) - inset)

    cx = (x_min + x_max) // 2
    cy = (y_min + y_max) // 2

    # === 1. CORNER FILLED CIRCLES ===
    corners = [
        (x_min, y_min),
        (x_max, y_min),
        (x_min, y_max),
        (x_max, y_max),
    ]
    for corner_x, corner_y in corners:
        r = CORNER_RADIUS_PX
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    px = corner_x + dx
                    py = corner_y + dy
                    if 0 <= px < RES_X and 0 <= py < RES_Y:
                        pt = trace(px, py, d)
                        if pt:
                            points.append(pt)

    # === 2. CENTER CROSSHAIR ===
    for y in range(cy - CROSSHAIR_LEN_PX, cy + CROSSHAIR_LEN_PX + 1):
        if 0 <= y < RES_Y:
            pt = trace(cx, y, d)
            if pt:
                points.append(pt)
    for x in range(cx - CROSSHAIR_LEN_PX, cx + CROSSHAIR_LEN_PX + 1):
        if 0 <= x < RES_X:
            pt = trace(x, cy, d)
            if pt:
                points.append(pt)

    # === 3. EDGE MIDPOINT MARKERS ===
    for y in range(y_min, y_min + EDGE_TICK_LEN_PX):
        pt = trace(cx, y, d)
        if pt:
            points.append(pt)
    for y in range(y_max - EDGE_TICK_LEN_PX + 1, y_max + 1):
        pt = trace(cx, y, d)
        if pt:
            points.append(pt)
    for x in range(x_min, x_min + EDGE_TICK_LEN_PX):
        pt = trace(x, cy, d)
        if pt:
            points.append(pt)
    for x in range(x_max - EDGE_TICK_LEN_PX + 1, x_max + 1):
        pt = trace(x, cy, d)
        if pt:
            points.append(pt)

    # === 4. BORDER RECTANGLE (per-pixel, with gaps for FOV ruler) ===
    # Trace along projector edge pixels (every BORDER_PIXEL_STEP-th pixel)
    # mapped to camera pixel space.  Each border point corresponds to an
    # actual projector pixel, not an interpolated 3D position.
    # 1280//5 = 256 pts per horizontal edge, 720//5 = 144 per vertical.
    y_center = RES_Y // 2
    x_center = RES_X // 2
    gap = RULER_GAP_MARGIN

    # H ruler exclusion zone for vertical edges (left/right)
    ruler_y_top = y_center - TICK_HEIGHT_WHOLE // 2 - gap
    ruler_y_bot = y_center + TICK_HEIGHT_WHOLE // 2 + gap

    # V ruler exclusion zone for horizontal edges (top/bottom)
    ruler_x_left = x_center - TICK_HEIGHT_WHOLE // 2 - gap
    ruler_x_right = x_center + TICK_HEIGHT_WHOLE // 2 + gap

    # --- Top edge: y = y_min, x varies across projector width ---
    for proj_x in range(0, RES_X, BORDER_PIXEL_STEP):
        cam_x = x_min + (x_max - x_min) * proj_x / (RES_X - 1)
        if ruler_x_left <= cam_x <= ruler_x_right:
            continue
        pt = trace(cam_x, y_min, d)
        if pt:
            border_points.append(pt)

    # --- Bottom edge: y = y_max, x varies ---
    for proj_x in range(0, RES_X, BORDER_PIXEL_STEP):
        cam_x = x_min + (x_max - x_min) * proj_x / (RES_X - 1)
        if ruler_x_left <= cam_x <= ruler_x_right:
            continue
        pt = trace(cam_x, y_max, d)
        if pt:
            border_points.append(pt)

    # --- Left edge: x = x_min, y varies across projector height ---
    for proj_y in range(0, RES_Y, BORDER_PIXEL_STEP):
        cam_y = y_min + (y_max - y_min) * proj_y / (RES_Y - 1)
        if ruler_y_top <= cam_y <= ruler_y_bot:
            continue
        pt = trace(x_min, cam_y, d)
        if pt:
            border_points.append(pt)

    # --- Right edge: x = x_max, y varies ---
    for proj_y in range(0, RES_Y, BORDER_PIXEL_STEP):
        cam_y = y_min + (y_max - y_min) * proj_y / (RES_Y - 1)
        if ruler_y_top <= cam_y <= ruler_y_bot:
            continue
        pt = trace(x_max, cam_y, d)
        if pt:
            border_points.append(pt)

    # === 5. SPARSE ALIGNMENT GRID ===
    if GRID_ENABLED:
        for col in range(1, GRID_COLS):
            for row in range(1, GRID_ROWS):
                gx = int(x_min + (x_max - x_min) * col / GRID_COLS)
                gy = int(y_min + (y_max - y_min) * row / GRID_ROWS)
                r = GRID_DOT_RADIUS_PX
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        if dx * dx + dy * dy <= r * r:
                            px = gx + dx
                            py = gy + dy
                            if 0 <= px < RES_X and 0 <= py < RES_Y:
                                pt = trace(px, py, d)
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

    Probes are placed at pixel locations between alignment grid dots,
    at alternating front/back depths in a checkerboard pattern. Points
    near the frame edges have the strongest parallax sensitivity.
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
    probe_idx = 0
    r = DEPTH_PROBE_RADIUS_PX

    if not GRID_ENABLED:
        print("Depth probes require GRID_ENABLED=True — skipping")
        return points

    # Compute grid positions (same as alignment pattern)
    grid_xs = [int(x_min + (x_max - x_min) * col / GRID_COLS)
               for col in range(1, GRID_COLS)]
    grid_ys = [int(y_min + (y_max - y_min) * row / GRID_ROWS)
               for row in range(1, GRID_ROWS)]

    # --- Probes between horizontally adjacent grid dots ---
    for gy in grid_ys:
        for col_idx in range(len(grid_xs) - 1):
            mid_x = (grid_xs[col_idx] + grid_xs[col_idx + 1]) // 2
            depth = DEPTH_FRONT if (probe_idx % 2 == 0) else DEPTH_BACK
            probe_idx += 1

            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if dx * dx + dy * dy <= r * r:
                        px = mid_x + dx
                        py = gy + dy
                        if 0 <= px < RES_X and 0 <= py < RES_Y:
                            pt = trace(px, py, depth)
                            if pt:
                                points.append(pt)

    # --- Probes between vertically adjacent grid dots ---
    for gx in grid_xs:
        for row_idx in range(len(grid_ys) - 1):
            mid_y = (grid_ys[row_idx] + grid_ys[row_idx + 1]) // 2
            depth = DEPTH_BACK if (probe_idx % 2 == 0) else DEPTH_FRONT
            probe_idx += 1

            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if dx * dx + dy * dy <= r * r:
                        px = gx + dx
                        py = mid_y + dy
                        if 0 <= px < RES_X and 0 <= py < RES_Y:
                            pt = trace(px, py, depth)
                            if pt:
                                points.append(pt)

    # --- Corner depth probes (strongest parallax at frame edges) ---
    corner_offset = CORNER_RADIUS_PX + 8
    corner_probes = [
        (x_min + corner_offset, y_min + corner_offset, DEPTH_FRONT),
        (x_max - corner_offset, y_min + corner_offset, DEPTH_BACK),
        (x_min + corner_offset, y_max - corner_offset, DEPTH_BACK),
        (x_max - corner_offset, y_max - corner_offset, DEPTH_FRONT),
    ]
    for cpx, cpy, depth in corner_probes:
        probe_idx += 1
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    px = cpx + dx
                    py = cpy + dy
                    if 0 <= px < RES_X and 0 <= py < RES_Y:
                        pt = trace(px, py, depth)
                        if pt:
                            points.append(pt)

    # --- Edge midpoint depth probes ---
    cx = (x_min + x_max) // 2
    cy = (y_min + y_max) // 2
    edge_offset = EDGE_TICK_LEN_PX + 5
    edge_probes = [
        (cx, y_min + edge_offset, DEPTH_FRONT),
        (cx, y_max - edge_offset, DEPTH_BACK),
        (x_min + edge_offset, cy, DEPTH_FRONT),
        (x_max - edge_offset, cy, DEPTH_BACK),
    ]
    for epx, epy, depth in edge_probes:
        probe_idx += 1
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    px = epx + dx
                    py = epy + dy
                    if 0 <= px < RES_X and 0 <= py < RES_Y:
                        pt = trace(px, py, depth)
                        if pt:
                            points.append(pt)

    print(f"Depth probes: {len(points)} pts across {probe_idx} probes "
          f"(d={DEPTH_FRONT} front, d={DEPTH_BACK} back)")
    return points

# -------------------------------------------------------------------
# VOLUMETRIC TEST REGIONS
# -------------------------------------------------------------------
def generate_vol_test_regions(trace):
    """Generate volumetric test patches, each assuming a different projector HFOV.

    Regions are spread across the projector's horizontal pixel grid.
    Each region remaps its projector pixels to camera pixels through its
    assumed HFOV (from VOL_TEST_HFOV_MIN to VOL_TEST_HFOV_MAX).  When
    illuminated by the real projector, the region whose assumed HFOV
    matches the actual HFOV will display correctly; others will show
    misaligned points.
    """
    scene = bpy.context.scene
    cam = scene.camera
    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)

    n_regions = VOL_TEST_N_REGIONS
    sz = VOL_TEST_SIZE_PX
    step = VOL_TEST_PIXEL_STEP
    points = []

    # Assign an HFOV to each region, evenly spaced across the test range
    if n_regions == 1:
        hfovs = [(VOL_TEST_HFOV_MIN + VOL_TEST_HFOV_MAX) / 2.0]
    else:
        hfovs = [VOL_TEST_HFOV_MIN +
                 i * (VOL_TEST_HFOV_MAX - VOL_TEST_HFOV_MIN) / (n_regions - 1)
                 for i in range(n_regions)]

    # Vertical center: position patches above the bottom VFOV ruler.
    # Compute target y in camera pixel space, then inverse-remap to
    # projector pixel space using the mid-range HFOV.
    wf = CALIB_WINDOW_FRACTION
    win_py = int(round(RES_Y * wf))
    bottom_ruler_y = int(vfov_to_pixel_y(VFOV_MIN_DEG, cam_vfov, win_py))
    cam_target_cy = bottom_ruler_y - VOL_TEST_GAP_PX - sz

    mid_hfov = (VOL_TEST_HFOV_MIN + VOL_TEST_HFOV_MAX) / 2.0
    mid_vfov = projector_vfov_from_hfov(mid_hfov)
    cam_v = (cam_target_cy + 0.5) / RES_Y
    tan_v = (cam_v - 0.5) * 2.0 * math.tan(math.radians(cam_vfov / 2.0))
    proj_v = 0.5 + 0.5 * tan_v / math.tan(math.radians(mid_vfov / 2.0))
    proj_cy = int(proj_v * RES_Y - 0.5)

    # Horizontal: spread across projector pixel grid
    margin = sz + 10

    print(f"Vol test: {n_regions} regions, {2*sz+1}x{2*sz+1} px, "
          f"step={step}, {VOL_TEST_PPR} PPR")
    print(f"  HFOV range: {VOL_TEST_HFOV_MIN:.1f} - "
          f"{VOL_TEST_HFOV_MAX:.1f} deg")

    for i in range(n_regions):
        hfov_test = hfovs[i]

        x_frac = (i + 1.0) / (n_regions + 1.0)
        proj_cx = int(margin + (RES_X - 2 * margin) * x_frac)

        region_count = 0
        for dx in range(-sz, sz + 1, step):
            for dy in range(-sz, sz + 1, step):
                proj_px = proj_cx + dx
                proj_py = proj_cy + dy
                if not (0 <= proj_px < RES_X and 0 <= proj_py < RES_Y):
                    continue

                cam_px, cam_py = remap_projector_to_camera(
                    proj_px, proj_py, hfov_test, cam_hfov, cam_vfov)
                if not (0 <= cam_px < RES_X and 0 <= cam_py < RES_Y):
                    continue

                for _ in range(VOL_TEST_PPR):
                    d = random.uniform(0.05, 0.95)
                    pt = trace(cam_px, cam_py, d)
                    if pt:
                        points.append(pt)
                        region_count += 1

        print(f"  Region {i+1}/{n_regions}: HFOV={hfov_test:.1f} deg, "
              f"proj_center=({proj_cx}, {proj_cy}), {region_count} pts")

    print(f"Vol test regions: {n_regions} patches, {len(points)} total pts")
    return points

# -------------------------------------------------------------------
# POINT CLUSTERING TESTS
# -------------------------------------------------------------------
def generate_cluster_tests(trace):
    """Generate test patches with clouds of points around random seeds.

    Per Nayar & Anand (Columbia CUCS-030-06, 2006), multiple micro-fractures
    per voxel increase light scattering. Random seed points are scattered
    within each patch, and each seed gets a cloud of companion points in a
    sphere via rejection sampling.

    Patches are arranged in a row in the lower portion of the projector FOV.
    Each patch has a separator tick above it and a dot indicator below.
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
    sz = CLUSTER_TEST_SIZE_PX
    spacing = CLUSTER_TEST_SPACING
    n_seeds = CLUSTER_TEST_N_SEEDS

    # Total width needed for all patches
    patch_width = 2 * sz + 1
    total_width = n_patches * patch_width + (n_patches - 1) * spacing

    # Center the row horizontally within projector bounds
    cx = (x_min + x_max) // 2
    start_x = cx - total_width // 2

    # Place in lower quarter of projector area
    cy_cluster = y_min + int((y_max - y_min) * 0.75)

    for cfg_idx, (label, n_companions, depth_radius, lateral_radius) in \
            enumerate(CLUSTER_CONFIGS):
        patch_cx = start_x + cfg_idx * (patch_width + spacing) + sz
        patch_cy = cy_cluster

        patch_count = 0
        for _ in range(n_seeds):
            # Random seed position within patch
            seed_px = patch_cx + random.uniform(-sz, sz)
            seed_py = patch_cy + random.uniform(-sz, sz)
            seed_depth = random.uniform(0.1, 0.9)

            if not (0 <= seed_px < RES_X and 0 <= seed_py < RES_Y):
                continue

            # Place the seed point itself
            pt = trace(seed_px, seed_py, seed_depth)
            if pt:
                points.append(pt)
                patch_count += 1

            # Place companion points in a sphere around the seed
            for _ in range(n_companions):
                while True:
                    rx = random.uniform(-1, 1)
                    ry = random.uniform(-1, 1)
                    rz = random.uniform(-1, 1)
                    if rx * rx + ry * ry + rz * rz <= 1.0:
                        break

                comp_px = seed_px + rx * lateral_radius
                comp_py = seed_py + ry * lateral_radius
                comp_depth = seed_depth + rz * depth_radius
                comp_depth = max(0.01, min(0.99, comp_depth))

                if 0 <= comp_px < RES_X and 0 <= comp_py < RES_Y:
                    pt = trace(comp_px, comp_py, comp_depth)
                    if pt:
                        points.append(pt)
                        patch_count += 1

        # Separator tick above patch for visual identification
        for tick_dy in range(-sz - 6, -sz - 2):
            py = patch_cy + tick_dy
            if 0 <= py < RES_Y and 0 <= patch_cx < RES_X:
                pt = trace(patch_cx, py, 0.5)
                if pt:
                    points.append(pt)

        # Indicator dots below patch (1 + companions per seed)
        n_dots = min(n_companions + 1, 12)
        for indicator in range(n_dots):
            ix = patch_cx - n_dots // 2 + indicator * 2
            iy = patch_cy + sz + 4
            if 0 <= ix < RES_X and 0 <= iy < RES_Y:
                pt = trace(ix, iy, 0.5)
                if pt:
                    points.append(pt)

        print(f"  Cluster '{label}': {n_seeds} seeds, "
              f"{n_companions} companions, "
              f"depth_r={depth_radius:.2f}, "
              f"lateral_r={lateral_radius:.1f}px "
              f"-> {patch_count} fractures")

    print(f"Cluster tests: {len(points)} total pts "
          f"across {n_patches} configs")
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

    if GENERATE_VOL_TEST:
        print("\n--- Volumetric Test Regions ---")
        vol_pts = generate_vol_test_regions(trace)
        create_obj_from_points(VOL_TEST_NAME, vol_pts,
                               color=(0.0, 1.0, 1.0, 1.0))  # Cyan
        all_points.extend(vol_pts)

    if GENERATE_CLUSTER_TEST:
        print("\n--- Point Clustering Tests ---")
        print("  (Nayar & Anand, Columbia CUCS-030-06, 2006)")
        cluster_pts = generate_cluster_tests(trace)
        create_obj_from_points(CLUSTER_TEST_NAME, cluster_pts,
                               color=(1.0, 1.0, 1.0, 1.0))  # White
        all_points.extend(cluster_pts)

    if DO_EXPORT and all_points:
        write_dxf_points(EXPORT_PATH, all_points)

    print(f"\nTotal: {len(all_points)} calibration points")
    print("DONE")

# --- Run from Blender scripting play button ---
generate_calibration()
