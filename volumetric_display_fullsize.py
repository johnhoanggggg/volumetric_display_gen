import bpy
import math
import random
from mathutils import Vector
from mathutils.bvhtree import BVHTree

# ===================================================================
# FULL-SIZE VOLUMETRIC DISPLAY GENERATOR
# ===================================================================
# Fills the inner volume of a glass block with fracture points using
# the projector's calibrated HFOV (37.6 deg).  Unlike the base
# volemetric_display_gen.py which maps pixels directly to the Blender
# camera frustum, this script maps projector pixels through the
# measured HFOV so the Blender camera can be set wider than the
# projector without affecting output.
#
# REQUIRED SCENE OBJECTS:
#   Camera  — Active scene camera (set FOV >= PROJECTOR_HFOV_DEG)
#   "Cube"  — Glass block (rays refract through this)
#
# WORKFLOW:
#   1. Calibrate projector FOV using projector_fov_calibration.py
#   2. Set PROJECTOR_HFOV_DEG below to measured value
#   3. Ensure Blender camera FOV >= projector FOV
#   4. Run script — generates point cloud + alignment helpers
#   5. DXF exported to EXPORT_PATH for laser etching
# ===================================================================

# -------------------------------------------------------------------
# CONFIGURATION
# -------------------------------------------------------------------
EXPORT_PATH   = "C:/Users/johnh/Downloads/LaserOutput.dxf"
DO_EXPORT     = True

# --- Projector specs (Nebra AnyBeam 720p) ---
RES_X         = 1280
RES_Y         = 720

# --- Calibrated projector FOV ---
PROJECTOR_HFOV_DEG = 38

# --- Scene objects ---
CUBE_NAME     = "Cube"
PCLOUD_NAME   = "PixelPerfectCloud"
HELPER_NAME   = "AlignmentHelpers"
BORDER_NAME   = "AlignmentBorder"
CIRCLES_NAME  = "AlignmentCircles"
DEPTH_NAME    = "DepthProbes"

# --- Glass block dimensions (mm) ---
# Set any to None to skip resizing and use existing cube geometry.
BLOCK_X_MM    = 50.0
BLOCK_Y_MM    = 50.0
BLOCK_Z_MM    = 80.0
BLOCK_UNIT_SCALE = 0.001   # mm -> Blender units (0.001 = meters)

# --- Safe zone ---
INNER_CUBE_SCALE = 0.90    # Inner 90% of block on each axis

# --- Density ---
PIXEL_STEP     = 5         # Trace every Nth projector pixel
POINTS_PER_RAY = 1         # Fracture points per ray along depth
POINT_RADIUS   = 0.00005

# --- Border ---
HELPER_MARGIN  = 2         # Pixels from border for alignment helpers

# --- Alignment pattern ---
CORNER_RADIUS_PX   = 5     # Filled circle radius at corners
CROSSHAIR_LEN_PX   = 40    # Center crosshair half-arm length
CROSSHAIR_WIDTH    = 3     # Crosshair line width in pixels
EDGE_TICK_LEN_PX   = 15    # Edge midpoint marker length
ALIGNMENT_INSET_PX = 2     # Inset from projector edge pixels
BORDER_PIXEL_STEP  = 5     # Trace every Nth pixel along border

# --- Depth probes ---
DEPTH_FRONT        = 0.15  # Front probe depth factor (0=front face)
DEPTH_BACK         = 0.85  # Back probe depth factor (1=back face)

# --- Projection transforms ---
ROTATE_90     = False
FLIP_X        = False
FLIP_Y        = False

# --- Optics ---
IOR_OUTSIDE   = 1.00
IOR_INSIDE    = 1.50
RAY_MAX_DIST  = 100000.0

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
    name = f"GlowMat_{strength}_{color[0]:.2f}_{color[1]:.2f}_{color[2]:.2f}"
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

# -------------------------------------------------------------------
# CAMERA / FOV HELPERS
# -------------------------------------------------------------------
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

def projector_vfov_from_hfov(hfov_deg):
    return 2.0 * math.degrees(math.atan(
        math.tan(math.radians(hfov_deg / 2.0)) * RES_Y / RES_X))

# -------------------------------------------------------------------
# PROJECTOR-TO-CAMERA PIXEL MAPPING
# -------------------------------------------------------------------
def hfov_to_pixel_x(hfov_deg, camera_hfov_deg):
    half_angle = math.radians(hfov_deg / 2.0)
    cam_half = math.radians(camera_hfov_deg / 2.0)
    pixel_offset = (math.tan(half_angle) / math.tan(cam_half)) * (RES_X / 2.0)
    return RES_X / 2.0 + pixel_offset

def vfov_to_pixel_y(vfov_deg, camera_vfov_deg):
    half_angle = math.radians(vfov_deg / 2.0)
    cam_half = math.radians(camera_vfov_deg / 2.0)
    pixel_offset = (math.tan(half_angle) / math.tan(cam_half)) * (RES_Y / 2.0)
    return RES_Y / 2.0 + pixel_offset

def get_projector_pixel_bounds(cam_hfov, cam_vfov):
    proj_vfov = projector_vfov_from_hfov(PROJECTOR_HFOV_DEG)
    px_right = hfov_to_pixel_x(PROJECTOR_HFOV_DEG, cam_hfov)
    px_left = RES_X - px_right
    py_bottom = vfov_to_pixel_y(proj_vfov, cam_vfov)
    py_top = RES_Y - py_bottom
    return px_left, px_right, py_top, py_bottom

def proj_to_cam(proj_px, proj_py, px_left, px_right, py_top, py_bottom):
    cam_x = px_left + (px_right - px_left) * proj_px / (RES_X - 1)
    cam_y = py_top + (py_bottom - py_top) * proj_py / (RES_Y - 1)
    return cam_x, cam_y

# -------------------------------------------------------------------
# RAY TRACER
# -------------------------------------------------------------------
def build_ray_tracer(scene, camera, cube):
    """Returns trace(cam_pix_x, cam_pix_y, depth_factor) -> world point.

    cam_pix_x/y are camera pixel coordinates (may be floats).
    depth_factor: 0=front of safe zone, 1=back.
    """
    for poly in cube.data.polygons:
        poly.use_smooth = False
    cube.data.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    bvh = BVHTree.FromObject(cube, depsgraph)

    cube_mat = cube.matrix_world
    cube_mat_inv = cube_mat.inverted()
    normal_mat = cube_mat_inv.transposed().to_3x3()

    # Compute safe zone from actual mesh bounding box
    local_bb = [Vector(corner) for corner in cube.bound_box]
    bb_min = Vector((min(v[i] for v in local_bb) for i in range(3)))
    bb_max = Vector((max(v[i] for v in local_bb) for i in range(3)))
    center = (bb_min + bb_max) * 0.5
    half = (bb_max - bb_min) * 0.5
    box_min = center - half * INNER_CUBE_SCALE
    box_max = center + half * INNER_CUBE_SCALE

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

    def trace_interval(pix_x, pix_y):
        """Returns (ray_origin_local, ray_dir_local, t_enter, t_exit) or None."""
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

        return (ray_origin_local_2, ray_dir_local_2, max(0.0, t_enter), t_exit)

    return trace, trace_interval, cube_mat

# -------------------------------------------------------------------
# MAIN
# -------------------------------------------------------------------
def generate_laser_cloud():
    print("=" * 55)
    print("  FULL-SIZE VOLUMETRIC DISPLAY GENERATOR")
    print("=" * 55)

    scene = bpy.context.scene
    cube = bpy.data.objects.get(CUBE_NAME)
    cam = scene.camera
    if not cam or not cube:
        return print(f"ERROR: Need active Camera + object named '{CUBE_NAME}'")

    # Apply block dimensions
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

    # Camera and projector FOV info
    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)
    proj_vfov = projector_vfov_from_hfov(PROJECTOR_HFOV_DEG)

    print(f"Camera:    HFOV={cam_hfov:.2f} deg  VFOV={cam_vfov:.2f} deg")
    print(f"Projector: {RES_X}x{RES_Y}, "
          f"HFOV={PROJECTOR_HFOV_DEG:.1f} deg, VFOV={proj_vfov:.1f} deg")
    print(f"Safe zone: inner {INNER_CUBE_SCALE*100:.0f}%")
    print(f"Density:   step={PIXEL_STEP}, PPR={POINTS_PER_RAY}")

    if cam_hfov < PROJECTOR_HFOV_DEG:
        print(f"WARNING: Camera HFOV ({cam_hfov:.1f}) narrower than "
              f"projector ({PROJECTOR_HFOV_DEG:.1f}). "
              f"Widen camera FOV for correct mapping.")

    # Projector pixel bounds in camera pixel space
    px_left, px_right, py_top, py_bottom = get_projector_pixel_bounds(
        cam_hfov, cam_vfov)
    print(f"Projector edges in camera pixels: "
          f"X=[{px_left:.1f}, {px_right:.1f}] "
          f"Y=[{py_top:.1f}, {py_bottom:.1f}]")

    p2c = lambda ppx, ppy: proj_to_cam(ppx, ppy, px_left, px_right,
                                        py_top, py_bottom)

    trace, trace_interval, cube_mat = build_ray_tracer(scene, cam, cube)

    # ------------------------------------------------------------------
    # 1. GENERATE MAIN CLOUD
    # ------------------------------------------------------------------
    print(f"\nGenerating cloud ({RES_X}x{RES_Y}, step={PIXEL_STEP})...")
    fracture_coords = []

    for proj_x in range(HELPER_MARGIN, RES_X - HELPER_MARGIN, PIXEL_STEP):
        for proj_y in range(HELPER_MARGIN, RES_Y - HELPER_MARGIN, PIXEL_STEP):
            # Apply projection transforms in projector pixel space
            px, py = proj_x, proj_y
            if ROTATE_90:
                px, py = py, RES_X - 1 - px
            if FLIP_X:
                px = RES_X - 1 - px
            if FLIP_Y:
                py = RES_Y - 1 - py

            cam_x, cam_y = p2c(px, py)
            res = trace_interval(cam_x, cam_y)
            if not res:
                continue

            r_orig, r_dir, t_in, t_out = res
            ray_len = t_out - t_in
            step_size = ray_len / POINTS_PER_RAY

            for i in range(POINTS_PER_RAY):
                base_t = t_in + step_size * i
                t_val = base_t + random.uniform(0.0, 1.0) * step_size
                p_local = r_orig + r_dir * t_val
                fracture_coords.append(cube_mat @ p_local)

    print(f"Cloud: {len(fracture_coords)} fracture points")
    create_obj_from_points(PCLOUD_NAME, fracture_coords,
                           color=(1.0, 1.0, 1.0, 1.0))

    # ------------------------------------------------------------------
    # 2. ALIGNMENT PATTERN — corners, crosshair, edge marks
    # ------------------------------------------------------------------
    print("\nGenerating alignment pattern...")
    align_pts = []
    inset = ALIGNMENT_INSET_PX
    d = 0.5  # Mid-depth for on-plane alignment features

    proj_x_min = inset
    proj_x_max = RES_X - 1 - inset
    proj_y_min = inset
    proj_y_max = RES_Y - 1 - inset
    proj_cx = RES_X // 2
    proj_cy = RES_Y // 2

    # --- Corner filled circles ---
    corners = [
        (proj_x_min, proj_y_min),
        (proj_x_max, proj_y_min),
        (proj_x_min, proj_y_max),
        (proj_x_max, proj_y_max),
    ]
    r = CORNER_RADIUS_PX
    for cx, cy in corners:
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    ppx, ppy = cx + dx, cy + dy
                    if 0 <= ppx < RES_X and 0 <= ppy < RES_Y:
                        cam_x, cam_y = p2c(ppx, ppy)
                        pt = trace(cam_x, cam_y, d)
                        if pt: align_pts.append(pt)

    # --- Center crosshair ---
    half_cw = CROSSHAIR_WIDTH // 2
    for ppy in range(proj_cy - CROSSHAIR_LEN_PX,
                     proj_cy + CROSSHAIR_LEN_PX + 1):
        for wx in range(-half_cw, half_cw + 1):
            ppx = proj_cx + wx
            if 0 <= ppx < RES_X and 0 <= ppy < RES_Y:
                cam_x, cam_y = p2c(ppx, ppy)
                pt = trace(cam_x, cam_y, d)
                if pt: align_pts.append(pt)
    for ppx in range(proj_cx - CROSSHAIR_LEN_PX,
                     proj_cx + CROSSHAIR_LEN_PX + 1):
        for wy in range(-half_cw, half_cw + 1):
            ppy = proj_cy + wy
            if 0 <= ppx < RES_X and 0 <= ppy < RES_Y:
                cam_x, cam_y = p2c(ppx, ppy)
                pt = trace(cam_x, cam_y, d)
                if pt: align_pts.append(pt)

    # --- Edge midpoint markers (verify no roll) ---
    for ppy in range(proj_y_min, proj_y_min + EDGE_TICK_LEN_PX):
        if 0 <= ppy < RES_Y:
            cam_x, cam_y = p2c(proj_cx, ppy)
            pt = trace(cam_x, cam_y, d)
            if pt: align_pts.append(pt)
    for ppy in range(proj_y_max - EDGE_TICK_LEN_PX + 1, proj_y_max + 1):
        if 0 <= ppy < RES_Y:
            cam_x, cam_y = p2c(proj_cx, ppy)
            pt = trace(cam_x, cam_y, d)
            if pt: align_pts.append(pt)
    for ppx in range(proj_x_min, proj_x_min + EDGE_TICK_LEN_PX):
        if 0 <= ppx < RES_X:
            cam_x, cam_y = p2c(ppx, proj_cy)
            pt = trace(cam_x, cam_y, d)
            if pt: align_pts.append(pt)
    for ppx in range(proj_x_max - EDGE_TICK_LEN_PX + 1, proj_x_max + 1):
        if 0 <= ppx < RES_X:
            cam_x, cam_y = p2c(ppx, proj_cy)
            pt = trace(cam_x, cam_y, d)
            if pt: align_pts.append(pt)

    print(f"Alignment: {len(align_pts)} feature points")
    create_obj_from_points(HELPER_NAME, align_pts,
                           color=(1.0, 1.0, 0.0, 1.0))

    # ------------------------------------------------------------------
    # 3. BORDER FRAME — per-pixel outline at projector edges
    # ------------------------------------------------------------------
    print("Generating border frame...")
    border_pts = []

    # Top edge
    for ppx in range(proj_x_min, proj_x_max + 1, BORDER_PIXEL_STEP):
        cam_x, cam_y = p2c(ppx, proj_y_min)
        pt = trace(cam_x, cam_y, d)
        if pt: border_pts.append(pt)

    # Bottom edge
    for ppx in range(proj_x_min, proj_x_max + 1, BORDER_PIXEL_STEP):
        cam_x, cam_y = p2c(ppx, proj_y_max)
        pt = trace(cam_x, cam_y, d)
        if pt: border_pts.append(pt)

    # Left edge
    for ppy in range(proj_y_min, proj_y_max + 1, BORDER_PIXEL_STEP):
        cam_x, cam_y = p2c(proj_x_min, ppy)
        pt = trace(cam_x, cam_y, d)
        if pt: border_pts.append(pt)

    # Right edge
    for ppy in range(proj_y_min, proj_y_max + 1, BORDER_PIXEL_STEP):
        cam_x, cam_y = p2c(proj_x_max, ppy)
        pt = trace(cam_x, cam_y, d)
        if pt: border_pts.append(pt)

    print(f"Border: {len(border_pts)} points")
    create_obj_from_points(BORDER_NAME, border_pts,
                           color=(0.0, 0.5, 1.0, 1.0))

    # ------------------------------------------------------------------
    # 4. CORNER CIRCLES — filled circles with depth sweep for 3D visibility
    # ------------------------------------------------------------------
    print("Generating corner circles...")
    circle_pts = []
    cr = CORNER_RADIUS_PX + 3  # Slightly larger than the flat corner dots
    for cx, cy in corners:
        for dx in range(-cr, cr + 1):
            for dy in range(-cr, cr + 1):
                if dx * dx + dy * dy <= cr * cr:
                    ppx, ppy = cx + dx, cy + dy
                    if 0 <= ppx < RES_X and 0 <= ppy < RES_Y:
                        # Depth sweeps front-to-back across the circle
                        t = (dy + cr) / (2 * cr)
                        depth = DEPTH_FRONT + (DEPTH_BACK - DEPTH_FRONT) * t
                        cam_x, cam_y = p2c(ppx, ppy)
                        pt = trace(cam_x, cam_y, depth)
                        if pt: circle_pts.append(pt)

    print(f"Circles: {len(circle_pts)} points")
    create_obj_from_points(CIRCLES_NAME, circle_pts,
                           color=(0.0, 1.0, 1.0, 1.0))

    # ------------------------------------------------------------------
    # 5. DEPTH PROBES — off-plane points for FOV verification
    # ------------------------------------------------------------------
    # If the projector FOV is wrong, on-plane features can still line up
    # by adjusting distance. But off-plane probes introduce parallax:
    # only the correct FOV illuminates both on-plane and off-plane points.
    print("Generating depth probes...")
    depth_pts = []
    probe_idx = 0

    # Quarter-point probes along each edge at alternating front/back
    for frac in [0.25, 0.5, 0.75]:
        ppx = int(proj_x_min + (proj_x_max - proj_x_min) * frac)
        for ppy in [proj_y_min, proj_y_max]:
            depth = DEPTH_FRONT if (probe_idx % 2 == 0) else DEPTH_BACK
            probe_idx += 1
            cam_x, cam_y = p2c(ppx, ppy)
            pt = trace(cam_x, cam_y, depth)
            if pt: depth_pts.append(pt)

    for frac in [0.25, 0.5, 0.75]:
        ppy = int(proj_y_min + (proj_y_max - proj_y_min) * frac)
        for ppx in [proj_x_min, proj_x_max]:
            depth = DEPTH_BACK if (probe_idx % 2 == 0) else DEPTH_FRONT
            probe_idx += 1
            cam_x, cam_y = p2c(ppx, ppy)
            pt = trace(cam_x, cam_y, depth)
            if pt: depth_pts.append(pt)

    # Corner depth probes (offset inward from corners)
    offset = CORNER_RADIUS_PX + 12
    corner_probes = [
        (proj_x_min + offset, proj_y_min + offset, DEPTH_FRONT),
        (proj_x_max - offset, proj_y_min + offset, DEPTH_BACK),
        (proj_x_min + offset, proj_y_max - offset, DEPTH_BACK),
        (proj_x_max - offset, proj_y_max - offset, DEPTH_FRONT),
    ]
    for cpx, cpy, depth in corner_probes:
        if 0 <= cpx < RES_X and 0 <= cpy < RES_Y:
            cam_x, cam_y = p2c(cpx, cpy)
            pt = trace(cam_x, cam_y, depth)
            if pt: depth_pts.append(pt)

    # Interior grid probes at alternating depths
    for col in range(1, 5):
        for row in range(1, 3):
            gpx = int(RES_X * col / 5)
            gpy = int(RES_Y * row / 3)
            depth = DEPTH_FRONT if ((col + row) % 2 == 0) else DEPTH_BACK
            if 0 <= gpx < RES_X and 0 <= gpy < RES_Y:
                cam_x, cam_y = p2c(gpx, gpy)
                pt = trace(cam_x, cam_y, depth)
                if pt: depth_pts.append(pt)

    print(f"Depth probes: {len(depth_pts)} points "
          f"(front={DEPTH_FRONT}, back={DEPTH_BACK})")
    create_obj_from_points(DEPTH_NAME, depth_pts,
                           color=(1.0, 0.0, 0.8, 1.0))

    # ------------------------------------------------------------------
    # 6. EXPORT
    # ------------------------------------------------------------------
    all_points = (fracture_coords + align_pts + border_pts +
                  circle_pts + depth_pts)
    if DO_EXPORT:
        write_dxf_points(EXPORT_PATH, all_points)

    print(f"\nTotal: {len(all_points)} points")
    print(f"  Cloud:     {len(fracture_coords)}")
    print(f"  Alignment: {len(align_pts)}")
    print(f"  Border:    {len(border_pts)}")
    print(f"  Circles:   {len(circle_pts)}")
    print(f"  Probes:    {len(depth_pts)}")
    print("DONE")


# --- Run from Blender scripting play button ---
generate_laser_cloud()
