import bpy
import math
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
# -------------------------------------------------------------------
# PART 1: FOV RULER
#
#   SETUP: Set Blender camera FOV WIDER than the projector's max
#   possible FOV. If you think HFOV is 37.5-39.1, set camera to ~42 deg.
#   This ensures all ruler ticks fall within the camera's view.
#
#   The script etches vertical tick marks at angular positions
#   corresponding to HFOV values 37.0-39.5 in 0.1 deg steps.
#   Ticks are mirrored on both sides of the optical center.
#
#   READING: Project full-white 1280x720 through the glass.
#   Ticks inside the projector's cone glow. Outside = dark.
#   The last glowing tick = your FOV.
#
#   TICK HEIGHT ENCODING (no counting needed — just read the pattern):
#
#     .0 (whole degree) : 40px tall  — tallest, unmistakable landmark
#     .5 (half degree)  : 28px tall  — clear mid-point marker
#     other tenths      : 14px tall  — short ticks between landmarks
#
#     .0   .1  .2  .3  .4  .5   .6  .7  .8  .9  next .0
#     TALL  s   s   s   s  MED   s   s   s   s   TALL
#
#   You only ever count max 4 short ticks from the nearest landmark.
#   Example: last lit tick is 2 short ticks past a TALL tick → x.2 deg.
#
#   Horizontal ticks are vertical lines at the Y midline.
#   Vertical ticks are horizontal lines at the X midline.
#   A center crosshair confirms the optical axis.
#
#   The AnyBeam's laser scanning gives very crisp edge cutoff
#   (no vignetting like LCD/DLP), so the lit/dark boundary is sharp.
#
# -------------------------------------------------------------------
# PART 2: ALIGNMENT PATTERN
#
#   SETUP: Set Blender camera FOV = your measured projector FOV.
#   Now each projector pixel maps 1:1 to an angular position.
#
#   FEATURES:
#     Corner dots    — Verify X/Y position + FOV. Top=front, bottom=back.
#     Center cross   — Verify pointing direction (yaw + pitch).
#     Edge midpoints — Verify no roll. All 4 should be symmetric.
#     Border frame   — Depth-interpolated. Verify distance + tilt.
#     Sparse grid    — Interior dots. Catch distortion or local error.
#
# -------------------------------------------------------------------
# WORKFLOW:
#   1. Set Blender camera to ~42 deg HFOV (wider than projector)
#   2. Run with GENERATE_FOV_RULER=True, GENERATE_ALIGNMENT=False
#   3. Etch → project white → read last glowing tick
#   4. Set Blender camera to measured FOV (e.g. 37.6 deg)
#   5. Run with GENERATE_FOV_RULER=False, GENERATE_ALIGNMENT=True
#   6. Etch → align projector using test images
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

# Tick heights — the visual hierarchy
TICK_HEIGHT_WHOLE = 40    # .0 degrees (tallest landmark)
TICK_HEIGHT_HALF  = 28    # .5 degrees (mid landmark)
TICK_HEIGHT_TENTH = 14    # other tenths (short)

# Center reference crosshair half-arm length
CENTER_CROSS_LEN  = 25

# -------------------------------------------------------------------
# ALIGNMENT PATTERN CONFIG
# -------------------------------------------------------------------
ALIGN_HFOV_DEG     = 37.6
CORNER_RADIUS_PX   = 5
CROSSHAIR_LEN_PX   = 40
EDGE_TICK_LEN_PX   = 15
ALIGNMENT_INSET_PX = 2

# Interior alignment grid
GRID_ENABLED       = True
GRID_COLS          = 8
GRID_ROWS          = 5
GRID_DOT_RADIUS_PX = 2

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
FOV_RULER_NAME     = "FOV_Ruler"
ALIGNMENT_NAME     = "AlignmentPattern"
ALIGN_BORDER_NAME  = "AlignmentBorder"

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
    1=back) within the inner safe zone.
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
def hfov_to_pixel_x(hfov_deg, camera_hfov_deg):
    """For a projector with hfov_deg, its rightmost pixel lands at this
    pixel column in the Blender camera's pixel space.

    pixel = center + (RES_X/2) * tan(hfov/2) / tan(cam_hfov/2)
    """
    half_angle = math.radians(hfov_deg / 2.0)
    cam_half = math.radians(camera_hfov_deg / 2.0)
    pixel_offset = (math.tan(half_angle) / math.tan(cam_half)) * (RES_X / 2.0)
    return RES_X / 2.0 + pixel_offset

def vfov_to_pixel_y(vfov_deg, camera_vfov_deg):
    """Same as hfov_to_pixel_x but vertical."""
    half_angle = math.radians(vfov_deg / 2.0)
    cam_half = math.radians(camera_vfov_deg / 2.0)
    pixel_offset = (math.tan(half_angle) / math.tan(cam_half)) * (RES_Y / 2.0)
    return RES_Y / 2.0 + pixel_offset

# -------------------------------------------------------------------
# PART 1: FOV MEASUREMENT RULER
# -------------------------------------------------------------------
def get_tick_height(fov_rounded):
    """Tick height based on sub-degree value.

    .0 → TICK_HEIGHT_WHOLE (40px) — tallest landmark
    .5 → TICK_HEIGHT_HALF  (28px) — mid landmark
    else → TICK_HEIGHT_TENTH (14px) — short

    Between any two landmarks there are max 4 short ticks.
    """
    tenths = round((fov_rounded - int(fov_rounded)) * 10) % 10
    if tenths == 0:
        return TICK_HEIGHT_WHOLE
    elif tenths == 5:
        return TICK_HEIGHT_HALF
    else:
        return TICK_HEIGHT_TENTH

def generate_fov_ruler(trace):
    """Generate height-encoded tick marks for FOV measurement.

    Each tick is a single-pixel-wide vertical line (for HFOV) or
    horizontal line (for VFOV), placed at the angular position where
    a projector with that FOV would have its edge pixel.

    All ticks at glass midpoint depth (depth_factor=0.5).
    """
    scene = bpy.context.scene
    cam = scene.camera
    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)

    # Validate camera is wide enough for the ruler range
    max_h_pixel = hfov_to_pixel_x(HFOV_MAX_DEG, cam_hfov)
    max_v_pixel = vfov_to_pixel_y(VFOV_MAX_DEG, cam_vfov)
    if max_h_pixel >= RES_X:
        print(f"WARNING: Camera HFOV ({cam_hfov:.1f} deg) too narrow!")
        print(f"  Ruler max {HFOV_MAX_DEG} deg needs pixel {max_h_pixel:.0f}, "
              f"but camera only has {RES_X}px.")
        print(f"  Increase Blender camera FOV or decrease HFOV_MAX_DEG.")
    if max_v_pixel >= RES_Y:
        print(f"WARNING: Camera VFOV ({cam_vfov:.1f} deg) too narrow!")
        print(f"  Ruler max {VFOV_MAX_DEG} deg needs pixel {max_v_pixel:.0f}, "
              f"but camera only has {RES_Y}px.")

    print(f"Camera: HFOV={cam_hfov:.2f} deg, VFOV={cam_vfov:.2f} deg")
    print(f"H ruler: {HFOV_MIN_DEG}-{HFOV_MAX_DEG} deg | "
          f"V ruler: {VFOV_MIN_DEG}-{VFOV_MAX_DEG} deg")

    points = []
    y_center = RES_Y // 2
    x_center = RES_X // 2

    # === HORIZONTAL FOV RULER (vertical ticks at Y midline) ===
    fov = HFOV_MIN_DEG
    while fov <= HFOV_MAX_DEG + 0.001:
        fov_rounded = round(fov, 1)
        tick_h = get_tick_height(fov_rounded)
        half_h = tick_h // 2

        px_right = hfov_to_pixel_x(fov_rounded, cam_hfov)
        px_left = RES_X - px_right  # Mirror

        y_start = max(0, y_center - half_h)
        y_end = min(RES_Y - 1, y_center + half_h)

        # Right-side tick
        ix_r = int(round(px_right))
        if 0 <= ix_r < RES_X:
            for y in range(y_start, y_end + 1):
                pt = trace(ix_r, y)
                if pt:
                    points.append(pt)

        # Left-side tick (mirror)
        ix_l = int(round(px_left))
        if 0 <= ix_l < RES_X:
            for y in range(y_start, y_end + 1):
                pt = trace(ix_l, y)
                if pt:
                    points.append(pt)

        fov = round(fov + HFOV_STEP_DEG, 1)

    # === VERTICAL FOV RULER (horizontal ticks at X midline) ===
    fov = VFOV_MIN_DEG
    while fov <= VFOV_MAX_DEG + 0.001:
        fov_rounded = round(fov, 1)
        tick_w = get_tick_height(fov_rounded)  # Same height hierarchy
        half_w = tick_w // 2

        py_bottom = vfov_to_pixel_y(fov_rounded, cam_vfov)
        py_top = RES_Y - py_bottom  # Mirror

        x_start = max(0, x_center - half_w)
        x_end = min(RES_X - 1, x_center + half_w)

        # Bottom-side tick
        iy_b = int(round(py_bottom))
        if 0 <= iy_b < RES_Y:
            for x in range(x_start, x_end + 1):
                pt = trace(x, iy_b)
                if pt:
                    points.append(pt)

        # Top-side tick (mirror)
        iy_t = int(round(py_top))
        if 0 <= iy_t < RES_Y:
            for x in range(x_start, x_end + 1):
                pt = trace(x, iy_t)
                if pt:
                    points.append(pt)

        fov = round(fov + VFOV_STEP_DEG, 1)

    # === CENTER REFERENCE CROSSHAIR ===
    for y in range(y_center - CENTER_CROSS_LEN, y_center + CENTER_CROSS_LEN + 1):
        if 0 <= y < RES_Y:
            pt = trace(x_center, y)
            if pt:
                points.append(pt)
    for x in range(x_center - CENTER_CROSS_LEN, x_center + CENTER_CROSS_LEN + 1):
        if 0 <= x < RES_X:
            pt = trace(x, y_center)
            if pt:
                points.append(pt)

    print(f"FOV ruler: {len(points)} points")
    return points

# -------------------------------------------------------------------
# PART 2: ALIGNMENT CALIBRATION
# -------------------------------------------------------------------
def generate_alignment_pattern(trace):
    """Generate alignment calibration pattern.

    Blender camera FOV should equal ALIGN_HFOV_DEG so projector pixels
    map 1:1 to camera pixels. Features at pixel coords correspond
    directly to specific projector pixels.
    """
    points = []
    border_points = []
    inset = ALIGNMENT_INSET_PX

    x_min = inset
    x_max = RES_X - 1 - inset
    y_min = inset
    y_max = RES_Y - 1 - inset
    cx = RES_X // 2
    cy = RES_Y // 2

    # === 1. CORNER FILLED CIRCLES ===
    # Top corners at front of glass, bottom at back.
    # Depth split verifies distance AND vertical angle.
    corners = [
        (x_min, y_min, 0.1),   # Top-left, front
        (x_max, y_min, 0.1),   # Top-right, front
        (x_min, y_max, 0.9),   # Bottom-left, back
        (x_max, y_max, 0.9),   # Bottom-right, back
    ]
    for corner_x, corner_y, depth in corners:
        r = CORNER_RADIUS_PX
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    px = corner_x + dx
                    py = corner_y + dy
                    if 0 <= px < RES_X and 0 <= py < RES_Y:
                        pt = trace(px, py, depth)
                        if pt:
                            points.append(pt)

    # === 2. CENTER CROSSHAIR ===
    for y in range(cy - CROSSHAIR_LEN_PX, cy + CROSSHAIR_LEN_PX + 1):
        if 0 <= y < RES_Y:
            pt = trace(cx, y)
            if pt:
                points.append(pt)
    for x in range(cx - CROSSHAIR_LEN_PX, cx + CROSSHAIR_LEN_PX + 1):
        if 0 <= x < RES_X:
            pt = trace(x, cy)
            if pt:
                points.append(pt)

    # === 3. EDGE MIDPOINT MARKERS ===
    # Ticks pointing inward from each edge midpoint.
    # Top-center (front depth)
    for y in range(y_min, y_min + EDGE_TICK_LEN_PX):
        pt = trace(cx, y, 0.1)
        if pt:
            points.append(pt)
    # Bottom-center (back depth)
    for y in range(y_max - EDGE_TICK_LEN_PX + 1, y_max + 1):
        pt = trace(cx, y, 0.9)
        if pt:
            points.append(pt)
    # Left-center (mid depth)
    for x in range(x_min, x_min + EDGE_TICK_LEN_PX):
        pt = trace(x, cy)
        if pt:
            points.append(pt)
    # Right-center (mid depth)
    for x in range(x_max - EDGE_TICK_LEN_PX + 1, x_max + 1):
        pt = trace(x, cy)
        if pt:
            points.append(pt)

    # === 4. BORDER FRAME (depth-interpolated) ===
    # Top=front, bottom=back, sides interpolate linearly.
    for x in range(x_min, x_max + 1):
        pt = trace(x, y_min, 0.1)
        if pt:
            border_points.append(pt)
    for x in range(x_min, x_max + 1):
        pt = trace(x, y_max, 0.9)
        if pt:
            border_points.append(pt)
    for y in range(y_min, y_max + 1):
        depth = 0.1 + 0.8 * ((y - y_min) / max(1, y_max - y_min))
        pt = trace(x_min, y, depth)
        if pt:
            border_points.append(pt)
    for y in range(y_min, y_max + 1):
        depth = 0.1 + 0.8 * ((y - y_min) / max(1, y_max - y_min))
        pt = trace(x_max, y, depth)
        if pt:
            border_points.append(pt)

    # === 5. SPARSE ALIGNMENT GRID ===
    # Interior dots for catching distortion or local misalignment.
    if GRID_ENABLED:
        for col in range(1, GRID_COLS):
            for row in range(1, GRID_ROWS):
                gx = int(x_min + (x_max - x_min) * col / GRID_COLS)
                gy = int(y_min + (y_max - y_min) * row / GRID_ROWS)
                depth = 0.1 + 0.8 * ((gy - y_min) / max(1, y_max - y_min))
                r = GRID_DOT_RADIUS_PX
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        if dx * dx + dy * dy <= r * r:
                            px = gx + dx
                            py = gy + dy
                            if 0 <= px < RES_X and 0 <= py < RES_Y:
                                pt = trace(px, py, depth)
                                if pt:
                                    points.append(pt)

    print(f"Alignment: {len(points)} feature pts + "
          f"{len(border_points)} border pts")
    return points, border_points

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
    print(f"Camera: HFOV={cam_hfov:.2f} deg  VFOV={cam_vfov:.2f} deg")
    print(f"Projector: {RES_X}x{RES_Y} (Nebra AnyBeam)")

    trace = build_ray_tracer(scene, cam, cube)
    all_points = []

    if GENERATE_FOV_RULER:
        print("\n--- Part 1: FOV Ruler ---")
        ruler_pts = generate_fov_ruler(trace)
        create_obj_from_points(FOV_RULER_NAME, ruler_pts,
                               color=(0.0, 1.0, 0.0, 1.0))
        all_points.extend(ruler_pts)

    if GENERATE_ALIGNMENT:
        print("\n--- Part 2: Alignment Pattern ---")
        align_pts, border_pts = generate_alignment_pattern(trace)
        create_obj_from_points(ALIGNMENT_NAME, align_pts,
                               color=(1.0, 1.0, 0.0, 1.0))
        create_obj_from_points(ALIGN_BORDER_NAME, border_pts,
                               color=(0.0, 0.5, 1.0, 1.0))
        all_points.extend(align_pts)
        all_points.extend(border_pts)

    if DO_EXPORT and all_points:
        write_dxf_points(EXPORT_PATH, all_points)

    print(f"\nTotal: {len(all_points)} calibration points")
    print("DONE")

# --- Run from Blender scripting play button ---
generate_calibration()
