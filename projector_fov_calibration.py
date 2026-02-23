import bpy
import math
from mathutils import Vector
from mathutils.bvhtree import BVHTree

# ===================================================================
# PROJECTOR FOV CALIBRATION & ALIGNMENT SCRIPT
# ===================================================================
#
# PURPOSE:
#   1. FOV MEASUREMENT — Determine the exact horizontal and vertical
#      FOV of a 720p projector to the nearest 0.1 degrees.
#   2. POSE ALIGNMENT — Given a known FOV (default 37.6 deg horizontal),
#      generate calibration points inside the glass block so you can
#      physically align the projector on an optical mount to match the
#      Blender camera pose exactly.
#
# HOW THE FOV MEASUREMENT WORKS:
#   The script etches a "ruler" of vertical tick marks into the glass.
#   Each tick corresponds to a specific horizontal FOV value (37.0 to
#   39.5 in 0.1-degree steps). When you project a full-white 1280x720
#   image through the glass:
#     - Ticks inside the projected area will glow (they receive light)
#     - Ticks outside the projected area stay dark (no light reaches them)
#   The last glowing tick on each side tells you the FOV. For example,
#   if ticks up to "38.2" glow but "38.3" is dark, your HFOV is 38.2°.
#
#   The same logic applies vertically — horizontal tick marks are etched
#   at positions corresponding to vertical FOV values derived from the
#   16:9 aspect ratio.
#
#   Tick marks are grouped by whole-degree values. Within each degree,
#   the number of dashes in the tick encodes the tenth:
#     - 1 dash = x.0°,  2 dashes = x.1°, ... 10 dashes = x.9°
#   A longer "major" tick is placed at each whole degree boundary.
#
# HOW THE ALIGNMENT CALIBRATION WORKS:
#   Assuming HFOV = 37.6° (configurable), the script generates a test
#   pattern of fracture points designed for 6-DOF alignment:
#
#   1. CORNER DOTS — Filled circles at all 4 corners of the projection.
#      When correctly aligned, projector pixels at corners should light
#      up exactly these dots and no others.
#
#   2. CENTER CROSSHAIR — Vertical and horizontal lines through the
#      exact center of the projection. Aligning these to the physical
#      center pixel of the projector confirms pointing direction.
#
#   3. EDGE MIDPOINT MARKERS — Small tick marks at the midpoint of
#      each edge. These help verify there's no rotation (roll) — all
#      four midpoints should light up symmetrically.
#
#   4. BORDER FRAME — Full rectangular border at the projection edges.
#      The border uses depth interpolation (top=front of glass,
#      bottom=back of glass) so it's also useful for verifying the
#      projector's vertical angle and distance.
#
# USAGE:
#   1. Open Blender with a scene containing:
#      - A camera (the projector viewpoint)
#      - A cube named "Cube" (the glass block)
#   2. Set the Blender camera's horizontal FOV to the midpoint of your
#      expected range (e.g., 38.0°) for the FOV measurement pass.
#   3. Run this script — it generates point clouds and exports DXF.
#   4. Laser-etch the DXF into glass.
#   5. Project a full-white image and read off which ticks glow.
#   6. For alignment: update ALIGN_HFOV_DEG to your measured FOV,
#      set the Blender camera to match, and re-run with
#      GENERATE_FOV_RULER = False, GENERATE_ALIGNMENT = True.
#
# ===================================================================

# -------------------------------------------------------------------
# CONFIGURATION
# -------------------------------------------------------------------
EXPORT_PATH   = "C:/Users/johnh/Downloads/CalibrationOutput.dxf"
DO_EXPORT     = True

# --- Projector specs ---
RES_X         = 1280
RES_Y         = 720

# --- Scene objects ---
CUBE_NAME     = "Cube"

# --- Which calibration patterns to generate ---
GENERATE_FOV_RULER  = True     # Part 1: FOV measurement ticks
GENERATE_ALIGNMENT  = True     # Part 2: Pose alignment pattern

# --- FOV ruler settings ---
# Range of horizontal FOV values to mark (degrees)
HFOV_MIN_DEG  = 37.0
HFOV_MAX_DEG  = 39.5
HFOV_STEP_DEG = 0.1
# Tick mark dimensions (in pixels at the projector's native resolution)
TICK_LENGTH_MAJOR = 30   # Whole-degree tick height in pixels
TICK_LENGTH_MINOR = 15   # Sub-degree tick height in pixels
TICK_GAP          = 4    # Pixels between multi-dash ticks
TICK_Y_CENTER     = RES_Y // 2  # Vertical center for horizontal ruler

# Vertical FOV ruler (derived from HFOV via 16:9 aspect)
VFOV_MIN_DEG  = 20.0
VFOV_MAX_DEG  = 23.0
VFOV_STEP_DEG = 0.1
VTICK_LENGTH_MAJOR = 30
VTICK_LENGTH_MINOR = 15
VTICK_GAP          = 4
VTICK_X_CENTER     = RES_X // 2  # Horizontal center for vertical ruler

# --- Alignment settings (Part 2) ---
ALIGN_HFOV_DEG     = 37.6       # Assumed horizontal FOV for alignment
CORNER_RADIUS_PX   = 5          # Filled circle radius at corners
CROSSHAIR_LEN_PX   = 40         # Half-length of center crosshair arms
EDGE_TICK_LEN_PX   = 10         # Edge midpoint tick length
ALIGNMENT_INSET_PX = 2          # Pixels inset from true edge for border

# --- Glass / optics ---
INNER_CUBE_SCALE = 0.8
IOR_OUTSIDE      = 1.00
IOR_INSIDE       = 1.50
RAY_MAX_DIST     = 100000.0
POINT_RADIUS     = 0.00005

# --- Output object names ---
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
# CORE RAY PIPELINE
# -------------------------------------------------------------------
def build_ray_tracer(scene, camera, cube):
    """Returns a trace function: trace(pix_x, pix_y) -> world point or None.

    pix_x, pix_y are in the projector's native pixel coordinates
    (0..RES_X-1, 0..RES_Y-1). The returned point is placed at the
    midpoint depth of the inner safe zone along the refracted ray.
    """
    for poly in cube.data.polygons:
        poly.use_smooth = False
    cube.data.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    bvh = BVHTree.FromObject(cube, depsgraph)

    cube_mat = cube.matrix_world
    cube_mat_inv = cube_mat.inverted()
    normal_mat = cube_mat_inv.transposed().to_3x3()

    s = INNER_CUBE_SCALE
    box_min = Vector((-s, -s, -s))
    box_max = Vector((s, s, s))

    cam_origin, tl, tr, bl, br = get_camera_vectors(scene, camera)

    def trace(pix_x, pix_y, depth_factor=0.5):
        """Trace a ray for pixel (pix_x, pix_y) and return the world-space
        point at the given depth_factor (0.0=front, 1.0=back) within the
        inner safe zone. Returns None if the ray misses."""
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
# PART 1: FOV MEASUREMENT RULER
# -------------------------------------------------------------------
def hfov_to_pixel_x(hfov_deg, camera_hfov_deg):
    """Convert a horizontal FOV angle to the pixel column where the
    projection edge would fall if the projector had that FOV.

    The center of the sensor is pixel RES_X/2. The full angular width
    of the Blender camera spans RES_X pixels. A projector with a
    *different* FOV would have its edge pixels at different angular
    positions. We compute where the given hfov_deg boundary falls
    in the Blender camera's pixel space.

    half_angle = hfov_deg / 2 is the angle from center to edge.
    The Blender camera's half-angle = camera_hfov_deg / 2.
    Pixel offset from center = (tan(half_angle) / tan(cam_half)) * (RES_X/2)
    """
    half_angle = math.radians(hfov_deg / 2.0)
    cam_half = math.radians(camera_hfov_deg / 2.0)
    pixel_offset = (math.tan(half_angle) / math.tan(cam_half)) * (RES_X / 2.0)
    return RES_X / 2.0 + pixel_offset

def vfov_to_pixel_y(vfov_deg, camera_vfov_deg):
    """Same as above but for vertical FOV and pixel rows."""
    half_angle = math.radians(vfov_deg / 2.0)
    cam_half = math.radians(camera_vfov_deg / 2.0)
    pixel_offset = (math.tan(half_angle) / math.tan(cam_half)) * (RES_Y / 2.0)
    return RES_Y / 2.0 + pixel_offset

def get_camera_hfov(scene, camera):
    """Get the Blender camera's horizontal FOV in degrees."""
    render = scene.render
    sensor_fit = camera.data.sensor_fit
    focal_length = camera.data.lens

    aspect_x = render.resolution_x * render.pixel_aspect_x
    aspect_y = render.resolution_y * render.pixel_aspect_y

    if sensor_fit == 'HORIZONTAL' or (sensor_fit == 'AUTO' and aspect_x >= aspect_y):
        sensor_width = camera.data.sensor_width
    else:
        sensor_width = camera.data.sensor_height * (aspect_x / aspect_y)

    hfov_rad = 2.0 * math.atan(sensor_width / (2.0 * focal_length))
    return math.degrees(hfov_rad)

def get_camera_vfov(scene, camera):
    """Get the Blender camera's vertical FOV in degrees."""
    render = scene.render
    sensor_fit = camera.data.sensor_fit
    focal_length = camera.data.lens

    aspect_x = render.resolution_x * render.pixel_aspect_x
    aspect_y = render.resolution_y * render.pixel_aspect_y

    if sensor_fit == 'VERTICAL' or (sensor_fit == 'AUTO' and aspect_y > aspect_x):
        sensor_height = camera.data.sensor_height
    else:
        sensor_height = camera.data.sensor_width * (aspect_y / aspect_x)

    vfov_rad = 2.0 * math.atan(sensor_height / (2.0 * focal_length))
    return math.degrees(vfov_rad)

def generate_fov_ruler(trace):
    """Generate tick marks for FOV measurement.

    HORIZONTAL RULER: Vertical tick marks along the horizontal center,
    placed at pixel columns corresponding to each FOV value.
    Each tick is on the RIGHT side of the projection (positive offset
    from center). The LEFT side is symmetric, so measuring one side
    gives the full FOV.

    Encoding: For each 0.1° step, we draw N small dashes where N
    encodes the tenths digit (1 dash = x.0°, 2 = x.1°, ..., 10 = x.9°).
    Whole-degree boundaries get a longer major tick below the dashes.

    VERTICAL RULER: Same concept but horizontal ticks along the
    vertical center, on the BOTTOM side.
    """
    scene = bpy.context.scene
    cam = scene.camera
    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)

    print(f"Blender camera HFOV: {cam_hfov:.2f}°, VFOV: {cam_vfov:.2f}°")
    print(f"FOV ruler range: H={HFOV_MIN_DEG}°-{HFOV_MAX_DEG}°, "
          f"V={VFOV_MIN_DEG}°-{VFOV_MAX_DEG}°")

    points = []

    # --- Horizontal FOV ruler (vertical ticks on the right side) ---
    fov = HFOV_MIN_DEG
    while fov <= HFOV_MAX_DEG + 0.001:
        fov_rounded = round(fov, 1)
        px_right = hfov_to_pixel_x(fov_rounded, cam_hfov)
        px_left = RES_X - px_right  # Mirror on left side

        whole = int(fov_rounded)
        tenths = round((fov_rounded - whole) * 10)
        is_whole = (tenths == 0)

        # Number of dashes encodes the tenths digit
        # 0 tenths -> 1 dash (but it's the major tick), 1 tenth -> 1 dash, etc.
        num_dashes = tenths if tenths > 0 else 1

        # Major tick at whole degrees
        if is_whole:
            tick_h = TICK_LENGTH_MAJOR
        else:
            tick_h = TICK_LENGTH_MINOR

        # Draw dashes on the RIGHT side
        for d in range(num_dashes):
            dash_x = int(round(px_right)) + d * TICK_GAP
            if dash_x < 0 or dash_x >= RES_X:
                continue
            y_start = TICK_Y_CENTER - tick_h // 2
            y_end = TICK_Y_CENTER + tick_h // 2
            for y in range(max(0, y_start), min(RES_Y, y_end + 1)):
                pt = trace(dash_x, y)
                if pt:
                    points.append(pt)

        # Mirror: draw dashes on the LEFT side
        for d in range(num_dashes):
            dash_x = int(round(px_left)) - d * TICK_GAP
            if dash_x < 0 or dash_x >= RES_X:
                continue
            y_start = TICK_Y_CENTER - tick_h // 2
            y_end = TICK_Y_CENTER + tick_h // 2
            for y in range(max(0, y_start), min(RES_Y, y_end + 1)):
                pt = trace(dash_x, y)
                if pt:
                    points.append(pt)

        fov += HFOV_STEP_DEG
        fov = round(fov, 1)

    # --- Vertical FOV ruler (horizontal ticks on the bottom side) ---
    fov = VFOV_MIN_DEG
    while fov <= VFOV_MAX_DEG + 0.001:
        fov_rounded = round(fov, 1)
        py_bottom = vfov_to_pixel_y(fov_rounded, cam_vfov)
        py_top = RES_Y - py_bottom  # Mirror on top side

        whole = int(fov_rounded)
        tenths = round((fov_rounded - whole) * 10)
        is_whole = (tenths == 0)

        num_dashes = tenths if tenths > 0 else 1

        if is_whole:
            tick_w = VTICK_LENGTH_MAJOR
        else:
            tick_w = VTICK_LENGTH_MINOR

        # Draw dashes on the BOTTOM side
        for d in range(num_dashes):
            dash_y = int(round(py_bottom)) + d * VTICK_GAP
            if dash_y < 0 or dash_y >= RES_Y:
                continue
            x_start = VTICK_X_CENTER - tick_w // 2
            x_end = VTICK_X_CENTER + tick_w // 2
            for x in range(max(0, x_start), min(RES_X, x_end + 1)):
                pt = trace(x, dash_y)
                if pt:
                    points.append(pt)

        # Mirror: draw dashes on the TOP side
        for d in range(num_dashes):
            dash_y = int(round(py_top)) - d * VTICK_GAP
            if dash_y < 0 or dash_y >= RES_Y:
                continue
            x_start = VTICK_X_CENTER - tick_w // 2
            x_end = VTICK_X_CENTER + tick_w // 2
            for x in range(max(0, x_start), min(RES_X, x_end + 1)):
                pt = trace(x, dash_y)
                if pt:
                    points.append(pt)

        fov += VFOV_STEP_DEG
        fov = round(fov, 1)

    # --- Center reference line (always visible, confirms center) ---
    # Short vertical line at exact center
    cx, cy = RES_X // 2, RES_Y // 2
    for y in range(cy - 20, cy + 21):
        pt = trace(cx, y)
        if pt:
            points.append(pt)
    # Short horizontal line at exact center
    for x in range(cx - 20, cx + 21):
        pt = trace(x, cy)
        if pt:
            points.append(pt)

    print(f"FOV ruler: {len(points)} points generated")
    return points

# -------------------------------------------------------------------
# PART 2: ALIGNMENT CALIBRATION
# -------------------------------------------------------------------
def generate_alignment_pattern(trace):
    """Generate alignment calibration points assuming ALIGN_HFOV_DEG.

    The Blender camera should already be set to ALIGN_HFOV_DEG before
    running this. The alignment pattern uses the full RES_X x RES_Y
    pixel grid directly (no FOV conversion needed — the camera FOV
    IS the projector FOV in this mode).
    """
    points = []
    border_points = []
    inset = ALIGNMENT_INSET_PX

    # Effective pixel bounds
    x_min, x_max = inset, RES_X - 1 - inset
    y_min, y_max = inset, RES_Y - 1 - inset
    cx = RES_X // 2
    cy = RES_Y // 2

    # --- 1. Corner filled circles ---
    corners = [
        (x_min, y_min, 0.1),   # Top-left, front of glass
        (x_max, y_min, 0.1),   # Top-right, front of glass
        (x_min, y_max, 0.9),   # Bottom-left, back of glass
        (x_max, y_max, 0.9),   # Bottom-right, back of glass
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

    # --- 2. Center crosshair ---
    # Vertical arm
    for y in range(cy - CROSSHAIR_LEN_PX, cy + CROSSHAIR_LEN_PX + 1):
        if 0 <= y < RES_Y:
            pt = trace(cx, y)
            if pt:
                points.append(pt)
    # Horizontal arm
    for x in range(cx - CROSSHAIR_LEN_PX, cx + CROSSHAIR_LEN_PX + 1):
        if 0 <= x < RES_X:
            pt = trace(x, cy)
            if pt:
                points.append(pt)

    # --- 3. Edge midpoint markers ---
    # Top edge midpoint (vertical tick downward)
    for y in range(y_min, y_min + EDGE_TICK_LEN_PX):
        pt = trace(cx, y, 0.1)
        if pt:
            points.append(pt)
    # Bottom edge midpoint (vertical tick upward)
    for y in range(y_max - EDGE_TICK_LEN_PX, y_max + 1):
        pt = trace(cx, y, 0.9)
        if pt:
            points.append(pt)
    # Left edge midpoint (horizontal tick rightward)
    for x in range(x_min, x_min + EDGE_TICK_LEN_PX):
        pt = trace(x, cy)
        if pt:
            points.append(pt)
    # Right edge midpoint (horizontal tick leftward)
    for x in range(x_max - EDGE_TICK_LEN_PX, x_max + 1):
        pt = trace(x, cy)
        if pt:
            points.append(pt)

    # --- 4. Border frame with depth interpolation ---
    # Top edge (front of glass)
    for x in range(x_min, x_max + 1):
        pt = trace(x, y_min, 0.1)
        if pt:
            border_points.append(pt)
    # Bottom edge (back of glass)
    for x in range(x_min, x_max + 1):
        pt = trace(x, y_max, 0.9)
        if pt:
            border_points.append(pt)
    # Left edge (front->back interpolation top to bottom)
    for y in range(y_min, y_max + 1):
        depth = 0.1 + 0.8 * ((y - y_min) / max(1, y_max - y_min))
        pt = trace(x_min, y, depth)
        if pt:
            border_points.append(pt)
    # Right edge (front->back interpolation top to bottom)
    for y in range(y_min, y_max + 1):
        depth = 0.1 + 0.8 * ((y - y_min) / max(1, y_max - y_min))
        pt = trace(x_max, y, depth)
        if pt:
            border_points.append(pt)

    print(f"Alignment pattern: {len(points)} points, "
          f"border: {len(border_points)} points")
    return points, border_points

# -------------------------------------------------------------------
# MAIN
# -------------------------------------------------------------------
def generate_calibration():
    print("=" * 50)
    print("PROJECTOR FOV CALIBRATION & ALIGNMENT")
    print("=" * 50)

    scene = bpy.context.scene
    cube = bpy.data.objects.get(CUBE_NAME)
    cam = scene.camera
    if not cam or not cube:
        return print("Error: Missing Camera or Cube object")

    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)
    print(f"Camera HFOV: {cam_hfov:.2f}°  VFOV: {cam_vfov:.2f}°")
    print(f"Resolution: {RES_X}x{RES_Y}")

    trace = build_ray_tracer(scene, cam, cube)

    all_points = []

    # --- Part 1: FOV ruler ---
    if GENERATE_FOV_RULER:
        print("\n--- Generating FOV Ruler ---")
        ruler_pts = generate_fov_ruler(trace)
        create_obj_from_points(FOV_RULER_NAME, ruler_pts,
                               color=(0.0, 1.0, 0.0, 1.0))
        all_points.extend(ruler_pts)

    # --- Part 2: Alignment pattern ---
    if GENERATE_ALIGNMENT:
        print("\n--- Generating Alignment Pattern ---")
        align_pts, border_pts = generate_alignment_pattern(trace)
        create_obj_from_points(ALIGNMENT_NAME, align_pts,
                               color=(1.0, 1.0, 0.0, 1.0))
        create_obj_from_points(ALIGN_BORDER_NAME, border_pts,
                               color=(0.0, 0.5, 1.0, 1.0))
        all_points.extend(align_pts)
        all_points.extend(border_pts)

    # --- Export ---
    if DO_EXPORT and all_points:
        write_dxf_points(EXPORT_PATH, all_points)

    print(f"\nTotal calibration points: {len(all_points)}")
    print("Done.")

if __name__ == "__main__":
    generate_calibration()
