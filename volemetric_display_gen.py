import bpy
import math
import random
import os
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree

# -------------------------------------------------------------------
# CONFIGURATION
# -------------------------------------------------------------------
# --- OUTPUT ---
EXPORT_PATH   = "C:/Users/johnh/Downloads/LaserOutput.dxf"
DO_EXPORT     = True  

RES_X         = 256
RES_Y         = 144

CUBE_NAME     = "Cube"
PCLOUD_NAME   = "PixelPerfectCloud"
HELPER_NAME   = "AlignmentHelpers"
VIZ_VOL_NAME  = "Debug_InnerVolume"
VIZ_RAY_NAME  = "Debug_RayPaths"

FIT_MODE      = 'FIT'
ROTATE_90     = False         
FLIP_X        = False         
FLIP_Y        = False         

INNER_CUBE_SCALE = 0.8       

# --- DENSITY ---
PIXEL_STEP    = 1           
POINTS_PER_RAY = 1 
POINT_RADIUS  = 0.00005

# --- VISIBILITY ---
# Discard cloud points within this many pixels of the border/helpers
HELPER_MARGIN = 1  

# --- ALIGNMENT BORDER ---
PROJECTOR_HFOV_DEG = 37.6   # Measured projector HFOV (border maps to this)

# --- OPTICS ---
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
                f.write("0\nPOINT\n")
                f.write("8\n0\n") 
                f.write(f"10\n{p.x:.6f}\n") 
                f.write(f"20\n{p.y:.6f}\n") 
                f.write(f"30\n{p.z:.6f}\n") 
            f.write("0\nENDSEC\n0\nEOF\n")
        print(f"SUCCESS: Exported {len(points)} points to {filepath}")
    except Exception as e:
        print(f"ERROR: Could not write DXF file: {e}")

# -------------------------------------------------------------------
# GEOMETRY HELPERS
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
# FOV-TO-PIXEL MAPPING
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

def get_projector_pixel_bounds(cam_hfov, cam_vfov):
    """Map projector FOV edges into camera pixel coordinates (RES_X x RES_Y grid).

    Returns (px_left, px_right, py_top, py_bottom) as floats in the
    camera pixel grid.  When the camera FOV equals PROJECTOR_HFOV_DEG
    these collapse to (0, RES_X, 0, RES_Y).
    """
    proj_vfov = 2.0 * math.degrees(math.atan(
        math.tan(math.radians(PROJECTOR_HFOV_DEG / 2.0)) * RES_Y / RES_X))

    # Horizontal
    half_proj_h = math.tan(math.radians(PROJECTOR_HFOV_DEG / 2.0))
    half_cam_h  = math.tan(math.radians(cam_hfov / 2.0))
    px_right = (RES_X / 2.0) + (half_proj_h / half_cam_h) * (RES_X / 2.0)
    px_left  = RES_X - px_right

    # Vertical
    half_proj_v = math.tan(math.radians(proj_vfov / 2.0))
    half_cam_v  = math.tan(math.radians(cam_vfov / 2.0))
    py_bottom = (RES_Y / 2.0) + (half_proj_v / half_cam_v) * (RES_Y / 2.0)
    py_top    = RES_Y - py_bottom

    return px_left, px_right, py_top, py_bottom

# -------------------------------------------------------------------
# BLENDER SETUP
# -------------------------------------------------------------------
def create_emission_material(strength=5.0, color=(1.0, 1.0, 1.0, 1.0)):
    name = f"GlowMat_{strength}_{color[0]:.2f}"
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
        tree.interface.new_socket(name="Geometry", in_out='INPUT', socket_type='NodeSocketGeometry')
        tree.interface.new_socket(name="Geometry", in_out='OUTPUT', socket_type='NodeSocketGeometry')
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

def get_camera_vectors(scene, camera):
    frame = camera.data.view_frame(scene=scene)
    mat = camera.matrix_world
    world_frame = [mat @ v for v in frame]
    mat_inv = mat.inverted()
    local_frame = [mat_inv @ v for v in world_frame]
    sorted_y = sorted(zip(local_frame, world_frame), key=lambda x: x[0].y, reverse=True)
    top_set = sorted_y[:2]
    bot_set = sorted_y[2:]
    top_set.sort(key=lambda x: x[0].x)
    bot_set.sort(key=lambda x: x[0].x)
    return mat.translation, top_set[0][1], top_set[1][1], bot_set[0][1], bot_set[1][1]

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
    setup_point_rendering(obj, radius=POINT_RADIUS*1.5, color=color)
    return obj

# -------------------------------------------------------------------
# MAIN LOGIC
# -------------------------------------------------------------------
def generate_laser_cloud():
    print("=" * 40)
    scene = bpy.context.scene
    cube = bpy.data.objects.get(CUBE_NAME)
    cam = scene.camera
    if not cam or not cube: return print("Error: Missing Camera or Cube object")

    # --- SETUP BVH & MATRICES ---
    for poly in cube.data.polygons: poly.use_smooth = False
    cube.data.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    bvh = BVHTree.FromObject(cube, depsgraph)
    
    cube_mat = cube.matrix_world
    cube_mat_inv = cube_mat.inverted()
    normal_mat = cube_mat_inv.transposed().to_3x3()

    # --- BOUNDING BOXES ---
    # 1. Inner Box (Scaled)
    s_in = INNER_CUBE_SCALE
    box_min_inner = Vector((-s_in, -s_in, -s_in))
    box_max_inner = Vector(( s_in,  s_in,  s_in))

    # 2. Full Box (Unscaled / Scale 1.0) -- Used for alignment border
    s_full = 1.0
    box_min_full = Vector((-s_full, -s_full, -s_full))
    box_max_full = Vector(( s_full,  s_full,  s_full))

    cam_origin, tl, tr, bl, br = get_camera_vectors(scene, cam)

    # --- ASPECT RATIO CALCS ---
    render = scene.render
    render_aspect = (render.resolution_x * render.pixel_aspect_x) / \
                    (render.resolution_y * render.pixel_aspect_y)
    eff_w = RES_Y if ROTATE_90 else RES_X
    eff_h = RES_X if ROTATE_90 else RES_Y
    eff_aspect = eff_w / eff_h
    u_scale = 1.0
    v_scale = 1.0
    if FIT_MODE == 'FIT':
        if eff_aspect > render_aspect: v_scale = render_aspect / eff_aspect
        else: u_scale = eff_aspect / render_aspect
    elif FIT_MODE == 'FILL':
        if eff_aspect > render_aspect: u_scale = eff_aspect / render_aspect
        else: v_scale = render_aspect / eff_aspect

    # --- CORE RAY TRACING FUNCTION ---
    def get_ray_interval(pix_x, pix_y, b_min, b_max):
        """Returns (start_point, end_point, ray_dir_local) or None"""
        u_raw = (pix_x + 0.5) / RES_X
        v_raw = (pix_y + 0.5) / RES_Y
        
        if ROTATE_90: u_geom, v_geom = v_raw, u_raw
        else: u_geom, v_geom = u_raw, v_raw
        
        if FLIP_X: u_geom = 1.0 - u_geom
        if FLIP_Y: v_geom = 1.0 - v_geom
        
        u_cam = 0.5 + (u_geom - 0.5) * u_scale
        v_cam = 0.5 + (v_geom - 0.5) * v_scale

        vec_top = tl.lerp(tr, u_cam)
        vec_bot = bl.lerp(br, u_cam)
        target_pt = vec_bot.lerp(vec_top, v_cam)
        
        ray_dir_world = (target_pt - cam_origin).normalized()
        ray_origin_local = cube_mat_inv @ cam_origin
        ray_dir_local = (cube_mat_inv.to_3x3() @ ray_dir_world).normalized()
        
        hit1 = bvh.ray_cast(ray_origin_local, ray_dir_local, RAY_MAX_DIST)
        if not hit1[0]: return None
        
        loc_local, no_local, _, _ = hit1
        entry_point = cube_mat @ loc_local
        entry_normal = (normal_mat @ no_local).normalized()

        if entry_normal.dot(ray_dir_world) > -0.1: return None

        I_inside = refract(ray_dir_world, entry_normal, IOR_OUTSIDE, IOR_INSIDE)
        if not I_inside: return None
        
        ray_origin_local_2 = cube_mat_inv @ entry_point
        ray_dir_local_2 = (cube_mat_inv.to_3x3() @ I_inside).normalized()
        
        intervals = intersect_aabb(ray_origin_local_2, ray_dir_local_2, b_min, b_max)
        
        if intervals:
            t_enter, t_exit = intervals
            if t_exit > 0 and t_exit > t_enter:
                 return (ray_origin_local_2, ray_dir_local_2, max(0.0, t_enter), t_exit)
        return None

    # ----------------------------------------------
    # 1. GENERATE MAIN CLOUD (Using Inner Scaled Box)
    # ----------------------------------------------
    print(f"Generating Cloud ({RES_X}x{RES_Y})...")
    fracture_coords = []
    
    for x in range(0, RES_X, PIXEL_STEP):
        for y in range(0, RES_Y, PIXEL_STEP):
            
            # --- MARGIN CHECK ---
            # If (x, y) is within HELPER_MARGIN of the border, skip it
            if (x <= HELPER_MARGIN or x >= (RES_X - 1) - HELPER_MARGIN or 
                y <= HELPER_MARGIN or y >= (RES_Y - 1) - HELPER_MARGIN):
                continue
            
            res = get_ray_interval(x, y, box_min_inner, box_max_inner)
            if res:
                r_orig, r_dir, t_in, t_out = res
                ray_len = t_out - t_in
                step_size = ray_len / POINTS_PER_RAY
                
                for i in range(POINTS_PER_RAY):
                    base_t = t_in + (step_size * i)
                    t_val = base_t + (random.uniform(0.0, 1.0) * step_size)
                    p_local = r_orig + r_dir * t_val
                    fracture_coords.append(cube_mat @ p_local)

    create_obj_from_points(PCLOUD_NAME, fracture_coords, color=(1.0, 1.0, 1.0, 1.0))
    
    # ----------------------------------------------
    # 2. GENERATE ALIGNMENT BORDER (Projector FOV edges with interweaved depth)
    # ----------------------------------------------
    # Border maps to the projector's actual edge pixels at PROJECTOR_HFOV_DEG,
    # not the Blender camera edges. Every DEPTH_INTERLEAVE_N points alternate
    # between +/- depth offset for parallax verification.
    BORDER_BASE_DEPTH   = 0.5    # Mid-depth through the glass
    DEPTH_INTERLEAVE_N  = 5      # Every N points, apply a depth offset
    DEPTH_OFFSET_AMOUNT = 0.15   # Depth offset (fraction of ray segment)
    BORDER_PIXEL_STEP   = 1      # Trace every Nth projector pixel

    print("Generating Alignment Border...")
    helper_coords = []

    # Compute where projector FOV edges fall in camera pixel space
    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)
    px_left, px_right, py_top, py_bottom = get_projector_pixel_bounds(cam_hfov, cam_vfov)

    print(f"  Projector HFOV: {PROJECTOR_HFOV_DEG:.1f} deg")
    print(f"  Projector edges in cam pixels: X=[{px_left:.1f}, {px_right:.1f}] "
          f"Y=[{py_top:.1f}, {py_bottom:.1f}]")

    # Number of projector pixels along each axis
    n_proj_x = RES_X
    n_proj_y = RES_Y

    def proj_to_cam(proj_px, proj_py):
        """Convert projector pixel to camera pixel coordinate."""
        cam_x = px_left + (px_right - px_left) * proj_px / (n_proj_x - 1)
        cam_y = py_top + (py_bottom - py_top) * proj_py / (n_proj_y - 1)
        return cam_x, cam_y

    def trace_border_point(cam_x, cam_y, depth_factor):
        """Trace a single border point at the given camera pixel and depth."""
        res = get_ray_interval(cam_x, cam_y, box_min_full, box_max_full)
        if not res:
            return None
        r_orig, r_dir, t_in, t_out = res
        t_val = t_in + (t_out - t_in) * depth_factor
        p_local = r_orig + r_dir * t_val
        return cube_mat @ p_local

    # Border edges in projector pixel space
    proj_x_min = 0
    proj_x_max = n_proj_x - 1
    proj_y_min = 0
    proj_y_max = n_proj_y - 1

    border_edges = [
        ("TOP",    [(ppx, proj_y_min) for ppx in range(proj_x_min, proj_x_max + 1, BORDER_PIXEL_STEP)]),
        ("BOTTOM", [(ppx, proj_y_max) for ppx in range(proj_x_min, proj_x_max + 1, BORDER_PIXEL_STEP)]),
        ("LEFT",   [(proj_x_min, ppy) for ppy in range(proj_y_min, proj_y_max + 1, BORDER_PIXEL_STEP)]),
        ("RIGHT",  [(proj_x_max, ppy) for ppy in range(proj_y_min, proj_y_max + 1, BORDER_PIXEL_STEP)]),
    ]

    for label, pixel_list in border_edges:
        count = 0
        for proj_px, proj_py in pixel_list:
            cam_x, cam_y = proj_to_cam(proj_px, proj_py)

            # Interweaved depth: alternate +/- offset every N points
            group = (count // DEPTH_INTERLEAVE_N) % 2
            if group == 0:
                depth = BORDER_BASE_DEPTH + DEPTH_OFFSET_AMOUNT
            else:
                depth = BORDER_BASE_DEPTH - DEPTH_OFFSET_AMOUNT

            pt = trace_border_point(cam_x, cam_y, depth)
            if pt:
                helper_coords.append(pt)
            count += 1

    create_obj_from_points(HELPER_NAME, helper_coords, color=(1.0, 0.2, 0.0, 1.0))

    # ----------------------------------------------
    # 3. EXPORT
    # ----------------------------------------------
    if DO_EXPORT:
        total_points = helper_coords + fracture_coords
        write_dxf_points(EXPORT_PATH, total_points)

if __name__ == "__main__":
    generate_laser_cloud()