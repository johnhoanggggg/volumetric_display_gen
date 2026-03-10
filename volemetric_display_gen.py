import bpy
import math
import random
import os
import json
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree

# -------------------------------------------------------------------
# CONFIGURATION
# -------------------------------------------------------------------
# --- OUTPUT ---
EXPORT_PATH   = "C:/Users/johnh/Downloads/LaserOutput.dxf"
IMAGE_PATH    = "C:/Users/johnh/Downloads/ProjectorImage.png"
DO_EXPORT     = True
DO_EXPORT_IMAGE = True

RES_X         = 256
RES_Y         = 144

# --- PROJECTOR OUTPUT ---
PROJ_RES_X    = 1280          # Native projector resolution
PROJ_RES_Y    = 720
INFLATE       = False         # True = 1px per ray pixel with gaps; False = filled 5x5 blocks

CUBE_NAME     = "Cube"
CONTENT_NAME  = "ContentShape"   # 3D mesh to display (set "" to skip image gen)
PCLOUD_NAME   = "PixelPerfectCloud"
HELPER_NAME   = "AlignmentHelpers"
CONTENT_CLOUD_NAME = "ContentCloud"  # Visualization of content-hit fracture points
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

# --- CONTENT SURFACE SELECTION ---
# Max distance (world units) from a fracture point to the content surface
# for that pixel to be illuminated. Tune based on your glass/mesh scale.
SURFACE_THRESHOLD = 0.005

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

def write_config_txt(filepath, scene, cam, cube):
    """Export full setup configuration so a renderer can reconstruct the scene."""
    def mat4_rows(m):
        return [[m[row][col] for col in range(4)] for row in range(4)]
    def vec3(v):
        return [v.x, v.y, v.z]

    cam_origin, tl, tr, bl, br = get_camera_vectors(scene, cam)
    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)

    # Glass local bounding box
    local_bbox = [Vector(b) for b in cube.bound_box]
    bbox_min = [min(v[i] for v in local_bbox) for i in range(3)]
    bbox_max = [max(v[i] for v in local_bbox) for i in range(3)]

    # Glass world-space dimensions
    scale = cube.matrix_world.to_scale()
    world_size = [(bbox_max[i] - bbox_min[i]) * abs(scale[i]) for i in range(3)]

    config = {
        "generator": "volemetric_display_gen.py",

        # --- Projector / Camera ---
        "projector": {
            "resolution": [RES_X, RES_Y],
            "pixel_step": PIXEL_STEP,
            "projector_hfov_deg": PROJECTOR_HFOV_DEG,
            "camera_hfov_deg": round(cam_hfov, 6),
            "camera_vfov_deg": round(cam_vfov, 6),
            "camera_focal_length_mm": cam.data.lens,
            "camera_sensor_width_mm": cam.data.sensor_width,
            "camera_sensor_height_mm": cam.data.sensor_height,
            "camera_sensor_fit": cam.data.sensor_fit,
            "camera_position": vec3(cam_origin),
            "camera_matrix_world": mat4_rows(cam.matrix_world),
            "frustum_corners_world": {
                "top_left": vec3(tl),
                "top_right": vec3(tr),
                "bottom_left": vec3(bl),
                "bottom_right": vec3(br),
            },
        },

        # --- Glass Block ---
        "glass": {
            "object_name": CUBE_NAME,
            "matrix_world": mat4_rows(cube.matrix_world),
            "local_bbox_min": bbox_min,
            "local_bbox_max": bbox_max,
            "world_dimensions": world_size,
            "inner_cube_scale": INNER_CUBE_SCALE,
        },

        # --- Optics ---
        "optics": {
            "ior_outside": IOR_OUTSIDE,
            "ior_inside": IOR_INSIDE,
        },

        # --- Point Generation ---
        "point_generation": {
            "points_per_ray": POINTS_PER_RAY,
            "point_radius": POINT_RADIUS,
            "helper_margin_px": HELPER_MARGIN,
            "surface_threshold": SURFACE_THRESHOLD,
            "fit_mode": FIT_MODE,
            "rotate_90": ROTATE_90,
            "flip_x": FLIP_X,
            "flip_y": FLIP_Y,
        },

        # --- Alignment Border ---
        "alignment_border": {
            "border_base_depth": 0.5,
            "depth_interleave_n": 5,
            "depth_offset_amount": 0.15,
            "border_pixel_step": 1,
        },

        # --- Content (if present) ---
        "content": {
            "object_name": CONTENT_NAME if CONTENT_NAME else None,
        },
    }

    # If content object exists, add its transform
    content = bpy.data.objects.get(CONTENT_NAME) if CONTENT_NAME else None
    if content:
        config["content"]["matrix_world"] = mat4_rows(content.matrix_world)

    try:
        with open(filepath, 'w') as f:
            json.dump(config, f, indent=2)
        print(f"SUCCESS: Exported config to {filepath}")
    except Exception as e:
        print(f"ERROR: Could not write config file: {e}")

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

    # --- PROJECTOR-TO-CAMERA PIXEL MAPPING ---
    # All features (cloud + border) use projector pixel space mapped
    # through the 37.6 deg FOV so everything is in the same coordinate space.
    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)
    px_left, px_right, py_top, py_bottom = get_projector_pixel_bounds(cam_hfov, cam_vfov)

    print(f"  Projector HFOV: {PROJECTOR_HFOV_DEG:.1f} deg")
    print(f"  Projector edges in cam pixels: X=[{px_left:.1f}, {px_right:.1f}] "
          f"Y=[{py_top:.1f}, {py_bottom:.1f}]")

    def proj_to_cam(proj_px, proj_py):
        """Convert projector pixel to camera pixel coordinate."""
        cam_x = px_left + (px_right - px_left) * proj_px / (RES_X - 1)
        cam_y = py_top + (py_bottom - py_top) * proj_py / (RES_Y - 1)
        return cam_x, cam_y

    # ----------------------------------------------
    # 1. GENERATE MAIN CLOUD (Using Inner Scaled Box)
    # ----------------------------------------------
    # Also record which pixel produced each fracture point so we can
    # later decide which pixels to illuminate based on content proximity.
    print(f"Generating Cloud ({RES_X}x{RES_Y})...")
    fracture_coords = []
    pixel_to_points = {}   # (x, y) -> list of world-space Vector points

    for x in range(0, RES_X, PIXEL_STEP):
        for y in range(0, RES_Y, PIXEL_STEP):

            # --- MARGIN CHECK ---
            # If (x, y) is within HELPER_MARGIN of the border, skip it
            if (x <= HELPER_MARGIN or x >= (RES_X - 1) - HELPER_MARGIN or
                y <= HELPER_MARGIN or y >= (RES_Y - 1) - HELPER_MARGIN):
                continue

            cam_x, cam_y = proj_to_cam(x, y)
            res = get_ray_interval(cam_x, cam_y, box_min_inner, box_max_inner)
            if res:
                r_orig, r_dir, t_in, t_out = res
                ray_len = t_out - t_in
                step_size = ray_len / POINTS_PER_RAY
                pts = []

                for i in range(POINTS_PER_RAY):
                    base_t = t_in + (step_size * i)
                    t_val = base_t + (random.uniform(0.0, 1.0) * step_size)
                    p_local = r_orig + r_dir * t_val
                    p_world = cube_mat @ p_local
                    fracture_coords.append(p_world)
                    pts.append(p_world)

                pixel_to_points[(x, y)] = pts

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
    proj_x_max = RES_X - 1
    proj_y_min = 0
    proj_y_max = RES_Y - 1

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
    # 3. PROJECTOR IMAGE (content-targeted illumination)
    # ----------------------------------------------
    # For each pixel's fracture point(s), find the nearest point on the
    # ContentShape surface.  If the closest fracture point is within
    # SURFACE_THRESHOLD, light that pixel.  This selects the subset of
    # the volumetric cloud that sits on/near the target surface.

    # Auto-create ContentShape if it doesn't exist
    content = bpy.data.objects.get(CONTENT_NAME) if CONTENT_NAME else None
    if CONTENT_NAME and not content:
        print(f"  Creating default '{CONTENT_NAME}' (cube) inside glass block...")
        bpy.ops.mesh.primitive_cube_add(size=1.0,
                                         location=cube.matrix_world.translation)
        content = bpy.context.active_object
        content.name = CONTENT_NAME
        content.data.name = CONTENT_NAME
        cube_scale = cube.matrix_world.to_scale()
        content.scale = (cube_scale.x * INNER_CUBE_SCALE * 0.5,
                         cube_scale.y * INNER_CUBE_SCALE * 0.5,
                         cube_scale.z * INNER_CUBE_SCALE * 0.5)
        bpy.context.view_layer.update()
        depsgraph = bpy.context.evaluated_depsgraph_get()

    content_hit_points = []

    if content:
        print(f"\nGenerating projector image for '{CONTENT_NAME}'...")
        print(f"  Surface threshold: {SURFACE_THRESHOLD}")

        # Build BVH for content mesh
        for poly in content.data.polygons:
            poly.use_smooth = False
        content.data.update()
        content_bvh = BVHTree.FromObject(content, depsgraph)
        content_mat = content.matrix_world
        content_mat_inv = content_mat.inverted()

        # Determine which pixels hit the content surface
        hit_pixels = {}  # (x, y) -> best_point
        hit_count = 0

        for (x, y), pts in pixel_to_points.items():
            best_dist = float('inf')
            best_point = None

            for p_world in pts:
                p_local = content_mat_inv @ p_world
                nearest = content_bvh.find_nearest(p_local)
                if nearest[0] is None:
                    continue
                dist = nearest[3]
                if dist < best_dist:
                    best_dist = dist
                    best_point = p_world

            if best_dist > SURFACE_THRESHOLD:
                continue

            hit_count += 1
            hit_pixels[(x, y)] = best_point
            content_hit_points.append(best_point)

        print(f"  Content hits: {hit_count} / {len(pixel_to_points)} pixels "
              f"({100*hit_count/max(1,len(pixel_to_points)):.1f}%)")

        # Upscale to projector native resolution (720p)
        scale_x = PROJ_RES_X // RES_X  # 5
        scale_y = PROJ_RES_Y // RES_Y  # 5
        pixels = [0.0, 0.0, 0.0, 1.0] * (PROJ_RES_X * PROJ_RES_Y)

        for (x, y) in hit_pixels:
            flipped_y = (RES_Y - 1) - y
            if INFLATE:
                # Single pixel at mapped position (4px gap between neighbors)
                out_x = x * scale_x
                out_y = flipped_y * scale_y
                idx = (out_y * PROJ_RES_X + out_x) * 4
                pixels[idx]     = 1.0
                pixels[idx + 1] = 1.0
                pixels[idx + 2] = 1.0
            else:
                # Fill scale_x * scale_y block (nearest-neighbor upscale)
                base_x = x * scale_x
                base_y = flipped_y * scale_y
                for dy in range(scale_y):
                    for dx in range(scale_x):
                        idx = ((base_y + dy) * PROJ_RES_X + (base_x + dx)) * 4
                        pixels[idx]     = 1.0
                        pixels[idx + 1] = 1.0
                        pixels[idx + 2] = 1.0

        mode_label = "INFLATE (1px per ray)" if INFLATE else f"FILLED ({scale_x}x{scale_y} blocks)"
        print(f"  Output: {PROJ_RES_X}x{PROJ_RES_Y} — {mode_label}")

        # Create Blender image
        img_name = "ProjectorImage"
        img = bpy.data.images.get(img_name)
        if img:
            bpy.data.images.remove(img)
        img = bpy.data.images.new(img_name, PROJ_RES_X, PROJ_RES_Y, alpha=False)
        img.pixels = pixels

        if DO_EXPORT_IMAGE:
            try:
                img.filepath_raw = IMAGE_PATH
                img.file_format = 'PNG'
                img.save()
                print(f"  Projector image saved: {IMAGE_PATH}")
            except Exception as e:
                print(f"  WARNING: Could not save image: {e}")
                print("  (Image still available in Blender as 'ProjectorImage')")

        # Add image as camera background for verification
        cam.data.show_background_images = True
        for bg in list(cam.data.background_images):
            if bg.image and bg.image.name == img_name:
                cam.data.background_images.remove(bg)
        bg = cam.data.background_images.new()
        bg.image = img
        bg.alpha = 0.5
        bg.display_depth = 'FRONT'

        # Visualization: content-hit fracture points (the selected subset)
        if content_hit_points:
            create_obj_from_points(CONTENT_CLOUD_NAME, content_hit_points,
                                   color=(0.0, 1.0, 0.5, 1.0))
            print(f"  Content point cloud: '{CONTENT_CLOUD_NAME}' "
                  f"({len(content_hit_points)} points)")
    elif CONTENT_NAME:
        print(f"\nWARNING: No object named '{CONTENT_NAME}' in scene — "
              f"skipping projector image generation.")
        print(f"  Add a mesh named '{CONTENT_NAME}' to generate the illumination image.")

    # ----------------------------------------------
    # 4. EXPORT
    # ----------------------------------------------
    if DO_EXPORT:
        total_points = helper_coords + fracture_coords
        write_dxf_points(EXPORT_PATH, total_points)

        config_path = os.path.splitext(EXPORT_PATH)[0] + "_config.txt"
        write_config_txt(config_path, scene, cam, cube)

if __name__ == "__main__":
    generate_laser_cloud()