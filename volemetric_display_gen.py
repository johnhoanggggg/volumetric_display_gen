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

# --- INPUT IMAGE MODE ---
# Set to a PNG path to visualize which fracture points that image illuminates.
# The image is sampled at RES_X x RES_Y (downscaled from 720p if needed).
# Set to "" to disable.
INPUT_IMAGE_PATH  = ""
INPUT_CLOUD_NAME  = "InputImageCloud"   # Blender object for illuminated points
INPUT_BRIGHTNESS_THRESHOLD = 0.5        # Pixel brightness above this = "lit"

CUBE_NAME     = "Cube"
CONTENT_NAME  = "ContentShape"   # 3D mesh to display (set "" to skip image gen)
PCLOUD_NAME   = "PixelPerfectCloud"
HELPER_NAME   = "AlignmentHelpers"
EDGE_NAME     = "InnerCubeEdges"
CONTENT_CLOUD_NAME = "ContentCloud"  # Visualization of content-hit fracture points
VIZ_VOL_NAME  = "Debug_InnerVolume"
VIZ_RAY_NAME  = "Debug_RayPaths"

FIT_MODE      = 'FIT'
ROTATE_90     = False
FLIP_X        = False
FLIP_Y        = False

INNER_CUBE_SCALE = 0.9

# --- DENSITY ---
PIXEL_STEP    = 1
POINTS_PER_RAY = 1
RANDOM_SEED   = 42            # Fixed seed for reproducible point placement (None = random each run)
POINT_RADIUS  = 0.00005

# --- INNER CUBE EDGES ---
EDGE_POINT_DENSITY = 100.0    # Points per local-space unit along each edge

# --- VISIBILITY ---
# Discard cloud points within this many pixels of the border/helpers
HELPER_MARGIN = 1

# --- ALIGNMENT BORDER ---
PROJECTOR_HFOV_DEG = 38   # Measured projector HFOV (border maps to this)
CORNER_CIRCLE_RADIUS_PX = 3 # Corner circle radius in projector pixels
CORNER_CIRCLE_DENSITY   = 1 # Sub-pixel steps per pixel for circle fill

# --- CONTENT SURFACE SELECTION ---
# Max distance (world units) from a fracture point to the content surface
# for that pixel to be illuminated. Tune based on your glass/mesh scale.
SURFACE_THRESHOLD = 0.005

# --- OPTICS ---
IOR_OUTSIDE   = 1.00
IOR_INSIDE    = 1.50

# --- PHYSICAL ALIGNMENT (see ALIGNMENT.md) ---
# The real etched cloud never sits exactly where Blender put it, and the real
# projector never sits exactly where the camera is.  These options etch
# fiducials into the block, then render images for the MEASURED pose.
FIDUCIAL_JSON_PATH    = ""     # alignment_fiducials.json from projector_alignment_fiducials.py:
                               #   etch its fiducials + glyph into this cloud and keep
                               #   cloud rays clear of their pixels
ALIGNMENT_RESULT_PATH = ""     # alignment_result.json from alignment_tool/index.html:
                               #   render ProjectorImage.png by projecting each content
                               #   point through the measured projector pose
SPOT_RADIUS_PX        = 1      # Measured-pose image: light (2r+1)^2 px per point
SYMMETRIC_BBOX        = True   # Add 6 points so the DXF bounding box is centred on the
                               #   block (etching shops usually centre the bounding box)
MIN_SURFACE_MM        = 2.0    # Warn about points closer than this to a glass face
BLOCK_UNIT_SCALE      = 0.001  # mm -> Blender units (0.001 = scene in metres)
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
# PHYSICAL ALIGNMENT HELPERS (same maths as alignment_tool/alignment_core.js)
# -------------------------------------------------------------------
def get_glass_frame(cube):
    """Returns (G, half_mm): G maps glass-frame Blender units -> world.
    Glass frame = block centre, block axes; fiducial files use it in mm."""
    mw = cube.matrix_world
    _, rot, scl = mw.decompose()
    bb = [Vector(c) for c in cube.bound_box]
    bb_min = Vector((min(v[i] for v in bb) for i in range(3)))
    bb_max = Vector((max(v[i] for v in bb) for i in range(3)))
    center_world = mw @ ((bb_min + bb_max) * 0.5)
    half_units = [(bb_max[i] - bb_min[i]) * 0.5 * abs(scl[i]) for i in range(3)]
    G = Matrix.Translation(center_world) @ rot.to_matrix().to_4x4()
    return G, Vector([h / BLOCK_UNIT_SCALE for h in half_units])

def check_design_matches_scene(rig, cam, G):
    """Warn if a fiducial design was made for a different camera / FOV."""
    cam_loc, cam_rot, _ = cam.matrix_world.decompose()
    pos = (G.inverted() @ cam_loc) / BLOCK_UNIT_SCALE
    R = G.to_3x3().inverted() @ cam_rot.to_matrix()
    d_pos = (pos - Vector(rig["projector"]["position"])).length
    d_rot = max(abs(R[i][j] - rig["projector"]["rotation"][i][j])
                for i in range(3) for j in range(3))
    ok = d_pos < 0.01 and d_rot < 1e-4 and abs(rig["hfov_deg"] - PROJECTOR_HFOV_DEG) < 1e-6
    if not ok:
        print(f"  WARNING: fiducial design was made for a different camera pose / HFOV "
              f"(moved {d_pos:.3f} mm, HFOV {rig['hfov_deg']} vs {PROJECTOR_HFOV_DEG}). "
              f"Re-run projector_alignment_fiducials.py with this scene.")
    return ok

def solved_projector(result):
    """Projector model from alignment_result.json (glass frame, mm)."""
    rig, sv = result["design"]["rig"], result["solved"]
    W = rig["res"][0]
    return {
        "C": Vector(sv["position_mm"]),
        "R": Matrix(sv["rotation"]),
        "f": (W / 2.0) / math.tan(math.radians(sv["hfov_deg"]) / 2.0),
        "cx": sv["cx"], "cy": sv["cy"], "ior": sv["ior"],
        "scale": sv.get("cloud_scale", 1.0),
        "dscale": sv.get("cloud_depth_scale", 1.0),
        "half": rig["glass"]["half_mm"],
        "axis": rig["glass"]["front_axis"],
        "sign": rig["glass"]["front_sign"],
    }

def point_to_pixel(proj, P):
    """Glass-frame point (mm) -> continuous projector pixel (rows from the
    bottom) that lights it, or None.  Solves Snell's law at the flat front
    face (bisection in the plane of incidence), then projects the entry
    point.  Port of pointToPixel() in alignment_tool/alignment_core.js."""
    a, sgn, C = proj["axis"], proj["sign"], proj["C"]
    Q = Vector(P) * proj["scale"]
    Q[a] *= proj["dscale"]
    h = sgn * proj["half"][a]
    hc = sgn * (C[a] - h)
    hp = sgn * (h - Q[a])
    if hc <= 0 or hp < 0:
        return None
    b, c = [i for i in range(3) if i != a]
    Db, Dc = Q[b] - C[b], Q[c] - C[c]
    D = math.hypot(Db, Dc)
    n = proj["ior"]
    x = D
    if D > 1e-12 and hp > 0:
        lo, hi = 0.0, D
        for _ in range(60):
            x = 0.5 * (lo + hi)
            g = x / math.hypot(x, hc) - n * (D - x) / math.hypot(D - x, hp)
            if g > 0: hi = x
            else: lo = x
    E = Vector((0.0, 0.0, 0.0))
    E[a] = h
    E[b] = C[b] + (Db / D * x if D > 1e-12 else 0.0)
    E[c] = C[c] + (Dc / D * x if D > 1e-12 else 0.0)
    q = proj["R"].transposed() @ (E - C)
    if q.z >= 0:
        return None
    return proj["cx"] + proj["f"] * q.x / -q.z, proj["cy"] + proj["f"] * q.y / -q.z

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

def check_physical_placement(points, G, half_mm):
    """Reports where an etching shop will put this cloud and returns the
    extra points needed to centre its bounding box on the block.

    Shops typically centre the model's bounding box in the glass, so an
    asymmetric cloud is etched off-centre by half the asymmetry.  Six
    points at the mirrored extremes (on the axes, as far from the faces as
    the existing extremes) make the bounding box symmetric."""
    G_inv = G.inverted()
    local = [(G_inv @ p) / BLOCK_UNIT_SCALE for p in points]
    lo = [min(v[i] for v in local) for i in range(3)]
    hi = [max(v[i] for v in local) for i in range(3)]
    centre = [(lo[i] + hi[i]) / 2 for i in range(3)]
    print(f"  DXF bounding box centre vs block centre: "
          f"({centre[0]:+.2f}, {centre[1]:+.2f}, {centre[2]:+.2f}) mm")
    gap = min(half_mm[i] - max(abs(lo[i]), abs(hi[i])) for i in range(3))
    close = sum(1 for v in local if min(half_mm[i] - abs(v[i]) for i in range(3)) < MIN_SURFACE_MM)
    if close:
        print(f"  WARNING: {close} points are within {MIN_SURFACE_MM} mm of a glass face "
              f"(closest {gap:.2f} mm). Shops may refuse, clip or shrink the model to fit; "
              f"keep everything inside the safe zone.")
    if not SYMMETRIC_BBOX or max(abs(c) for c in centre) < 0.005:
        return []
    ext = [max(abs(lo[i]), abs(hi[i])) for i in range(3)]
    extra = []
    for i in range(3):
        for sgn in (-1, 1):
            v = Vector((0.0, 0.0, 0.0))
            v[i] = sgn * ext[i]
            extra.append(G @ (v * BLOCK_UNIT_SCALE))
    print(f"  Added 6 bounding-box points: a shop that centres the model would "
          f"otherwise etch it ({-centre[0]:+.2f}, {-centre[1]:+.2f}, {-centre[2]:+.2f}) mm off")
    return extra

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
        # Pixel k is centred at (k + 0.5) / RES of the projector span; the
        # tracer adds the 0.5 back.  (Spreading over RES - 1 drifted rays up
        # to half a pixel from the image pixel that is meant to light them.)
        cam_x = px_left + (px_right - px_left) * (proj_px + 0.5) / RES_X - 0.5
        cam_y = py_top + (py_bottom - py_top) * (proj_py + 0.5) / RES_Y - 0.5
        return cam_x, cam_y

    # --- FIDUCIALS (optional) ---
    # Ray-grid pixel (x, y) is lit by projector pixels around
    # ((x + 0.5) * PROJ_RES_X / RES_X, ...), so cloud rays near a fiducial's
    # pixel are dropped: only the fiducial lights up while measuring it.
    G, half_mm = get_glass_frame(cube)
    fiducial_design = None
    fiducial_coords = []
    if FIDUCIAL_JSON_PATH:
        with open(FIDUCIAL_JSON_PATH) as fh:
            fiducial_design = json.load(fh)
        check_design_matches_scene(fiducial_design["rig"], cam, G)
        if FIT_MODE != 'FIT' or ROTATE_90 or FLIP_X or FLIP_Y or \
                abs(render_aspect - PROJ_RES_X / PROJ_RES_Y) > 1e-6:
            print("  WARNING: fiducial keep-out assumes FIT, no rotate/flip and a "
                  f"{PROJ_RES_X}x{PROJ_RES_Y} render aspect.")
        to_world = lambda p: G @ (Vector(p) * BLOCK_UNIT_SCALE)
        fiducial_coords = [to_world(f["point_mm"]) for f in fiducial_design["fiducials"]]
        fiducial_coords += [to_world(p) for p in fiducial_design.get("glyph_mm", [])]
        keep_out = fiducial_design.get("keep_out_px", 20)
        fid_pixels = [(f["pixel"][0] + 0.5, f["pixel"][1] + 0.5)
                      for f in fiducial_design["fiducials"]]
        print(f"  Fiducials: {len(fid_pixels)} (+{len(fiducial_coords) - len(fid_pixels)} glyph points), "
              f"keep-out +/-{keep_out} projector px")

    def near_fiducial(x, y):
        if not fiducial_design:
            return False
        pu = (x + 0.5) * PROJ_RES_X / RES_X
        pv = (y + 0.5) * PROJ_RES_Y / RES_Y
        return any(abs(pu - fu) <= keep_out and abs(pv - fv) <= keep_out
                   for fu, fv in fid_pixels)

    # ----------------------------------------------
    # 1. GENERATE MAIN CLOUD (Using Inner Scaled Box)
    # ----------------------------------------------
    # Also record which pixel produced each fracture point so we can
    # later decide which pixels to illuminate based on content proximity.
    print(f"Generating Cloud ({RES_X}x{RES_Y})...")
    if RANDOM_SEED is not None:
        random.seed(RANDOM_SEED)
    fracture_coords = []
    pixel_to_points = {}   # (x, y) -> list of world-space Vector points

    for x in range(0, RES_X, PIXEL_STEP):
        for y in range(0, RES_Y, PIXEL_STEP):

            # --- MARGIN CHECK ---
            # If (x, y) is within HELPER_MARGIN of the border, skip it
            if (x <= HELPER_MARGIN or x >= (RES_X - 1) - HELPER_MARGIN or
                y <= HELPER_MARGIN or y >= (RES_Y - 1) - HELPER_MARGIN):
                continue
            if near_fiducial(x, y):
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
    DEPTH_INTERLEAVE_N  = 20     # Every N points, apply a depth offset
    DEPTH_OFFSET_AMOUNT = 0.45   # Depth offset (fraction of ray segment)
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

    TICK_LENGTH_PX = 3   # perpendicular tick length in projector pixels
    TICK_DENSITY   = 5   # sub-pixel steps per pixel along tick
    TICK_EVERY_N   = 3   # place tick on every Nth back-plane segment

    def trace_tick(mid_px, mid_py, label):
        """Trace a dense perpendicular tick centered on the border at (mid_px, mid_py)."""
        is_horiz = label in ("TOP", "BOTTOM")
        sign = 1 if label in ("TOP", "LEFT") else -1
        half = TICK_LENGTH_PX / 2.0
        n_steps = TICK_LENGTH_PX * TICK_DENSITY
        for i in range(n_steps + 1):
            frac = -half + (TICK_LENGTH_PX * i / n_steps)
            if is_horiz:
                tx, ty = mid_px, mid_py + sign * frac
            else:
                tx, ty = mid_px + sign * frac, mid_py
            tcx, tcy = proj_to_cam(tx, ty)
            tpt = trace_border_point(tcx, tcy, BORDER_BASE_DEPTH + DEPTH_OFFSET_AMOUNT)
            if tpt:
                helper_coords.append(tpt)

    for label, pixel_list in border_edges:
        count = 0
        back_seg_count = 0
        seg_points = []  # accumulate points in current segment
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

            # Track back-plane segment pixels for midpoint tick
            if group == 0:
                seg_points.append((proj_px, proj_py))
            else:
                if seg_points:
                    if back_seg_count % TICK_EVERY_N == 0:
                        mid_px, mid_py = seg_points[len(seg_points) // 2]
                        trace_tick(mid_px, mid_py, label)
                    back_seg_count += 1
                    seg_points = []

            count += 1

        # Handle last segment if it was back-plane
        if seg_points:
            if back_seg_count % TICK_EVERY_N == 0:
                mid_px, mid_py = seg_points[len(seg_points) // 2]
                trace_tick(mid_px, mid_py, label)

    # Corner circles on the front offset plane for alignment starting point
    print("Generating Corner Circles...")
    front_depth = BORDER_BASE_DEPTH - DEPTH_OFFSET_AMOUNT
    corner_pixels = [
        (proj_x_min, proj_y_min),  # top-left
        (proj_x_max, proj_y_min),  # top-right
        (proj_x_min, proj_y_max),  # bottom-left
        (proj_x_max, proj_y_max),  # bottom-right
    ]
    r = CORNER_CIRCLE_RADIUS_PX
    step = 1.0 / CORNER_CIRCLE_DENSITY
    corner_count = 0
    for cx, cy in corner_pixels:
        sx = cx - r
        while sx <= cx + r:
            sy = cy - r
            while sy <= cy + r:
                dx, dy = sx - cx, sy - cy
                if dx * dx + dy * dy <= r * r:
                    tcx, tcy = proj_to_cam(sx, sy)
                    pt = trace_border_point(tcx, tcy, front_depth)
                    if pt:
                        helper_coords.append(pt)
                        corner_count += 1
                sy += step
            sx += step
    print(f"  {corner_count} corner circle points (4 corners, r={r}px)")

    create_obj_from_points(HELPER_NAME, helper_coords, color=(1.0, 0.2, 0.0, 1.0))

    # ----------------------------------------------
    # 2b. INNER CUBE EDGE MICROFRACTURES
    # ----------------------------------------------
    # Place fracture points along all 12 edges of the inner safe-zone cube.
    # These are NOT illuminated by the projector — they serve as physical
    # registration marks visible under ambient light.
    print("Generating Inner Cube Edge fractures...")
    edge_coords = []

    corners = [
        Vector((-s_in, -s_in, -s_in)),
        Vector(( s_in, -s_in, -s_in)),
        Vector(( s_in,  s_in, -s_in)),
        Vector((-s_in,  s_in, -s_in)),
        Vector((-s_in, -s_in,  s_in)),
        Vector(( s_in, -s_in,  s_in)),
        Vector(( s_in,  s_in,  s_in)),
        Vector((-s_in,  s_in,  s_in)),
    ]
    edges = [
        (0,1),(1,2),(2,3),(3,0),  # front face
        (4,5),(5,6),(6,7),(7,4),  # back face
        (0,4),(1,5),(2,6),(3,7),  # connecting
    ]

    for a_idx, b_idx in edges:
        a, b = corners[a_idx], corners[b_idx]
        a_world = cube_mat @ a
        b_world = cube_mat @ b
        length = (b_world - a_world).length
        n_pts = max(1, round(length * EDGE_POINT_DENSITY))
        for i in range(n_pts):
            t = (i + 0.5) / n_pts
            p_local = a.lerp(b, t)
            edge_coords.append(cube_mat @ p_local)

    print(f"  {len(edge_coords)} edge points (12 edges, {EDGE_POINT_DENSITY} pts/unit)")
    create_obj_from_points(EDGE_NAME, edge_coords, color=(0.0, 0.6, 1.0, 1.0))

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

        if ALIGNMENT_RESULT_PATH:
            # Light the pixel that the MEASURED projector pose maps onto each
            # selected (etched) point, instead of the pixel it was designed for.
            with open(ALIGNMENT_RESULT_PATH) as fh:
                result = json.load(fh)
            check_design_matches_scene(result["design"]["rig"], cam, G)
            proj = solved_projector(result)
            G_inv = G.inverted()
            r = SPOT_RADIUS_PX
            missed = 0
            for p_world in hit_pixels.values():
                uv = point_to_pixel(proj, (G_inv @ p_world) / BLOCK_UNIT_SCALE)
                if not uv:
                    missed += 1
                    continue
                pu, pv = int(math.floor(uv[0])), int(math.floor(uv[1]))
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        X, Y = pu + dx, pv + dy
                        if 0 <= X < PROJ_RES_X and 0 <= Y < PROJ_RES_Y:
                            idx = (Y * PROJ_RES_X + X) * 4
                            pixels[idx:idx + 3] = [1.0, 1.0, 1.0]
            if missed:
                print(f"  WARNING: {missed} points not reachable through the front face")

        for (x, y) in ([] if ALIGNMENT_RESULT_PATH else hit_pixels):
            # Blender image rows start at the bottom, like ray-grid rows (no flip)
            row = y
            if INFLATE:
                # Single pixel at the ray's centre pixel (4px gap between neighbors)
                out_x = x * scale_x + scale_x // 2
                out_y = row * scale_y + scale_y // 2
                idx = (out_y * PROJ_RES_X + out_x) * 4
                pixels[idx]     = 1.0
                pixels[idx + 1] = 1.0
                pixels[idx + 2] = 1.0
            else:
                # Fill scale_x * scale_y block (nearest-neighbor upscale)
                base_x = x * scale_x
                base_y = row * scale_y
                for dy in range(scale_y):
                    for dx in range(scale_x):
                        idx = ((base_y + dy) * PROJ_RES_X + (base_x + dx)) * 4
                        pixels[idx]     = 1.0
                        pixels[idx + 1] = 1.0
                        pixels[idx + 2] = 1.0

        mode_label = "INFLATE (1px per ray)" if INFLATE else f"FILLED ({scale_x}x{scale_y} blocks)"
        if ALIGNMENT_RESULT_PATH:
            mode_label = (f"MEASURED POSE ({2*SPOT_RADIUS_PX+1}x{2*SPOT_RADIUS_PX+1} px per point, "
                          f"fit RMS {result['rms_px']:.2f} px)")
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
    # 4. INPUT IMAGE → POINT CLOUD VISUALIZATION
    # ----------------------------------------------
    if INPUT_IMAGE_PATH:
        print(f"\nLoading input image: {INPUT_IMAGE_PATH}")
        # Load image into Blender
        inp_img_name = "InputProjectorImage"
        inp_img = bpy.data.images.get(inp_img_name)
        if inp_img:
            bpy.data.images.remove(inp_img)
        try:
            inp_img = bpy.data.images.load(INPUT_IMAGE_PATH)
            inp_img.name = inp_img_name
        except Exception as e:
            inp_img = None
            print(f"  ERROR: Could not load image: {e}")

        if inp_img:
            img_w, img_h = inp_img.size[0], inp_img.size[1]
            print(f"  Image size: {img_w}x{img_h}")
            inp_pixels = list(inp_img.pixels)  # flat RGBA

            illuminated_points = []
            lit_count = 0

            for (x, y), pts in pixel_to_points.items():
                # Map ray-grid pixel (x, y) to image pixel
                # Support both native res (256x144) and 720p (1280x720)
                if img_w == RES_X and img_h == RES_Y:
                    sample_x, sample_y = x, y
                else:
                    sample_x = int((x + 0.5) * img_w / RES_X)
                    sample_y = int((y + 0.5) * img_h / RES_Y)

                sample_x = min(sample_x, img_w - 1)
                sample_y = min(sample_y, img_h - 1)

                # Blender image rows start at the bottom, like ray-grid rows
                idx = (sample_y * img_w + sample_x) * 4
                r, g, b = inp_pixels[idx], inp_pixels[idx + 1], inp_pixels[idx + 2]
                brightness = 0.299 * r + 0.587 * g + 0.114 * b

                if brightness >= INPUT_BRIGHTNESS_THRESHOLD:
                    lit_count += 1
                    illuminated_points.extend(pts)

            print(f"  Lit pixels: {lit_count} / {len(pixel_to_points)} "
                  f"({100*lit_count/max(1,len(pixel_to_points)):.1f}%)")
            print(f"  Illuminated fracture points: {len(illuminated_points)}")

            if illuminated_points:
                create_obj_from_points(INPUT_CLOUD_NAME, illuminated_points,
                                       color=(0.2, 0.8, 1.0, 1.0))
                print(f"  Created '{INPUT_CLOUD_NAME}' in scene")

            # Add as camera background
            cam.data.show_background_images = True
            for bg in list(cam.data.background_images):
                if bg.image and bg.image.name == inp_img_name:
                    cam.data.background_images.remove(bg)
            bg = cam.data.background_images.new()
            bg.image = inp_img
            bg.alpha = 0.5
            bg.display_depth = 'FRONT'

    # ----------------------------------------------
    # 5. EXPORT
    # ----------------------------------------------
    if DO_EXPORT:
        total_points = helper_coords + fracture_coords + edge_coords + fiducial_coords
        if fiducial_coords:
            create_obj_from_points("AlignFiducials", fiducial_coords, color=(1.0, 0.0, 0.8, 1.0))
        total_points += check_physical_placement(total_points, G, half_mm)
        write_dxf_points(EXPORT_PATH, total_points)

        config_path = os.path.splitext(EXPORT_PATH)[0] + "_config.txt"
        write_config_txt(config_path, scene, cam, cube)

if __name__ == "__main__":
    generate_laser_cloud()