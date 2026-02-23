import bpy
import math
import bpy_extras
from mathutils import Vector
from mathutils.bvhtree import BVHTree

# -------------------------------------------------------------------
# CONFIGURATION
# -------------------------------------------------------------------
EXPORT_PATH   = "C:/Users/johnh/Downloads/LaserOutput.dxf"
DO_EXPORT     = True  

# --- SONY MP-CL1A SPECS ---
RES_X         = 1280 // 4
RES_Y         = 720 // 4

# --- OBJECT NAMES ---
CRYSTAL_NAME     = "Cube"        
CONTENT_NAME     = "TargetShape" 
PCLOUD_NAME      = "PixelPerfectCloud"
HELPER_NAME      = "AlignmentHelpers" 
CAM_BOUNDS_NAME  = "CameraBounds"     
CAM_CIRCLES_NAME = "AlignmentCircles"

# --- SAFETY SETTINGS ---
# 0.90 means points are only valid in the inner 90% of the glass
MARGIN_PROPORTION = 0.90  

# --- HELPER SETTINGS ---
DRAW_CROSSHAIRS  =False   
HELPER_STEP      = 1      
FRAME_INSET      = 0      

# --- ALIGNMENT GUIDE SETTINGS ---
CAM_INSET_PX     = 5     # Offset for the inner guide line
CIRCLE_RADIUS_PX = 3     # Radius of filled circles at corners

# --- SAFETY MARGIN (Pixels) ---
BORDER_MARGIN = 0  

# --- OPTICS ---
IOR_OUTSIDE   = 1.00
IOR_INSIDE    = 1.50
RAY_MAX_DIST  = 1000.0
EMISSION_STRENGTH = 50.0

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
# GEOMETRY & OPTICS
# -------------------------------------------------------------------
def get_bvh_interval(bvh, matrix, origin_local, dir_local):
    hit1 = bvh.ray_cast(origin_local, dir_local, RAY_MAX_DIST)
    if not hit1[0]: return None
    
    loc1, norm1, _, _ = hit1
    dist1 = (loc1 - origin_local).length
    is_inside_start = norm1.dot(dir_local) > 0
    
    if is_inside_start:
        return 0.0, dist1
    else:
        epsilon = 0.0001
        start_exit = loc1 + (dir_local * epsilon)
        hit2 = bvh.ray_cast(start_exit, dir_local, RAY_MAX_DIST)
        if hit2[0]:
            dist2 = (hit2[0] - origin_local).length
            return dist1, dist2
        else:
            return dist1, dist1

def intersect_ray_bvh_exit(bvh, origin_local, dir_local):
    epsilon = 0.0001
    start_pt = origin_local + (dir_local * epsilon)
    hit = bvh.ray_cast(start_pt, dir_local, RAY_MAX_DIST)
    if hit[0]:
        return (hit[0] - origin_local).length
    return None

def refract(I, N, n1, n2):
    I, N = I.normalized(), N.normalized()
    eta = n1 / n2
    cosi = -max(-1.0, min(1.0, I.dot(N)))
    k = 1.0 - eta * eta * (1.0 - cosi * cosi)
    if k < 0.0: return None
    return (eta * I + (eta * cosi - math.sqrt(k)) * N).normalized()

# -------------------------------------------------------------------
# BLENDER SETUP
# -------------------------------------------------------------------
def setup_point_rendering(obj, radius=0.005, color=(1.0, 1.0, 1.0, 1.0)):
    name = f"LaserGlow_{obj.name}"
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    mat.use_nodes = True
    tree = mat.node_tree
    tree.nodes.clear()
    out = tree.nodes.new('ShaderNodeOutputMaterial')
    out.location = (300, 0)
    emit = tree.nodes.new('ShaderNodeEmission')
    emit.inputs['Strength'].default_value = EMISSION_STRENGTH
    emit.inputs['Color'].default_value = color
    tree.links.new(emit.outputs['Emission'], out.inputs['Surface'])

    if obj.data.materials: obj.data.materials[0] = mat
    else: obj.data.materials.append(mat)

    mod = obj.modifiers.get("PointViz")
    if not mod: mod = obj.modifiers.new("PointViz", 'NODES')
    tree = mod.node_group or bpy.data.node_groups.new(f"Viz_{obj.name}", 'GeometryNodeTree')
    mod.node_group = tree
    
    if hasattr(tree, 'interface'):
        tree.interface.clear()
        tree.interface.new_socket(name="Geometry", in_out='INPUT', socket_type='NodeSocketGeometry')
        tree.interface.new_socket(name="Geometry", in_out='OUTPUT', socket_type='NodeSocketGeometry')
    else:
        tree.inputs.clear(); tree.outputs.clear()
        tree.inputs.new('NodeSocketGeometry', 'Geometry')
        tree.outputs.new('NodeSocketGeometry', 'Geometry')
        
    tree.nodes.clear()
    in_n = tree.nodes.new('NodeGroupInput'); in_n.location = (-400,0)
    mtp = tree.nodes.new('GeometryNodeMeshToPoints'); mtp.location = (-200,0)
    mtp.inputs['Radius'].default_value = radius
    set_mat = tree.nodes.new('GeometryNodeSetMaterial'); set_mat.location = (0,0)
    set_mat.inputs['Material'].default_value = mat
    out_n = tree.nodes.new('NodeGroupOutput'); out_n.location = (200,0)
    tree.links.new(in_n.outputs[0], mtp.inputs['Mesh'])
    tree.links.new(mtp.outputs[0], set_mat.inputs['Geometry'])
    tree.links.new(set_mat.outputs[0], out_n.inputs[0])

def create_obj_from_points(name, points, color=(1.0, 1.0, 1.0, 1.0)):
    mesh = bpy.data.meshes.get(name) or bpy.data.meshes.new(name)
    mesh.clear_geometry()
    mesh.from_pydata(points, [], [])
    obj = bpy.data.objects.get(name)
    if not obj:
        obj = bpy.data.objects.new(name, mesh)
        bpy.context.collection.objects.link(obj)
    else: obj.data = mesh
    setup_point_rendering(obj, radius=0.00005, color=color)
    return obj

# -------------------------------------------------------------------
# MAIN LOGIC
# -------------------------------------------------------------------
def generate_laser_cloud():
    print("=" * 40)
    scene = bpy.context.scene
    crystal_obj = bpy.data.objects.get(CRYSTAL_NAME)
    content_obj = bpy.data.objects.get(CONTENT_NAME)
    cam = scene.camera
    
    if not crystal_obj or not content_obj or not cam: 
        return print("Error: Objects missing.")

    # --- SETUP BVH ---
    depsgraph = bpy.context.evaluated_depsgraph_get()
    bvh_crystal = BVHTree.FromObject(crystal_obj, depsgraph)
    bvh_content = BVHTree.FromObject(content_obj, depsgraph)
    
    c_mat, t_mat = crystal_obj.matrix_world, content_obj.matrix_world
    c_mat_inv, t_mat_inv = c_mat.inverted(), t_mat.inverted()
    c_norm = c_mat_inv.transposed().to_3x3()

    # --- BOUNDING BOX MARGIN SETUP ---
    local_bbox = [Vector(b) for b in crystal_obj.bound_box]
    l_min = Vector((
        min(v.x for v in local_bbox) * MARGIN_PROPORTION,
        min(v.y for v in local_bbox) * MARGIN_PROPORTION,
        min(v.z for v in local_bbox) * MARGIN_PROPORTION
    ))
    l_max = Vector((
        max(v.x for v in local_bbox) * MARGIN_PROPORTION,
        max(v.y for v in local_bbox) * MARGIN_PROPORTION,
        max(v.z for v in local_bbox) * MARGIN_PROPORTION
    ))
    
    def is_inside_safe_zone(pt_world):
        pt_loc = c_mat_inv @ pt_world
        return (l_min.x <= pt_loc.x <= l_max.x and
                l_min.y <= pt_loc.y <= l_max.y and
                l_min.z <= pt_loc.z <= l_max.z)

    def intersect_safe_zone_aabb(origin_loc, dir_loc):
        """
        Returns (t_enter, t_exit) for the Safe Zone Box.
        t_enter = Front/Top Hit.
        t_exit = Back/Bottom Hit.
        """
        t_min, t_max = 0.0, 100000.0
        for i in range(3):
            if abs(dir_loc[i]) < 1e-6: 
                if origin_loc[i] < l_min[i] or origin_loc[i] > l_max[i]: return None
            else:
                inv_d = 1.0 / dir_loc[i]
                t0 = (l_min[i] - origin_loc[i]) * inv_d
                t1 = (l_max[i] - origin_loc[i]) * inv_d
                if inv_d < 0.0: t0, t1 = t1, t0
                t_min = max(t_min, t0)
                t_max = min(t_max, t1)
        if t_max <= t_min: return None
        return t_min, t_max

    # --- CAMERA VECTORS ---
    frame = cam.data.view_frame(scene=scene)
    mat = cam.matrix_world
    tr, br, bl, tl = [mat @ v for v in frame]
    cam_origin = mat.translation

    vec_right_full = (tr - tl)
    vec_down_full  = (bl - tl)
    pixel_width_world = vec_right_full / RES_X
    pixel_height_vector = vec_down_full / RES_Y 
    
    grid_start_pos = tl + (vec_right_full * 0.5) + (vec_down_full * 0.5) - (pixel_width_world * RES_X * 0.5) - (pixel_height_vector * RES_Y * 0.5)

    # --- CORE RAY FUNCTION ---
    def get_refracted_ray(target_pos):
        ray_dir_world = (target_pos - cam_origin).normalized()
        r_orig_loc = c_mat_inv @ cam_origin
        r_dir_loc = (c_mat_inv.to_3x3() @ ray_dir_world).normalized()
        hit1 = bvh_crystal.ray_cast(r_orig_loc, r_dir_loc, RAY_MAX_DIST)
        if not hit1[0]: return None
        
        entry_pt = c_mat @ hit1[0]
        entry_norm = (c_norm @ hit1[1]).normalized()
        if entry_norm.dot(ray_dir_world) > -0.01: return None

        I_in = refract(ray_dir_world, entry_norm, IOR_OUTSIDE, IOR_INSIDE)
        if not I_in: return None
        return entry_pt, I_in

    # --- 1. GENERATE CONTENT ---
    print(f"Tracing '{CONTENT_NAME}'...")
    cloud_pts = []
    min_x_hit, max_x_hit = RES_X + 1, -1
    min_y_hit, max_y_hit = RES_Y + 1, -1
    
    STEP = 1
    for y in range(BORDER_MARGIN, RES_Y - BORDER_MARGIN, STEP):
        for x in range(BORDER_MARGIN, RES_X - BORDER_MARGIN, STEP):
            offset = (pixel_width_world * x) + (pixel_height_vector * y)
            target_pt = grid_start_pos + offset
            res = get_refracted_ray(target_pt)
            if res:
                entry_world, dir_world = res
                r_orig_t = t_mat_inv @ entry_world
                r_dir_t = (t_mat_inv.to_3x3() @ dir_world).normalized()
                intervals = get_bvh_interval(bvh_content, t_mat, r_orig_t, r_dir_t)
                
                if intervals:
                    target_dist = intervals[1] 
                    candidate_pt = t_mat @ (r_orig_t + r_dir_t * target_dist)
                    if is_inside_safe_zone(candidate_pt):
                        cloud_pts.append(candidate_pt)
                        if x < min_x_hit: min_x_hit = x
                        if x > max_x_hit: max_x_hit = x
                        if y < min_y_hit: min_y_hit = y
                        if y > max_y_hit: max_y_hit = y

    create_obj_from_points(PCLOUD_NAME, cloud_pts, color=(1.0, 1.0, 1.0, 1.0))

    # --- 2. GENERATE OBJECT HELPERS (Orange) ---
    def trace_line_midpoints(x_start, x_end, y_start, y_end, target_list):
        steps = int(max(abs(x_end - x_start), abs(y_end - y_start)))
        if steps < 2: steps = 2
        for i in range(0, steps + 1, HELPER_STEP):
            factor = i / steps
            cur_x = x_start + (x_end - x_start) * factor
            cur_y = y_start + (y_end - y_start) * factor
            offset = (pixel_width_world * cur_x) + (pixel_height_vector * cur_y)
            res = get_refracted_ray(grid_start_pos + offset)
            if res:
                entry_world, dir_world = res
                entry_loc = c_mat_inv @ entry_world
                dir_loc   = (c_mat_inv.to_3x3() @ dir_world).normalized()
                total_dist = intersect_ray_bvh_exit(bvh_crystal, entry_loc, dir_loc)
                if total_dist:
                    p_mid_loc = entry_loc + (dir_loc * (total_dist / 2.0))
                    target_list.append(c_mat @ p_mid_loc)

    helper_pts = []
    if max_x_hit != -1:
        min_x_hit += FRAME_INSET; max_x_hit -= FRAME_INSET
        min_y_hit += FRAME_INSET; max_y_hit -= FRAME_INSET
        trace_line_midpoints(min_x_hit, max_x_hit, min_y_hit, min_y_hit, helper_pts) 
        trace_line_midpoints(min_x_hit, max_x_hit, max_y_hit, max_y_hit, helper_pts) 
        trace_line_midpoints(min_x_hit, min_x_hit, min_y_hit, max_y_hit, helper_pts) 
        trace_line_midpoints(max_x_hit, max_x_hit, min_y_hit, max_y_hit, helper_pts) 
        if DRAW_CROSSHAIRS:
            mid_x, mid_y = (min_x_hit + max_x_hit) / 2, (min_y_hit + max_y_hit) / 2
            trace_line_midpoints(mid_x, mid_x, min_y_hit, max_y_hit, helper_pts)
            trace_line_midpoints(min_x_hit, max_x_hit, mid_y, mid_y, helper_pts)
    create_obj_from_points(HELPER_NAME, helper_pts, color=(1.0, 0.2, 0.0, 1.0))

    # --- 3. GENERATE CAMERA BOUNDS & CIRCLES (Blue/Cyan) ---
    print("Tracing Camera Bounds & Circles...")
    cam_bound_pts = []
    circle_pts = []

    def trace_safe_point(cx, cy, depth_mode, output_list):
        """Traces single point, snapped to Front or Back of Safe Zone."""
        offset = (pixel_width_world * cx) + (pixel_height_vector * cy)
        res = get_refracted_ray(grid_start_pos + offset)
        if res:
            entry_world, dir_world = res
            entry_loc = c_mat_inv @ entry_world
            dir_loc   = (c_mat_inv.to_3x3() @ dir_world).normalized()
            hits = intersect_safe_zone_aabb(entry_loc, dir_loc)
            if hits:
                # Mode: 'FRONT' -> First hit (Entry), 'BACK' -> Second hit (Exit)
                t_val = hits[0] if depth_mode == 'FRONT' else hits[1]
                output_list.append(c_mat @ (entry_loc + dir_loc * t_val))

    def trace_line_safezone(x_start, x_end, y_start, y_end, start_mode, end_mode):
        """Traces line, interpolating depth between FRONT and BACK."""
        steps = int(max(abs(x_end - x_start), abs(y_end - y_start)))
        if steps < 2: steps = 2
        for i in range(0, steps + 1, HELPER_STEP):
            factor = i / steps
            cur_x = x_start + (x_end - x_start) * factor
            cur_y = y_start + (y_end - y_start) * factor
            
            offset = (pixel_width_world * cur_x) + (pixel_height_vector * cur_y)
            res = get_refracted_ray(grid_start_pos + offset)
            if res:
                entry_world, dir_world = res
                entry_loc = c_mat_inv @ entry_world
                dir_loc   = (c_mat_inv.to_3x3() @ dir_world).normalized()
                
                # Intersect with AABB
                hits = intersect_safe_zone_aabb(entry_loc, dir_loc)
                if hits:
                    t_min, t_max = hits
                    
                    w_start = 0.0 if start_mode == 'FRONT' else 1.0
                    w_end   = 0.0 if end_mode == 'FRONT' else 1.0
                    current_w = w_start * (1.0 - factor) + w_end * factor
                    
                    t_final = t_min * (1.0 - current_w) + t_max * current_w
                    cam_bound_pts.append(c_mat @ (entry_loc + dir_loc * t_final))

    def generate_filled_circle_at(center_x, center_y, depth_mode):
        r = CIRCLE_RADIUS_PX
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx*dx + dy*dy <= r*r:
                    trace_safe_point(center_x + dx, center_y + dy, depth_mode, circle_pts)

    # --- A. Outer Frame ---
    C_MIN_X, C_MAX_X = 0, RES_X
    C_MIN_Y, C_MAX_Y = 0, RES_Y
    
    trace_line_safezone(C_MIN_X, C_MAX_X, C_MIN_Y, C_MIN_Y, 'FRONT', 'FRONT')
    trace_line_safezone(C_MIN_X, C_MAX_X, C_MAX_Y, C_MAX_Y, 'BACK', 'BACK') 
    trace_line_safezone(C_MIN_X, C_MIN_X, C_MIN_Y, C_MAX_Y, 'FRONT', 'BACK') 
    trace_line_safezone(C_MAX_X, C_MAX_X, C_MIN_Y, C_MAX_Y, 'FRONT', 'BACK') 

    generate_filled_circle_at(C_MIN_X, C_MIN_Y, 'FRONT')
    generate_filled_circle_at(C_MAX_X, C_MIN_Y, 'FRONT')
    generate_filled_circle_at(C_MIN_X, C_MAX_Y, 'BACK') 
    generate_filled_circle_at(C_MAX_X, C_MAX_Y, 'BACK') 

    # --- B. Inner Frame ---
    I_MIN_X, I_MAX_X = CAM_INSET_PX, RES_X - CAM_INSET_PX
    I_MIN_Y, I_MAX_Y = CAM_INSET_PX, RES_Y - CAM_INSET_PX
    
    trace_line_safezone(I_MIN_X, I_MAX_X, I_MIN_Y, I_MIN_Y, 'FRONT', 'FRONT')
    trace_line_safezone(I_MIN_X, I_MAX_X, I_MAX_Y, I_MAX_Y, 'BACK', 'BACK') 
    trace_line_safezone(I_MIN_X, I_MIN_X, I_MIN_Y, I_MAX_Y, 'FRONT', 'BACK') 
    trace_line_safezone(I_MAX_X, I_MAX_X, I_MIN_Y, I_MAX_Y, 'FRONT', 'BACK') 

    generate_filled_circle_at(I_MIN_X, I_MIN_Y, 'FRONT')
    generate_filled_circle_at(I_MAX_X, I_MIN_Y, 'FRONT')
    generate_filled_circle_at(I_MIN_X, I_MAX_Y, 'BACK') 
    generate_filled_circle_at(I_MAX_X, I_MAX_Y, 'BACK') 

    create_obj_from_points(CAM_BOUNDS_NAME, cam_bound_pts, color=(0.0, 0.5, 1.0, 1.0))
    create_obj_from_points(CAM_CIRCLES_NAME, circle_pts, color=(0.0, 1.0, 1.0, 1.0)) 

    if DO_EXPORT:
        write_dxf_points(EXPORT_PATH, helper_pts + cloud_pts + cam_bound_pts + circle_pts)

if __name__ == "__main__":
    generate_laser_cloud()