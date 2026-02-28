import bpy
import math
import json
from mathutils import Vector
from mathutils.bvhtree import BVHTree

# ===================================================================
# PROJECTOR 3D IMAGE GENERATOR
# ===================================================================
# Generates a projector image (PNG) that displays a 3D shape inside a
# laser-etched glass block when projected.
#
# HOW IT WORKS:
#   1. Builds the same refracted ray mapping as the calibration script:
#      for every projector pixel, traces a ray through the glass
#      (with Snell's law refraction) and records the 3D point inside
#      the safe zone at a specified depth.
#
#   2. Given a target 3D mesh ("ContentShape") and an external viewer
#      camera ("ViewerCamera"), determines which projector pixels to
#      illuminate so the external viewer sees the 3D shape.
#
#   3. For each projector pixel:
#      a. Compute the refracted 3D point(s) along its ray in the glass.
#      b. Check if ANY point along the ray intersects the target mesh.
#      c. If yes: set the pixel to white (or shaded by surface normal
#         relative to the external viewer direction).
#      d. If no: pixel stays black.
#
#   4. Outputs the image as a PNG at OUTPUT_IMAGE_PATH.
#
# BLENDER VERIFICATION:
#   The script also creates verification objects in the Blender scene:
#   - "ProjectorImage_Points": the lit fracture points (what the
#     projector would illuminate in the glass).
#   - Loads the output image as a camera background on the projector
#     camera, so you can verify the mapping from the projector's view.
#   - Optionally renders from the ViewerCamera to show what the
#     external viewer would see.
#
# SETUP:
#   1. Scene must have: Camera (projector), "Cube" (glass block),
#      "ContentShape" (target 3D mesh), "ViewerCamera" (external viewer)
#   2. Set PROJECTOR_HFOV_DEG to your measured projector FOV
#   3. Run the script — generates PNG and verification objects
#
# ===================================================================

# -------------------------------------------------------------------
# CONFIGURATION
# -------------------------------------------------------------------
OUTPUT_IMAGE_PATH = "C:/Users/johnh/Downloads/ProjectorImage.png"
MAPPING_JSON_PATH = "C:/Users/johnh/Downloads/projector_mapping.json"
DO_EXPORT_IMAGE   = True

# --- Projector specs (must match calibration) ---
RES_X = 1280
RES_Y = 720

# --- Scene objects ---
CUBE_NAME       = "Cube"
CONTENT_NAME    = "ContentShape"   # Target 3D mesh to display
VIEWER_CAM_NAME = "ViewerCamera"   # External viewer camera

# --- Projector FOV (must match calibration) ---
PROJECTOR_HFOV_DEG = 37.6

# --- Glass / optics (must match calibration) ---
SAFE_ZONE_MARGIN = 0.90
IOR_OUTSIDE      = 1.00
IOR_INSIDE       = 1.50
RAY_MAX_DIST     = 100000.0

# --- Ray sampling ---
# Number of depth samples per ray to check for content intersection.
# More samples = better accuracy but slower.  1 = single midpoint test.
DEPTH_SAMPLES    = 8
PIXEL_STEP       = 1   # Trace every Nth pixel (1 = every pixel)

# --- Shading ---
# 'BINARY': pixel is white if ray hits content, black otherwise
# 'NORMAL': shade by dot(surface_normal, viewer_direction)
SHADING_MODE     = 'NORMAL'

# --- Visualization ---
POINT_RADIUS     = 0.00005
VIZ_POINT_NAME   = "ProjectorImage_Points"

# -------------------------------------------------------------------
# GEOMETRY & OPTICS (shared with calibration script)
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
    name = f"ProjGlow_{strength}_{color[0]:.2f}_{color[1]:.2f}_{color[2]:.2f}"
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
# FOV HELPERS
# -------------------------------------------------------------------
def projector_vfov_from_hfov(hfov_deg):
    return 2.0 * math.degrees(math.atan(
        math.tan(math.radians(hfov_deg / 2.0)) * RES_Y / RES_X))

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
# RAY TRACER (builds full refracted ray for a projector pixel)
# -------------------------------------------------------------------
def build_ray_tracer(scene, camera, cube):
    """Returns trace_ray(pix_x, pix_y) -> (ray_origin, ray_dir, t_enter, t_exit)
    in world space, or None.  This returns the full refracted ray segment
    through the safe zone, not just a single point.
    """
    for poly in cube.data.polygons:
        poly.use_smooth = False
    cube.data.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    bvh = BVHTree.FromObject(cube, depsgraph)

    cube_mat = cube.matrix_world
    cube_mat_inv = cube_mat.inverted()
    normal_mat = cube_mat_inv.transposed().to_3x3()

    local_bb = [Vector(corner) for corner in cube.bound_box]
    bb_min = Vector((min(v[i] for v in local_bb) for i in range(3)))
    bb_max = Vector((max(v[i] for v in local_bb) for i in range(3)))
    center = (bb_min + bb_max) * 0.5
    half = (bb_max - bb_min) * 0.5
    box_min = center - half * SAFE_ZONE_MARGIN
    box_max = center + half * SAFE_ZONE_MARGIN

    cam_origin, tl, tr, bl, br = get_camera_vectors(scene, camera)

    def trace_ray(pix_x, pix_y):
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

        # Return world-space ray origin and direction within safe zone
        p_enter_local = ray_origin_local_2 + ray_dir_local_2 * t_enter
        p_exit_local = ray_origin_local_2 + ray_dir_local_2 * t_exit
        p_enter_world = cube_mat @ p_enter_local
        p_exit_world = cube_mat @ p_exit_local

        return p_enter_world, p_exit_world, t_exit - t_enter

    return trace_ray

# -------------------------------------------------------------------
# CONTENT INTERSECTION
# -------------------------------------------------------------------
def check_content_intersection(content_bvh, content_mat_inv,
                               content_normal_mat,
                               ray_enter, ray_exit, n_samples,
                               viewer_dir=None):
    """Sample along the refracted ray segment and check for content hits.

    Returns (hit_point_world, surface_normal_world, shade) or None.
    - hit_point_world: the 3D point where the ray intersects content
    - surface_normal_world: the content surface normal at hit
    - shade: brightness value (0-1) based on viewer angle
    """
    ray_vec = ray_exit - ray_enter
    ray_len = ray_vec.length
    if ray_len < 1e-9:
        return None
    ray_dir = ray_vec.normalized()

    # Cast ray through content in local space
    origin_local = content_mat_inv @ ray_enter
    dir_local = (content_mat_inv.to_3x3() @ ray_dir).normalized()

    hit = content_bvh.ray_cast(origin_local, dir_local, ray_len)
    if not hit[0]:
        return None

    hit_local, normal_local = hit[0], hit[1]
    hit_world = content_mat_inv.inverted() @ hit_local
    normal_world = (content_normal_mat @ normal_local).normalized()

    shade = 1.0
    if viewer_dir is not None and SHADING_MODE == 'NORMAL':
        # Lambertian shading: dot(normal, to_viewer)
        shade = max(0.1, normal_world.dot(-viewer_dir.normalized()))

    return hit_world, normal_world, shade

# -------------------------------------------------------------------
# MAIN
# -------------------------------------------------------------------
def generate_projector_image():
    print("=" * 55)
    print("  PROJECTOR 3D IMAGE GENERATOR")
    print("=" * 55)

    scene = bpy.context.scene
    cam = scene.camera
    cube = bpy.data.objects.get(CUBE_NAME)
    content = bpy.data.objects.get(CONTENT_NAME)
    viewer_cam = bpy.data.objects.get(VIEWER_CAM_NAME)

    if not cam:
        return print("ERROR: No active camera (projector viewpoint).")
    if not cube:
        return print(f"ERROR: No object named '{CUBE_NAME}' in scene.")
    if not content:
        return print(f"ERROR: No object named '{CONTENT_NAME}' in scene. "
                     f"Add a mesh to display in the glass.")

    cam_hfov = get_camera_hfov(scene, cam)
    cam_vfov = get_camera_vfov(scene, cam)
    proj_vfov = projector_vfov_from_hfov(PROJECTOR_HFOV_DEG)
    print(f"Camera: HFOV={cam_hfov:.2f} deg  VFOV={cam_vfov:.2f} deg")
    print(f"Projector: {RES_X}x{RES_Y}, HFOV={PROJECTOR_HFOV_DEG:.1f} deg")
    print(f"Content: '{CONTENT_NAME}'")

    px_left, px_right, py_top, py_bottom = get_projector_pixel_bounds(
        cam_hfov, cam_vfov)
    p2c = lambda ppx, ppy: proj_to_cam(ppx, ppy, px_left, px_right,
                                        py_top, py_bottom)

    # Build ray tracer for projector->glass mapping
    trace_ray = build_ray_tracer(scene, cam, cube)

    # Build BVH for content mesh
    depsgraph = bpy.context.evaluated_depsgraph_get()
    content_bvh = BVHTree.FromObject(content, depsgraph)
    content_mat = content.matrix_world
    content_mat_inv = content_mat.inverted()
    content_normal_mat = content_mat_inv.transposed().to_3x3()

    # External viewer direction (from viewer camera if available)
    viewer_dir = None
    if viewer_cam:
        viewer_pos = viewer_cam.matrix_world.translation
        glass_center = cube.matrix_world.translation
        viewer_dir = (glass_center - viewer_pos).normalized()
        print(f"ViewerCamera: looking from {viewer_pos} "
              f"toward glass at {glass_center}")
    else:
        print("No ViewerCamera found — using binary shading")

    # --- Trace every projector pixel ---
    print(f"\nTracing {RES_X}x{RES_Y} projector pixels (step={PIXEL_STEP})...")
    pixels = [0.0, 0.0, 0.0, 1.0] * (RES_X * RES_Y)
    lit_points = []
    mapping_3d = {}  # (proj_px, proj_py) -> (x, y, z)
    hit_count = 0
    total_traced = 0

    for proj_py in range(0, RES_Y, PIXEL_STEP):
        for proj_px in range(0, RES_X, PIXEL_STEP):
            cam_x, cam_y = p2c(proj_px, proj_py)
            ray_result = trace_ray(cam_x, cam_y)
            if not ray_result:
                continue
            total_traced += 1

            ray_enter, ray_exit, seg_len = ray_result

            # Check if refracted ray intersects content
            hit = check_content_intersection(
                content_bvh, content_mat_inv, content_normal_mat,
                ray_enter, ray_exit, DEPTH_SAMPLES,
                viewer_dir)

            if hit:
                hit_world, normal_world, shade = hit
                hit_count += 1

                # Set pixel in image (flip Y for PNG bottom-left origin)
                flipped_y = (RES_Y - 1) - proj_py
                idx = (flipped_y * RES_X + proj_px) * 4
                pixels[idx]     = shade
                pixels[idx + 1] = shade
                pixels[idx + 2] = shade

                lit_points.append(hit_world)
                mapping_3d[(proj_px, proj_py)] = (
                    hit_world.x, hit_world.y, hit_world.z)

        # Progress
        if proj_py % 100 == 0:
            print(f"  Row {proj_py}/{RES_Y}: "
                  f"{hit_count} hits / {total_traced} traced")

    print(f"\nResult: {hit_count} lit pixels out of {total_traced} traced "
          f"({100*hit_count/max(1,total_traced):.1f}%)")

    # --- Save projector image ---
    img_name = "ProjectorImage"
    img = bpy.data.images.get(img_name)
    if img:
        bpy.data.images.remove(img)
    img = bpy.data.images.new(img_name, RES_X, RES_Y, alpha=False)
    img.pixels = pixels

    if DO_EXPORT_IMAGE:
        try:
            img.filepath_raw = OUTPUT_IMAGE_PATH
            img.file_format = 'PNG'
            img.save()
            print(f"Projector image saved: {OUTPUT_IMAGE_PATH}")
        except Exception as e:
            print(f"WARNING: Could not save image: {e}")
            print("  (Image still available in Blender as 'ProjectorImage')")

    # --- Create verification point cloud ---
    if lit_points:
        create_obj_from_points(VIZ_POINT_NAME, lit_points,
                               color=(0.0, 1.0, 0.5, 1.0))
        print(f"Verification point cloud: '{VIZ_POINT_NAME}' "
              f"({len(lit_points)} points)")

    # --- Add image as projector camera background ---
    cam.data.show_background_images = True
    for bg in list(cam.data.background_images):
        if bg.image and bg.image.name == img_name:
            cam.data.background_images.remove(bg)
    bg = cam.data.background_images.new()
    bg.image = img
    bg.alpha = 0.5
    bg.display_depth = 'FRONT'
    print("Projector image added to camera background")

    # --- Export 3D mapping ---
    if mapping_3d:
        mapping_out = {}
        for (ppx, ppy), (wx, wy, wz) in mapping_3d.items():
            mapping_out[f"{ppx},{ppy}"] = [wx, wy, wz]
        try:
            with open(MAPPING_JSON_PATH, 'w') as f:
                json.dump({
                    "type": "projector_3d_image",
                    "res_x": RES_X,
                    "res_y": RES_Y,
                    "projector_hfov_deg": PROJECTOR_HFOV_DEG,
                    "content_name": CONTENT_NAME,
                    "shading_mode": SHADING_MODE,
                    "hit_count": hit_count,
                    "points": mapping_out,
                }, f, indent=2)
            print(f"3D mapping exported: {MAPPING_JSON_PATH} "
                  f"({len(mapping_out)} entries)")
        except Exception as e:
            print(f"WARNING: Could not write mapping: {e}")

    # --- Verification instructions ---
    print("\n--- VERIFICATION ---")
    print("1. Switch to Camera view (Numpad 0) to see the projector")
    print("   image overlaid on the fracture point cloud.")
    print(f"2. The '{VIZ_POINT_NAME}' object shows which fracture")
    print("   points would glow when this image is projected.")
    if viewer_cam:
        print(f"3. Switch to '{VIEWER_CAM_NAME}' view to see the 3D")
        print("   shape as the external viewer would perceive it.")
    else:
        print(f"3. Add a camera named '{VIEWER_CAM_NAME}' to see the")
        print("   shape from an external perspective.")
    print("4. The projector image is available as 'ProjectorImage' in")
    print("   Blender's image editor.")
    print("\nDONE")

# --- Run from Blender scripting play button ---
generate_projector_image()
