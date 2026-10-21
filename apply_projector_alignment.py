import bpy
import math
import json
from mathutils import Vector, Matrix

# ===================================================================
# APPLY PROJECTOR ALIGNMENT
# ===================================================================
# Reads alignment_result.json exported by alignment_tool/index.html and
# moves the scene camera to where the projector REALLY is relative to
# the etched cloud.  Re-run projector_3d_image.py afterwards: its
# images are then rendered for the true pose, so the physical
# projector does not need to be nudged into the designed position.
#
# The original camera is kept as CAMERA_BACKUP_NAME ("Camera_Nominal").
# Point clouds that are already etched were designed with THAT camera.
# Make it the active camera again before regenerating a cloud (DXF);
# use the solved camera only for projector images.
#
# Sets on the active camera:
#   - pose (from the solved position / rotation in the glass frame)
#   - HFOV and lens shift (principal point) for RES_X x RES_Y
#   - scene render resolution
# Then prints the PROJECTOR_HFOV_DEG / IOR_INSIDE values to use.
# ===================================================================

# -------------------------------------------------------------------
# CONFIGURATION
# -------------------------------------------------------------------
RESULT_JSON_PATH   = "C:/Users/johnh/Downloads/alignment_result.json"
CUBE_NAME          = "Cube"
CAMERA_BACKUP_NAME = "Camera_Nominal"
APPLY_INTRINSICS   = True    # Also set HFOV + lens shift on the camera

# -------------------------------------------------------------------
# GLASS FRAME (same definition as projector_alignment_fiducials.py)
# -------------------------------------------------------------------
def get_glass_frame(cube, unit_scale):
    mw = cube.matrix_world
    _, rot, scl = mw.decompose()
    bb = [Vector(c) for c in cube.bound_box]
    bb_min = Vector((min(v[i] for v in bb) for i in range(3)))
    bb_max = Vector((max(v[i] for v in bb) for i in range(3)))
    center_world = mw @ ((bb_min + bb_max) * 0.5)
    half_units = [(bb_max[i] - bb_min[i]) * 0.5 * abs(scl[i]) for i in range(3)]
    G = Matrix.Translation(center_world) @ rot.to_matrix().to_4x4()
    return G, Vector([h / unit_scale for h in half_units])

def refract(I, N, n1, n2):
    I = I.normalized()
    N = N.normalized()
    eta = n1 / n2
    cosi = -max(-1.0, min(1.0, I.dot(N)))
    k = 1.0 - eta * eta * (1.0 - cosi * cosi)
    if k < 0.0: return None
    cost = math.sqrt(k)
    return (eta * I + (eta * cosi - cost) * N).normalized()

def ray_miss_distance(solved, rig, pixel, point_mm):
    """Distance (mm) between a fiducial and the refracted ray of the
    pixel that was measured to light it, under the solved rig."""
    W = rig["res"][0]
    f = (W / 2.0) / math.tan(math.radians(solved["hfov_deg"]) / 2.0)
    R = Matrix(solved["rotation"])
    C = Vector(solved["position_mm"])
    u, v = pixel[0] + 0.5, pixel[1] + 0.5
    d = (R @ Vector(((u - solved["cx"]) / f, (v - solved["cy"]) / f, -1.0))).normalized()
    a, s = rig["glass"]["front_axis"], rig["glass"]["front_sign"]
    h = s * rig["glass"]["half_mm"][a]          # the glass itself is not rescaled
    t = (h - C[a]) / d[a]
    E = C + d * t
    N = Vector((0.0, 0.0, 0.0))
    N[a] = s
    d2 = refract(d, N, 1.0, solved["ior"])
    P = Vector(point_mm) * solved.get("cloud_scale", 1.0)
    P[a] *= solved.get("cloud_depth_scale", 1.0)
    w = P - E
    return (w - d2 * w.dot(d2)).length

# -------------------------------------------------------------------
# MAIN
# -------------------------------------------------------------------
def apply_projector_alignment():
    print("=" * 55)
    print("  APPLY PROJECTOR ALIGNMENT")
    print("=" * 55)
    try:
        with open(RESULT_JSON_PATH) as fh:
            result = json.load(fh)
    except Exception as e:
        return print(f"ERROR: Could not read {RESULT_JSON_PATH}: {e}")
    if result.get("type") != "projector_alignment_result":
        return print("ERROR: Not an alignment_result.json from alignment_tool/index.html")

    scene = bpy.context.scene
    cam = scene.camera
    cube = bpy.data.objects.get(CUBE_NAME)
    if not cam or not cube:
        return print(f"ERROR: Need active Camera + object named '{CUBE_NAME}'.")

    design = result["design"]
    rig = design["rig"]
    solved = result["solved"]
    unit_scale = design.get("block_unit_scale", 0.001)
    G, half_mm = get_glass_frame(cube, unit_scale)
    for i in range(3):
        if abs(half_mm[i] - rig["glass"]["half_mm"][i]) > 0.05:
            print(f"WARNING: Cube size differs from the fiducial design on axis {'XYZ'[i]} "
                  f"({2*half_mm[i]:.2f} vs {2*rig['glass']['half_mm'][i]:.2f} mm)")

    # Keep the nominal (design) camera
    if not bpy.data.objects.get(CAMERA_BACKUP_NAME):
        backup = cam.copy()
        backup.data = cam.data.copy()
        backup.name = CAMERA_BACKUP_NAME
        for coll in cam.users_collection:
            coll.objects.link(backup)
        print(f"Original camera saved as '{CAMERA_BACKUP_NAME}'")

    # Pose: glass frame (mm) → world
    R = Matrix(solved["rotation"])
    loc = G @ (Vector(solved["position_mm"]) * unit_scale)
    rot_world = G.to_3x3() @ R
    old_loc = cam.matrix_world.translation.copy()
    cam.matrix_world = Matrix.Translation(loc) @ rot_world.to_4x4()
    print(f"Camera moved by {(loc - old_loc).length / unit_scale:.3f} mm")

    W, H = rig["res"]
    if APPLY_INTRINSICS:
        scene.render.resolution_x = W
        scene.render.resolution_y = H
        scene.render.pixel_aspect_x = 1.0
        scene.render.pixel_aspect_y = 1.0
        cam.data.type = 'PERSP'
        cam.data.sensor_fit = 'HORIZONTAL'
        cam.data.angle = math.radians(solved["hfov_deg"])
        # Blender shift is in units of the larger image side; a positive
        # shift moves the frame, so the optical axis lands left / below centre.
        cam.data.shift_x = (W / 2.0 - solved["cx"]) / max(W, H)
        cam.data.shift_y = (H / 2.0 - solved["cy"]) / max(W, H)
        print(f"Camera HFOV {solved['hfov_deg']:.3f} deg, shift "
              f"({cam.data.shift_x:+.4f}, {cam.data.shift_y:+.4f})")

    # Check: each measured pixel's ray should pass through its fiducial
    fids = {f["id"]: f for f in design["fiducials"]}
    worst = 0.0
    for r in result["readings"]:
        miss = ray_miss_distance(solved, rig, r["measured_pixel"], fids[r["id"]]["point_mm"])
        worst = max(worst, miss)
    print(f"Check: measured rays pass within {worst*1000:.0f} um of their fiducials "
          f"(fit RMS {result['rms_px']:.2f} px)")

    print("\nDiagnosis:")
    for line in result.get("diagnosis", []):
        print("  " + line)

    print("\nNEXT:")
    print(f"  In projector_3d_image.py set PROJECTOR_HFOV_DEG = {solved['hfov_deg']:.3f}")
    print(f"                              IOR_INSIDE         = {solved['ior']:.4f}")
    print("  then re-run it to render images for the measured projector pose.")
    print(f"  Use '{CAMERA_BACKUP_NAME}' (the design pose) when regenerating clouds.")
    print("DONE")

if __name__ == "__main__":
    apply_projector_alignment()
