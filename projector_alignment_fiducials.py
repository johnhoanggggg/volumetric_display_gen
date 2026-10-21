import bpy
import math
import json
from mathutils import Vector, Matrix

# ===================================================================
# PROJECTOR ALIGNMENT FIDUCIALS
# ===================================================================
# Etches a small constellation of single-point fiducials whose exact
# projector pixel is known by design.  After etching, the interactive
# tool (alignment_tool/index.html) finds which pixel ACTUALLY lights
# each one, and a solver turns those readings into the projector's
# true pose relative to the etched cloud.
#
# WHY THIS IS ROBUST:
#   The etching shop's centring error (~1 mm), the projector stand and
#   small IOR / FOV errors all look the same to the projector: the
#   cloud sits somewhere slightly different from where Blender thinks.
#   Measuring a handful of fiducials pins that down (6 pose DOF, plus
#   optional FOV / principal point / IOR / scale), and the solved pose
#   is fed back into Blender so images are rendered for where the
#   cloud REALLY is.  No more nudging the projector by trial and error.
#
# WHAT IS GENERATED:
#   AlignFiducials     (magenta) — N_COLS x N_ROWS single points spread
#                      across the usable projector area, alternating
#                      front / back depth (checkerboard) so the fit can
#                      separate translation from rotation (parallax).
#   AlignAnchors       (blue)    — 8 points at the safe-zone corners.
#                      The cloud's bounding box becomes symmetric about
#                      the block centre, so a shop that "centres the
#                      model" puts it exactly where you designed it.
#                      Also measurable with calipers + loupe.
#   AlignGlyph         (yellow)  — An "F" near the front top-left (as
#                      seen from the projector).  Under a flashlight it
#                      reveals mirrored / rotated etching at a glance.
#
#   DXF (EXPORT_PATH)  — all points above, world coordinates.
#   JSON (FIDUCIAL_JSON_PATH) — rig + fiducial design, load this into
#                      alignment_tool/index.html.
#   PNG (CHECK_IMAGE_PATH) — every fiducial's design pixel lit.  If the
#                      projector were perfectly placed, all fiducials
#                      would glow at once.
#
# PROJECTOR MODEL:
#   The active scene camera supplies the projector POSE only.  Its
#   intrinsics come from PROJECTOR_HFOV_DEG / RES_X / RES_Y here, so
#   the Blender camera lens does not matter.  Pixel (k) is the ray
#   through k + 0.5; pixel rows count UP from the bottom.
#   Rays must enter through the block face facing the projector.
#
# PRODUCTION BLOCKS: set FIDUCIAL_JSON_PATH in volemetric_display_gen.py
#   to the JSON written here.  It etches these fiducials + glyph into the
#   display cloud and keeps cloud rays off their pixels, so every block
#   carries its own alignment reference.
#
# WORKFLOW: see ALIGNMENT.md
# ===================================================================

# -------------------------------------------------------------------
# CONFIGURATION
# -------------------------------------------------------------------
EXPORT_PATH        = "C:/Users/johnh/Downloads/AlignmentFiducials.dxf"
FIDUCIAL_JSON_PATH = "C:/Users/johnh/Downloads/alignment_fiducials.json"
CHECK_IMAGE_PATH   = "C:/Users/johnh/Downloads/AlignmentCheck.png"
DO_EXPORT          = True

# --- Projector (Nebra AnyBeam 720p) ---
RES_X              = 1280
RES_Y              = 720
PROJECTOR_HFOV_DEG = 38
PRINCIPAL_X        = None    # Continuous px; None = RES_X / 2
PRINCIPAL_Y        = None    # Continuous px (from bottom); None = RES_Y / 2

# --- Scene / glass ---
CUBE_NAME          = "Cube"
BLOCK_X_MM         = None    # Set all three to resize the cube, None = as-is
BLOCK_Y_MM         = None
BLOCK_Z_MM         = None
BLOCK_UNIT_SCALE   = 0.001   # mm → Blender units (0.001 = scene in metres)
SAFE_ZONE_MARGIN   = 0.90    # Inner 90% of the block on each axis
IOR_INSIDE         = 1.517   # K9 / BK7 ≈ 1.515 (red) – 1.525 (blue)

# --- Fiducial layout ---
N_COLS             = 5
N_ROWS             = 3
DEPTH_FRONT        = 0.10    # Depth factor along the safe-zone segment
DEPTH_BACK         = 0.90
INSET_PX           = 40      # Keep sites this far inside the usable area
KEEP_OUT_PX        = 20      # Other content should avoid ±this many px
POINTS_PER_FIDUCIAL = 1      # >1 stacks points along the design ray
STACK_SPACING_MM   = 0.15

# --- Extras ---
ADD_ANCHORS        = True
ADD_GLYPH          = True
GLYPH_SIZE_MM      = 4.0
GLYPH_SPACING_MM   = 0.25
GLYPH_INSET_MM     = 1.0

# --- Visualisation ---
POINT_RADIUS       = 0.00005
FIDUCIAL_NAME      = "AlignFiducials"
ANCHOR_NAME        = "AlignAnchors"
GLYPH_NAME         = "AlignGlyph"

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
        if abs(ray_dir[i]) < 1e-12:
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
    name = f"AlignGlow_{strength}_{color[0]:.2f}_{color[1]:.2f}_{color[2]:.2f}"
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
# GLASS FRAME (block centre, block axes, millimetres)
# -------------------------------------------------------------------
def get_glass_frame(cube):
    """Returns (G, half_mm): G maps glass-frame Blender units → world."""
    mw = cube.matrix_world
    _, rot, scl = mw.decompose()
    bb = [Vector(c) for c in cube.bound_box]
    bb_min = Vector((min(v[i] for v in bb) for i in range(3)))
    bb_max = Vector((max(v[i] for v in bb) for i in range(3)))
    center_world = mw @ ((bb_min + bb_max) * 0.5)
    half_units = [(bb_max[i] - bb_min[i]) * 0.5 * abs(scl[i]) for i in range(3)]
    G = Matrix.Translation(center_world) @ rot.to_matrix().to_4x4()
    half_mm = Vector([h / BLOCK_UNIT_SCALE for h in half_units])
    return G, half_mm

def build_projector(cam, G, half_mm):
    """Projector model in the glass frame (mm).  Same maths as
    alignment_tool/alignment_core.js makeModel()."""
    cam_loc, cam_rot, _ = cam.matrix_world.decompose()
    G_rot_inv = G.to_3x3().inverted()
    C = (G.inverted() @ cam_loc) / BLOCK_UNIT_SCALE
    R = G_rot_inv @ cam_rot.to_matrix()
    f = (RES_X / 2.0) / math.tan(math.radians(PROJECTOR_HFOV_DEG) / 2.0)
    cx = RES_X / 2.0 if PRINCIPAL_X is None else PRINCIPAL_X
    cy = RES_Y / 2.0 if PRINCIPAL_Y is None else PRINCIPAL_Y

    # Front face = the face the projector is outside of and looks into
    view = R @ Vector((0.0, 0.0, -1.0))
    best = None
    for a in range(3):
        for s in (1, -1):
            if C[a] * s <= half_mm[a]:
                continue
            facing = -view[a] * s
            if best is None or facing > best[0]:
                best = (facing, a, s)
    if best is None or best[0] <= 0:
        return None
    return {"C": C, "R": R, "f": f, "cx": cx, "cy": cy,
            "axis": best[1], "sign": best[2], "half": half_mm}

def pixel_to_point(proj, u, v, depth_frac):
    """Continuous projector pixel → glass-frame point (mm) at depth_frac
    of the refracted ray's segment through the safe zone, or None."""
    C, R, a, s, half = proj["C"], proj["R"], proj["axis"], proj["sign"], proj["half"]
    d = (R @ Vector(((u - proj["cx"]) / proj["f"],
                     (v - proj["cy"]) / proj["f"], -1.0)).normalized())
    if abs(d[a]) < 1e-12:
        return None
    t = (s * half[a] - C[a]) / d[a]
    if t <= 0:
        return None
    E = C + d * t
    for b in range(3):
        if b != a and abs(E[b]) > half[b]:
            return None
    N = Vector((0.0, 0.0, 0.0))
    N[a] = s
    if N.dot(d) > -0.1:
        return None
    d2 = refract(d, N, 1.0, IOR_INSIDE)
    if not d2:
        return None
    hi = Vector([h * SAFE_ZONE_MARGIN for h in half])
    seg = intersect_aabb(E, d2, -hi, hi)
    if not seg or seg[1] <= seg[0]:
        return None
    return E + d2 * (seg[0] + (seg[1] - seg[0]) * depth_frac)

# -------------------------------------------------------------------
# LAYOUT
# -------------------------------------------------------------------
def find_usable_rect(proj, step=8):
    """Bounding rect of pixels whose rays cross the safe zone at both
    fiducial depths."""
    xs, ys = [], []
    for y in range(0, RES_Y, step):
        for x in range(0, RES_X, step):
            if (pixel_to_point(proj, x + 0.5, y + 0.5, DEPTH_FRONT) and
                    pixel_to_point(proj, x + 0.5, y + 0.5, DEPTH_BACK)):
                xs.append(x)
                ys.append(y)
    if not xs:
        return None
    return min(xs), max(xs), min(ys), max(ys)

def generate_fiducials(proj):
    rect = find_usable_rect(proj)
    if not rect:
        print("ERROR: No projector pixel reaches the safe zone through the front face.")
        return []
    x0, x1, y0, y1 = rect
    print(f"  Usable projector area: x {x0}..{x1}, y {y0}..{y1} (rows from bottom)")
    x0 += INSET_PX; x1 -= INSET_PX
    y0 += INSET_PX; y1 -= INSET_PX
    if x1 <= x0 or y1 <= y0:
        print("ERROR: Usable area smaller than 2 x INSET_PX — reduce INSET_PX.")
        return []

    cx_mid, cy_mid = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    fiducials = []
    for j in range(N_ROWS):
        for i in range(N_COLS):
            fu = x0 + (x1 - x0) * (i / (N_COLS - 1) if N_COLS > 1 else 0.5)
            fv = y0 + (y1 - y0) * (j / (N_ROWS - 1) if N_ROWS > 1 else 0.5)
            depth = DEPTH_FRONT if (i + j) % 2 == 0 else DEPTH_BACK
            U, V = int(round(fu)), int(round(fv))
            # Nudge towards the centre if this exact pixel misses
            p = None
            for k in range(20):
                p = pixel_to_point(proj, U + 0.5, V + 0.5, depth)
                if p:
                    break
                U = int(round(U + (cx_mid - U) * 0.1))
                V = int(round(V + (cy_mid - V) * 0.1))
            if not p:
                print(f"  WARNING: site ({i},{j}) unusable, skipped")
                continue
            fiducials.append({
                "label": f"{'F' if depth == DEPTH_FRONT else 'B'}{j}{i}",
                "pixel": [U, V],
                "depth_frac": depth,
                "point_mm": [p.x, p.y, p.z],
            })

    # Centre-out order: the solver gets well-conditioned readings first
    fiducials.sort(key=lambda f: (f["pixel"][0] - cx_mid) ** 2 + (f["pixel"][1] - cy_mid) ** 2)
    for idx, f in enumerate(fiducials):
        f["id"] = idx
    return fiducials

def fiducial_points(proj, fid):
    """Etched points for one fiducial (stacked along the design ray)."""
    U, V = fid["pixel"]
    base = Vector(fid["point_mm"])
    if POINTS_PER_FIDUCIAL <= 1:
        return [base]
    deeper = pixel_to_point(proj, U + 0.5, V + 0.5, min(1.0, fid["depth_frac"] + 0.01))
    direction = (deeper - base).normalized() if deeper else Vector((0, 0, 0))
    offs = [(k - (POINTS_PER_FIDUCIAL - 1) / 2.0) * STACK_SPACING_MM
            for k in range(POINTS_PER_FIDUCIAL)]
    return [base + direction * o for o in offs]

def generate_anchors(half_mm):
    hi = [h * SAFE_ZONE_MARGIN for h in half_mm]
    return [Vector((sx * hi[0], sy * hi[1], sz * hi[2]))
            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]

def generate_glyph(proj):
    """An 'F' readable from the projector side, near the front top-left."""
    a, s, R, half = proj["axis"], proj["sign"], proj["R"], proj["half"]
    lateral = [b for b in range(3) if b != a]
    right_g = R.col[0]
    up_g = R.col[1]
    ax_r = max(lateral, key=lambda b: abs(right_g[b]))
    ax_u = [b for b in lateral if b != ax_r][0]
    sg_r = 1 if right_g[ax_r] >= 0 else -1
    sg_u = 1 if up_g[ax_u] >= 0 else -1
    hi = [h * SAFE_ZONE_MARGIN for h in half]

    origin_r = -hi[ax_r] + GLYPH_INSET_MM            # left edge (projector view)
    origin_u = hi[ax_u] - GLYPH_INSET_MM - GLYPH_SIZE_MM  # bottom of glyph
    depth = s * (hi[a] - GLYPH_INSET_MM)

    S = GLYPH_SIZE_MM
    strokes = [((0, 0), (0, S)), ((0, S), (0.6 * S, S)), ((0, 0.5 * S), (0.45 * S, 0.5 * S))]
    pts = []
    for (r0, u0), (r1, u1) in strokes:
        n = max(1, int(round(math.hypot(r1 - r0, u1 - u0) / GLYPH_SPACING_MM)))
        for k in range(n + 1):
            r = r0 + (r1 - r0) * k / n
            u = u0 + (u1 - u0) * k / n
            p = Vector((0.0, 0.0, 0.0))
            p[a] = depth
            p[ax_r] = sg_r * (origin_r + r)
            p[ax_u] = sg_u * (origin_u + u)
            pts.append(p)
    # Strokes share endpoints; de-duplicate
    uniq = {}
    for p in pts:
        uniq[(round(p.x, 4), round(p.y, 4), round(p.z, 4))] = p
    return list(uniq.values())

def save_check_image(fiducials):
    """PNG with every fiducial's design pixel lit (rows from bottom, as
    Blender stores images — so no flip is needed)."""
    name = "AlignmentCheck"
    img = bpy.data.images.get(name)
    if img:
        bpy.data.images.remove(img)
    img = bpy.data.images.new(name, RES_X, RES_Y, alpha=False)
    pixels = [0.0, 0.0, 0.0, 1.0] * (RES_X * RES_Y)
    for f in fiducials:
        U, V = f["pixel"]
        idx = (V * RES_X + U) * 4
        pixels[idx:idx + 3] = [0.0, 1.0, 0.0]
    img.pixels = pixels
    try:
        img.filepath_raw = CHECK_IMAGE_PATH
        img.file_format = 'PNG'
        img.save()
        print(f"Check image saved: {CHECK_IMAGE_PATH}")
    except Exception as e:
        print(f"WARNING: Could not save check image: {e}")

# -------------------------------------------------------------------
# MAIN
# -------------------------------------------------------------------
def generate_alignment_fiducials():
    print("=" * 55)
    print("  PROJECTOR ALIGNMENT FIDUCIALS")
    print("=" * 55)
    scene = bpy.context.scene
    cube = bpy.data.objects.get(CUBE_NAME)
    cam = scene.camera
    if not cam or not cube:
        return print(f"ERROR: Need active Camera + object named '{CUBE_NAME}'.")

    if BLOCK_X_MM is not None and BLOCK_Y_MM is not None and BLOCK_Z_MM is not None:
        s = BLOCK_UNIT_SCALE
        cube.dimensions = Vector((BLOCK_X_MM * s, BLOCK_Y_MM * s, BLOCK_Z_MM * s))
        bpy.context.view_layer.update()

    G, half_mm = get_glass_frame(cube)
    print(f"Glass block: {2*half_mm[0]:.1f} x {2*half_mm[1]:.1f} x {2*half_mm[2]:.1f} mm, "
          f"IOR {IOR_INSIDE}")
    proj = build_projector(cam, G, half_mm)
    if not proj:
        return print("ERROR: Camera must be outside the block, looking at one face.")
    axis_name = "XYZ"[proj["axis"]]
    print(f"Projector: {RES_X}x{RES_Y}, HFOV {PROJECTOR_HFOV_DEG} deg, "
          f"{proj['C'].length:.1f} mm from block centre, "
          f"entering face {'+' if proj['sign'] > 0 else '-'}{axis_name}")

    to_world = lambda p: G @ (p * BLOCK_UNIT_SCALE)
    all_points = []

    fiducials = generate_fiducials(proj)
    fid_world = []
    for f in fiducials:
        fid_world.extend(to_world(p) for p in fiducial_points(proj, f))
    if fid_world:
        create_obj_from_points(FIDUCIAL_NAME, fid_world, color=(1.0, 0.0, 0.8, 1.0))
        all_points.extend(fid_world)
    print(f"  {len(fiducials)} fiducials:")
    for f in fiducials:
        print(f"    #{f['id']:2d} {f['label']}  pixel ({f['pixel'][0]:4d}, {f['pixel'][1]:3d})  "
              f"depth {f['depth_frac']:.2f}")

    anchors = generate_anchors(half_mm) if ADD_ANCHORS else []
    if anchors:
        create_obj_from_points(ANCHOR_NAME, [to_world(p) for p in anchors],
                               color=(0.0, 0.5, 1.0, 1.0))
        all_points.extend(to_world(p) for p in anchors)
        print(f"  8 anchors at the safe-zone corners "
              f"({(1 - SAFE_ZONE_MARGIN) * half_mm[0]:.2f} mm in from each X face, etc.)")

    glyph = []
    if ADD_GLYPH:
        glyph = generate_glyph(proj)
        create_obj_from_points(GLYPH_NAME, [to_world(p) for p in glyph],
                               color=(1.0, 1.0, 0.0, 1.0))
        all_points.extend(to_world(p) for p in glyph)
        print(f"  Orientation glyph 'F': {len(glyph)} points")

    if DO_EXPORT and all_points:
        write_dxf_points(EXPORT_PATH, all_points)
        R = proj["R"]
        data = {
            "type": "projector_alignment_fiducials",
            "version": 1,
            "rig": {
                "res": [RES_X, RES_Y],
                "hfov_deg": PROJECTOR_HFOV_DEG,
                "cx": proj["cx"],
                "cy": proj["cy"],
                "ior": IOR_INSIDE,
                "projector": {
                    "position": list(proj["C"]),
                    "rotation": [list(R[i]) for i in range(3)],
                },
                "glass": {
                    "half_mm": list(half_mm),
                    "safe_margin": SAFE_ZONE_MARGIN,
                    "front_axis": proj["axis"],
                    "front_sign": proj["sign"],
                },
            },
            "fiducials": fiducials,
            "keep_out_px": KEEP_OUT_PX,
            "anchors_mm": [list(p) for p in anchors],
            "glyph_mm": [list(p) for p in glyph],
            "block_unit_scale": BLOCK_UNIT_SCALE,
        }
        try:
            with open(FIDUCIAL_JSON_PATH, 'w') as fh:
                json.dump(data, fh, indent=2)
            print(f"Fiducial design saved: {FIDUCIAL_JSON_PATH}")
        except Exception as e:
            print(f"ERROR: Could not write fiducial JSON: {e}")
        save_check_image(fiducials)

    print("\nNEXT: etch the DXF, then open alignment_tool/index.html, load the JSON,")
    print("      and follow ALIGNMENT.md.")
    print("DONE")

if __name__ == "__main__":
    generate_alignment_fiducials()
