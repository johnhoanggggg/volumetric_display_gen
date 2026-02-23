# Microfracture - Volumetric Display via Laser-Etched Glass

## Project Overview

This project generates point clouds for fabricating volumetric 3D displays inside solid glass blocks. Micro-fracture points are laser-etched into the glass using Laser Induced Damage (LID) — a focused laser creates tiny cracks at precise 3D coordinates. These cracks scatter light and are nearly invisible under ambient conditions but glow brightly when illuminated by a projector, creating a visible 3D image suspended inside the glass.

Based on the method described in Nayar & Anand's "3D Volumetric Display using Passive Optical Scatterers" (Columbia CAVE Lab, 2006). The core tradeoff: projector 2D pixel resolution is exchanged for depth resolution in the third dimension.

## How It Works

1. A camera/projector casts rays through a glass block
2. Rays refract at the glass surface (Snell's law, IOR air=1.0, glass=1.5)
3. Fracture points are placed along the refracted ray path inside the glass
4. Points are filtered to a safe zone (inner portion of glass to avoid edge damage)
5. The point cloud is exported as DXF for the laser etching machine
6. When the physical glass is illuminated by the projector from the same angle, each pixel lights up its corresponding fracture point, recreating the 3D shape

## Files

- **`sphere_gen.py`** — Content-targeted ray tracer. Traces refracted rays through the glass into a target mesh ("TargetShape"). Places fracture points where rays intersect the content geometry. Generates alignment helpers, camera bounds, and corner circles. More complex optics pipeline with separate BVH trees for crystal and content.

- **`volemetric_display_gen.py`** — Volumetric fill ray tracer. Fills the entire inner volume of the glass with fracture points (no target mesh needed — just the glass cube). Each pixel ray gets `POINTS_PER_RAY` fracture points randomly distributed along its path through the inner safe zone. Supports aspect ratio fitting, 90-degree rotation, and axis flipping for projector alignment. Simpler pipeline — only needs Camera + Cube.

Both scripts share the same core pattern: camera pixel grid → ray tracing → refraction at glass surface → AABB filtering → DXF export. They share the same DXF writer, refraction function, point rendering setup, and Blender object creation utilities.

## Required Blender Scene Objects

### sphere_gen.py
- **Camera** — Active scene camera (projector viewpoint)
- **"Cube"** — Glass block (rays refract through this)
- **"TargetShape"** — 3D content mesh to etch into glass

### volemetric_display_gen.py
- **Camera** — Active scene camera (projector viewpoint)
- **"Cube"** — Glass block (rays refract through this)
- No target mesh needed — fills the entire inner volume

## Generated Blender Objects

### sphere_gen.py
| Object | Color | Purpose |
|---|---|---|
| `PixelPerfectCloud` | White | Content fracture points (ray-content intersections) |
| `AlignmentHelpers` | Orange | Frame outline around content bounds |
| `CameraBounds` | Blue | Crystal boundary frames (outer + inner) |
| `AlignmentCircles` | Cyan | Filled circles at frame corners for registration |

### volemetric_display_gen.py
| Object | Color | Purpose |
|---|---|---|
| `PixelPerfectCloud` | White | Volumetric fracture points (fills inner volume) |
| `AlignmentHelpers` | Orange | Border edge helpers with depth interpolation |

## Configuration

### sphere_gen.py
```
EXPORT_PATH        — Output DXF file path
RES_X, RES_Y       — Projector resolution (1280x720 downsampled 4x to 320x180)
CRYSTAL_NAME       — Blender object name for glass block ("Cube")
CONTENT_NAME       — Blender object name for target mesh ("TargetShape")
MARGIN_PROPORTION  — Safe zone ratio (0.90 = inner 90% of glass)
IOR_OUTSIDE        — Index of refraction outside glass (1.00, air)
IOR_INSIDE         — Index of refraction inside glass (1.50, glass)
BORDER_MARGIN      — Pixel margin to skip at camera edges
CAM_INSET_PX       — Inner alignment guide offset in pixels
CIRCLE_RADIUS_PX   — Corner marker circle radius in pixels
DRAW_CROSSHAIRS    — Toggle crosshair helpers through content center
```

### volemetric_display_gen.py
```
EXPORT_PATH        — Output DXF file path
RES_X, RES_Y       — Projector resolution (256x144)
CUBE_NAME          — Blender object name for glass block ("Cube")
INNER_CUBE_SCALE   — Safe zone scale factor (0.8 = inner 80% of glass)
PIXEL_STEP         — Skip every N pixels (1 = every pixel)
POINTS_PER_RAY     — Number of fracture points per ray along depth axis
POINT_RADIUS       — Blender visualization point size
HELPER_MARGIN      — Pixels from border reserved for alignment helpers
FIT_MODE           — Aspect ratio handling: 'FIT' (letterbox) or 'FILL' (crop)
ROTATE_90          — Rotate projection 90 degrees
FLIP_X, FLIP_Y     — Mirror projection axes
IOR_OUTSIDE        — Index of refraction outside glass (1.00)
IOR_INSIDE         — Index of refraction inside glass (1.50)
```

## Pipeline

### sphere_gen.py (content-targeted)
```
Camera pixel grid (320x180)
    → Ray direction per pixel
    → Ray-crystal intersection (BVH)
    → Snell's law refraction at glass surface
    → Refracted ray inside crystal
    → Ray-content intersection (BVH on TargetShape)
    → Safe zone filter (AABB, 90% margin)
    → Single fracture point per ray (at content back-face)
    → DXF export
```

### volemetric_display_gen.py (volumetric fill)
```
Camera pixel grid (256x144)
    → Pixel-to-UV with rotation/flip/aspect correction
    → Camera frustum interpolation (bilinear on view frame corners)
    → Ray-cube intersection (BVH)
    → Snell's law refraction at glass surface
    → Refracted ray intersected with inner AABB (safe zone)
    → POINTS_PER_RAY fracture points randomly distributed along ray segment
    → DXF export
```

## Code Structure

### Shared patterns (both scripts)
- `write_dxf_points(filepath, points)` — DXF point export
- `refract(I, N, n1, n2)` — Snell's law refraction with total internal reflection check
- `setup_point_rendering(obj, radius, color)` — Blender geometry nodes + emission shader
- `create_obj_from_points(name, points, color)` — Point list → Blender mesh object
- `generate_laser_cloud()` — Main entry point

### sphere_gen.py specific
- `get_bvh_interval(bvh, matrix, origin, dir)` — Entry/exit distances for ray through BVH volume
- `intersect_ray_bvh_exit(bvh, origin, dir)` — Ray exit point from BVH volume
- `get_refracted_ray(target_pos)` — Full refraction pipeline: camera→crystal entry→refracted direction
- `trace_line_midpoints(...)` — Traces helper points along pixel-space line segments at crystal midpoint depth
- `trace_line_safezone(...)` — Traces alignment points with depth interpolation (FRONT↔BACK)
- `generate_filled_circle_at(...)` — Creates filled circle of points at a pixel coordinate

### volemetric_display_gen.py specific
- `intersect_aabb(ray_origin, ray_dir, box_min, box_max)` — Pure AABB ray intersection (no BVH needed for simple box)
- `create_emission_material(strength, color)` — Standalone material factory (cached by parameters)
- `get_camera_vectors(scene, camera)` — Extracts sorted camera frustum corners (TL, TR, BL, BR)
- `get_ray_interval(pix_x, pix_y, b_min, b_max)` — Full pipeline: pixel→UV→aspect correct→frustum lerp→refract→AABB interval

## Key Differences Between Scripts

| Aspect | sphere_gen.py | volemetric_display_gen.py |
|---|---|---|
| Purpose | Etch a specific 3D shape | Fill volume with scattering points |
| Target mesh | Required ("TargetShape") | Not needed |
| Points per ray | 1 (at content surface) | Configurable (`POINTS_PER_RAY`) |
| Point placement | Deterministic (at geometry intersection) | Randomized along ray segment |
| Safe zone | `MARGIN_PROPORTION` scales bounding box | `INNER_CUBE_SCALE` scales unit cube |
| Ray intersection | BVH (crystal) + BVH (content) | BVH (crystal) + AABB (inner box) |
| Aspect ratio | Matches camera directly | FIT/FILL modes with rotation/flip |
| Alignment aids | Helpers + bounds + circles (4 objects) | Helpers only (2 objects) |
| Resolution | 320x180 (1280/4 x 720/4) | 256x144 |

## Build / Run

1. Open Blender (4.x recommended)
2. Set up scene with Camera and a cube named "Cube" (the glass block)
3. For `sphere_gen.py`: also add a mesh named "TargetShape" (the 3D content)
4. Open the desired script in Blender's scripting workspace
5. Run — generates point cloud objects in the viewport and exports DXF to `EXPORT_PATH`

## Output

- `LaserOutput.dxf` — Combined point cloud in DXF format for laser etching hardware (both scripts export to the same default path)

## Conventions

- All ray tracing uses Blender's BVHTree for mesh-ray acceleration
- Coordinate transforms between world and crystal-local spaces via `matrix_world` / its inverse / normal matrix
- Safe zone is a scaled AABB of the crystal's local bounding box
- Point rendering uses Blender geometry nodes (Mesh to Points) with emission shaders
- Scripts are self-contained — no imports beyond bpy, math, random, mathutils
- Both scripts use the same `generate_laser_cloud()` entry point name

## Optics Notes

- Refraction implements Snell's law: `eta = n1/n2`, total internal reflection when `k < 0`
- Entry normal validation rejects grazing-angle hits (sphere_gen: dot > -0.01, volemetric: dot > -0.1)
- sphere_gen places points at the content **exit** (back-face) distance
- volemetric_display_gen randomly distributes points along the full inner AABB segment
- Helper points use depth interpolation: top edges → front of glass, bottom edges → back of glass
