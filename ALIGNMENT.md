# Aligning a Real Projector to a Real Etched Block

The physical cloud in the glass never sits exactly where Blender put it. The shop centres the model by its own rules, the etching machine has its own tolerances, and the block is only nominally 50 × 50 × 80 mm. The physical projector never sits exactly where the Blender camera is either. Each of these errors is small, but at a typical distance **1 mm equals about 14–17 projector pixels**, while `volemetric_display_gen.py` puts one point every 5 px. Nudging the projector cannot fix a 6-DOF error by eye.

This workflow etches **fiducials into every block**, measures which projector pixel *actually* lights each one, solves where the projector really is relative to the etched cloud, and **renders the image for that measured pose**. The projector only has to be roughly placed and then held still.

## 1. Simulated result

The simulation (`volemetric_display_gen.py`, 50 × 50 × 80 mm block, projector 110 mm away) used realistic physical errors:

* the shop etched the cloud 0.7 / −0.5 / 0.8 mm off and rotated 0.3°
* the projector stand is off by a few mm and 0.2–0.3°
* the glass is K9 (n = 1.517), not the 1.50 the scripts assume

The fiducials then light up as far as **19 px** from their design pixels. Projecting the generated image and checking which etched points light up gives:

| Image | Intended points lit | Lit points that are wrong |
|---|---|---|
| Original code, perfect placement, no physical error at all | 0 % | ~100 % |
| Bugs fixed, nominal image, physical errors above | 29 % | 73 % |
| Bugs fixed, image rendered for the pose measured from 15 fiducials | **100 %** | 3 % (3×3 spot overlap) |

### Error budget

Projector-pixel error over the safe zone (`alignment_tool/alignment_core.js`):

| Physical error | Raw (median / worst px) | After pose solve (px) |
|---|---|---|
| Cloud etched 1 mm off sideways | 13.8 / 16.7 | 0.2 / 0.3 |
| Cloud etched 1 mm deeper | 1.2 / 2.8 | 0.2 / 0.4 |
| Projector 1 mm sideways | 13.8 / 16.7 | 0.2 / 0.3 |
| Projector 2 mm closer / further | 3.5 / 8.6 | 0.2 / 0.5 |
| Projector yaw 0.2° | 6.6 / 6.8 | 0.3 / 0.3 |
| Projector roll 0.5° | 2.1 / 4.3 | 0.1 / 0.2 |
| Lens offset (principal point) 10 px | 10.0 / 10.0 | 0.2 / 0.4 |
| HFOV off by 0.3° | 2.1 / 4.2 | 0.2 / 0.7 |
| Shop rescaled the model by 1 % | 2.4 / 5.9 | 0.1 / 0.2 |
| Glass IOR 1.500 vs 1.517 | 0.5 / 1.2 | 0.3 / 1.2 |
| Red vs blue dispersion | 0.3 / 0.7 | 0.3 / 0.7 |

* **Position dominates; IOR barely matters.** A cloud offset and a projector offset look identical to the projector, so one pose solve absorbs both. You never need to know which one happened.
* **Measuring the IOR more precisely is low value.** K9 is ≈1.517 in green and the scripts use 1.50, which costs about 1 px at the back corners. The tool can fit the IOR (±0.02) if you are curious.
* **Calibrate and display in green.** With full RGB, the red and blue copies of a voxel land up to 0.7 px apart.

## 2. Bugs that made alignment impossible

These were fixed in `volemetric_display_gen.py`, `projector_3d_image.py`, `projector_fov_calibration.py` and `volumetric_display_fullsize.py`.

* **Projector images were vertically mirrored.** Blender stores image rows from the bottom, and the ray grid also counts rows from the bottom. The image writers flipped the rows anyway, as did the input-image reader, which flipped them back consistently. Projected, each pixel therefore lit the ray mirrored about the horizontal centre. That gives the right 2D silhouette for a centred, symmetric shape, but at the wrong depths, so it looks like "almost aligned" no matter where the projector goes. *If you turned on a projector flip or ceiling mode to compensate, turn it off.*
* **Rays drifted up to half a block from their pixels.** `proj_to_cam()` spread pixels over `RES − 1` instead of centring them at `k + 0.5`, so many rays fell into a neighbouring 5×5 image block. Fixing only the mirror bug still lit just 25 % of the right points; fixing both lights 100 %. `INFLATE` mode now also lights the ray's centre pixel instead of the block corner.
* **The DXF bounding box was off-centre.** In the test scene its centre was 1.2–1.9 mm off the block centre in depth. A shop that centres the bounding box etches the whole cloud that far off. `SYMMETRIC_BBOX` (on by default) adds 6 points to make the bounding box symmetric. The script also prints the offset and **warns about points within 2 mm of a glass face**. The alignment border is traced in the full block, so some border points sit on the surface. A shop may refuse those points, clip them, or shrink the whole model to fit.

## 3. Physical setup checklist

1. Send **exactly 1280×720** from the laptop (the AnyBeam's native resolution) with display scaling at 100 %. A 1080p signal is resampled and breaks 1:1 pixel mapping.
2. Turn **keystone off**, use normal front/table mode, and no digital zoom.
3. Mount the projector **rigidly** on a base plate. Register the block against two stops (an L-bracket) so it goes back to the same place every time.
4. Work in a dark room, in green.

**Ask the etching shop** not to scale the model and to place the DXF origin at the block centre. When the block arrives, check the "F" glyph with a flashlight. It should read correctly from the projector side, near the front top-left. If it is mirrored or rotated, the shop swapped axes, which calibration cannot fix.

## 4. Procedure for each block

1. **Fiducials.** In the same scene and with the same `PROJECTOR_HFOV_DEG` as the display, run `projector_alignment_fiducials.py`. It writes `alignment_fiducials.json`: 15 single points at known projector pixels on a 5×3 grid, alternating front and back depth so the fit can separate a sideways shift from a rotation. It also writes a calibration-only DXF (fiducials, corner anchors, glyph) and `AlignmentCheck.png`.
2. **Cloud.** In `volemetric_display_gen.py`, set `FIDUCIAL_JSON_PATH` to that JSON and run it. The DXF now contains the fiducials and glyph. Cloud rays within ±`KEEP_OUT_PX` (20) of a fiducial's pixel are dropped, so while you measure a fiducial, only it lights up. Read the printed bounding-box and surface warnings. **Etch this DXF.**
3. **Rough placement.** Project `AlignmentCheck.png` and place the projector until some fiducials glow. Within ~±30 px is enough. Then lock it down.
4. **Measure.** Open `alignment_tool/index.html` in Chrome or Edge on the laptop (no install) and load the JSON. Click **Open projector window**, drag the window to the projector screen and press **F**; the status line must read *1:1 pixel mapping OK*. Then, for each fiducial:
   * The cursor starts at the predicted pixel. Use `←`/`→` to move a vertical line until the fiducial glows. Use `.` for a wider band and `Shift` + arrow keys for coarse steps.
   * Press `Space` for a horizontal line and use `↑`/`↓`.
   * Press `Space` again for a single dot to confirm, then `Enter`.
   * The fit updates after every reading. From the third reading on, predictions usually land within 1–2 px.
   * Press `A` to light every fiducial at its predicted pixel. They should all glow together.
5. **Read the diagnosis.** It shows how far the projector is, in mm and degrees, from where it should be relative to *this block's* cloud, with ± uncertainty. It also gives the physical move that would fix it. Readings off by more than 2 px are flagged; usually the wrong point lit, so re-measure. Tick *Principal point* if residuals stay above ~1 px, because many projectors have a lens offset.
6. **Render for the measured pose.** Click **Export result**. In `volemetric_display_gen.py`, also set `ALIGNMENT_RESULT_PATH` to that file and re-run it. Keep every other setting and `RANDOM_SEED` unchanged so the identical cloud is regenerated; set `DO_EXPORT = False` if you like. `ProjectorImage.png` now lights, for each selected etched point, the pixel the *measured* projector maps onto it, as a (2·`SPOT_RADIUS_PX`+1)² spot.
   * For `projector_3d_image.py`, run `apply_projector_alignment.py` instead. It moves the scene camera to the measured pose and keeps the original as `Camera_Nominal`.
7. **Re-check.** After anything is bumped, press `A` in the tool. If all fiducials still glow, nothing changed. If not, re-measure 3–4 of them; a fit that starts close needs only a few readings.

## 5. Choosing the resolution

* After the pose solve, the residual error is under ~1 px. A point every 3–5 px with a ~1 px laser spot is therefore safe.
* Dropping to a lower resolution only makes an *unmeasured* block work if the point spacing exceeds the physical error, which is ~15 px per mm here. Measuring is far cheaper than throwing away resolution.
* Use both depths. Fiducials all on one plane cannot tell a sideways shift from a yaw or pitch; the front/back checkerboard provides that parallax.

## 6. Further: per-point camera lookup table

If residuals stay above ~1 px even with the principal point free, the remaining error is unmodelled. Possible causes are MEMS scan non-linearity, etch-machine distortion, or a non-flat face. The next step is measuring every point instead of fitting a model:

1. Put a phone on a tripod and project Gray-code column and row patterns (11 + 10 frames).
2. Decode which projector pixel lights each etched point.
3. Render through that table.

The camera needs no calibration; it only has to see which points light.
