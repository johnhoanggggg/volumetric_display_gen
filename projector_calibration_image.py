import bpy

# -------------------------------------------------------------------
# CONFIGURATION
# -------------------------------------------------------------------
IMAGE_PATH    = "C:/Users/johnh/Downloads/CalibrationBorder.png"
PROJ_RES_X    = 1280
PROJ_RES_Y    = 720

# -------------------------------------------------------------------
# GENERATE CALIBRATION IMAGE
# -------------------------------------------------------------------
def generate_calibration_image():
    """Black 720p image with a 1px white border for projector alignment."""
    print("Generating calibration border image...")

    pixels = [0.0, 0.0, 0.0, 1.0] * (PROJ_RES_X * PROJ_RES_Y)

    for x in range(PROJ_RES_X):
        for y in range(PROJ_RES_Y):
            if x == 0 or x == PROJ_RES_X - 1 or y == 0 or y == PROJ_RES_Y - 1:
                idx = (y * PROJ_RES_X + x) * 4
                pixels[idx]     = 1.0
                pixels[idx + 1] = 1.0
                pixels[idx + 2] = 1.0

    img_name = "CalibrationBorder"
    img = bpy.data.images.get(img_name)
    if img:
        bpy.data.images.remove(img)
    img = bpy.data.images.new(img_name, PROJ_RES_X, PROJ_RES_Y, alpha=False)
    img.pixels = pixels

    try:
        img.filepath_raw = IMAGE_PATH
        img.file_format = 'PNG'
        img.save()
        print(f"  Saved: {IMAGE_PATH}")
    except Exception as e:
        print(f"  WARNING: Could not save image: {e}")
        print(f"  (Image still available in Blender as '{img_name}')")

    print(f"  Resolution: {PROJ_RES_X}x{PROJ_RES_Y}")
    print("  White 1px border on black background")

if __name__ == "__main__":
    generate_calibration_image()
