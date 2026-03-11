from PIL import Image

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

    img = Image.new("RGB", (PROJ_RES_X, PROJ_RES_Y), (0, 0, 0))
    pixels = img.load()

    for x in range(PROJ_RES_X):
        pixels[x, 0] = (255, 255, 255)
        pixels[x, PROJ_RES_Y - 1] = (255, 255, 255)
    for y in range(PROJ_RES_Y):
        pixels[0, y] = (255, 255, 255)
        pixels[PROJ_RES_X - 1, y] = (255, 255, 255)

    img.save(IMAGE_PATH)
    print(f"  Saved: {IMAGE_PATH}")
    print(f"  Resolution: {PROJ_RES_X}x{PROJ_RES_Y}")
    print("  White 1px border on black background")

if __name__ == "__main__":
    generate_calibration_image()
