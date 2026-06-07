import cv2
import numpy as np
import os
import argparse

parser = argparse.ArgumentParser(
    description="Convert a single PNG image into a C const array for the 2.13\" EPD.",
    epilog="examples:\n"
           "  mono : python png2bit.py test.png\n"
           "  gray : python png2bit.py test.png --mode gray4",
    formatter_class=argparse.RawDescriptionHelpFormatter,
)
parser.add_argument("input_png", type=str, help="The name of the PNG file to convert.")
parser.add_argument("-o", "--output", type=str, help="Optional name of the output H file")
parser.add_argument("--mode", choices=["mono", "gray4"], default="mono",
                    help="Output mode: mono (black & white) or gray4 (4-level grayscale). "
                         "Both write png_HEX.h with uint8 img0[]; you choose the matching "
                         "run mode on the device.")
args = parser.parse_args()

target_png_file = args.input_png

IMG_WIDTH = 250
IMG_HEIGHT = 122

try:
    p = os.path.dirname(os.path.abspath(__file__))
except NameError:
    p = os.getcwd()

imgpath = p + '/'

if not os.path.exists(os.path.join(imgpath, target_png_file)):
    print(f"Error：Cannot find the specific file '{target_png_file}'。")
    exit()

output_c_file = args.output if args.output else "png_HEX.h"
path = os.path.join(p, output_c_file)
img_name = target_png_file
full_img_path = os.path.join(imgpath, img_name)

print(f"--- file is processing: {img_name} (mode={args.mode}) ---")

im = cv2.imread(full_img_path, cv2.IMREAD_GRAYSCALE)
if im is None:
    print(f"Error：Cannot read the picture '{img_name}'。")
    exit()

im = cv2.resize(im, (IMG_WIDTH, IMG_HEIGHT))

if args.mode == "mono":
    im = np.where(im >= 128, 255, 0).astype(np.uint8)

print(f"Resized to (HxW): {IMG_HEIGHT}x{IMG_WIDTH}")
print(f"Will generated array: const uint8_t img0[{IMG_WIDTH * IMG_HEIGHT}]")

buffer = im.flatten().tolist()

with open(path, 'w', encoding='utf-8') as f:
    f.write("#ifndef PNG_HEX_H\n")
    f.write("#define PNG_HEX_H\n\n")
    f.write("#include <stdint.h>\n\n")
    f.write(f"// Image data for: {img_name} (mode={args.mode})\n")
    f.write(f"// Dimensions (W x H): {IMG_WIDTH} x {IMG_HEIGHT}, 8bpp row-major\n")
    f.write(f"const uint8_t img0[{IMG_WIDTH * IMG_HEIGHT}] = {{\n")

    for i, byte in enumerate(buffer):
        if i % 12 == 0:
            f.write("    ")
        f.write(f"0x{byte:02X}, ")
        if (i + 1) % 12 == 0:
            f.write("\n")

    f.write("\n};\n\n")
    f.write("#endif // PNG_HEX_H\n")

print(f"\n--- Finish！Already generated file: {output_c_file} ---")
