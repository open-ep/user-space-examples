import cv2
import argparse
import numpy as np
import os

parser = argparse.ArgumentParser(description="Convert PNG to .h header for EPD")
parser.add_argument("input_png", type=str, help="Input PNG file")
parser.add_argument("-o", "--output", type=str, help="Output .h file (optional)")
parser.add_argument("-n", "--name", type=str, default="epd_image", help="Array name (default: epd_image)")
parser.add_argument("--mode", choices=["mono", "gray"], default="mono", help="Output mode: mono (1bpp) or gray (8-level)")
args = parser.parse_args()

input_file = args.input_png
output_file = "image_output.h"
array_name = args.name

EPD_WIDTH = 800
EPD_HEIGHT = 480

# 讀取影像
im = cv2.imread(input_file, cv2.IMREAD_GRAYSCALE)
if im is None:
    print("❌ Cannot read image.")
    exit()

im = cv2.resize(im, (EPD_WIDTH, EPD_HEIGHT))
im = cv2.flip(im, 1)

# 轉成 byte array
buffer = []

if args.mode == "mono":
    _, bw = cv2.threshold(im, 128, 255, cv2.THRESH_BINARY)
    bytes_per_row = EPD_WIDTH // 8
    for y in range(EPD_HEIGHT):
        for x_byte in range(bytes_per_row):
            byte = 0
            for bit in range(8):
                x = x_byte * 8 + bit
                pixel = bw[y, x]
                bit_val = 1 if pixel > 0 else 0
                byte = (byte << 1) | bit_val
            buffer.append(byte)

if args.mode == "gray":
    GRAY_LEVELS = 8                     # 3-bit grayscale
    step = 256 // GRAY_LEVELS           # 32
    im_q = (im // step)                 # 0~7 (logical levels)

    gray_map = [3, 2, 1, 0, 7, 6, 5, 4] # (0~7, 0=最黑, 7=最白)
    im_q = np.take(gray_map, im_q)      # still 0~7
    im_q = im_q << 5                    # 0x00, 0x20, ..., 0xE0
    im_q = im_q & 0xE0


    # 轉成 byte array
    buffer = im_q.flatten().tolist()


if args.mode == "mono":
    array_len = EPD_WIDTH * EPD_HEIGHT // 8
else:
    array_len = EPD_WIDTH * EPD_HEIGHT

# 輸出成 .h 檔
with open(output_file, "w") as f:
    f.write(f"// Auto-generated from {os.path.basename(input_file)}\n")
    f.write(f"#ifndef {array_name.upper()}_H\n")
    f.write(f"#define {array_name.upper()}_H\n\n")
    f.write(f"#include <stdint.h>\n\n")
    f.write(f"const uint8_t {array_name}[{array_len}] = {{\n")

    for i, byte in enumerate(buffer):
        if i % 12 == 0:
            f.write("    ")
        f.write(f"0x{byte:02X}, ")
        if (i + 1) % 12 == 0:
            f.write("\n")
    f.write("\n};\n\n")
    f.write(f"#endif // {array_name.upper()}_H\n")

print(f"✅ Generated: {output_file}")