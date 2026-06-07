/*
 * Author: LC Wang <zaq14760@gmail.com>
 * Date: 2025-05-23
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301,
 * USA.
 */

#include <fcntl.h>
#include <gpiod.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include "png_HEX.h"
#define EPD_SPI_DEVICE "/dev/spidev0.0"
#define EPD_GPIO_CHIP "gpiochip0"

#define EPD_DC_PIN 0
#define EPD_RST_PIN 5
#define EPD_BUSY_PIN 26

#define SPI_SPEED 5000000

#define EPD_MODE_MONO 0
#define EPD_MODE_FAST 1
#define EPD_MODE_GRAY4 2

int spi_fd;
struct gpiod_chip *chip;
struct gpiod_line *epd_dc_line, *epd_rst_line, *epd_busy_line;

static const uint8_t lut_4G[153] = {
	0x40, 0x48, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x08, 0x48, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x02, 0x48, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x20, 0x48, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x0A, 0x19, 0x00, 0x03, 0x08, 0x00, 0x00,
	0x14, 0x01, 0x00, 0x14, 0x01, 0x00, 0x03,
	0x0A, 0x03, 0x00, 0x08, 0x19, 0x00, 0x00,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00,
};

static const uint8_t gray_bit24[4] = {1, 0, 1, 0};
static const uint8_t gray_bit26[4] = {1, 1, 0, 0};

void sleep_ms(unsigned int milliseconds) {
	struct timespec ts;
	ts.tv_sec = milliseconds / 1000;
	ts.tv_nsec = (milliseconds % 1000) * 1000000;
	nanosleep(&ts, NULL);
}

void sleep_us(unsigned int microseconds) {
	struct timespec ts;
	ts.tv_sec = microseconds / 1000000;
	ts.tv_nsec = (microseconds % 1000000) * 1000;
	nanosleep(&ts, NULL);
}

void spi_write(uint8_t *data, int len) {
	struct spi_ioc_transfer tr = {
	    .tx_buf = (unsigned long)data,
	    .len = len,
	    .speed_hz = SPI_SPEED,
	    .bits_per_word = 8,
	};
	ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
}

void epd_writeCommand(uint8_t command) {
	gpiod_line_set_value(epd_dc_line, 0);
	sleep_us(1);
	spi_write(&command, 1);
}

void epd_writeData(uint8_t data) {
	gpiod_line_set_value(epd_dc_line, 1);
	sleep_us(1);
	spi_write(&data, 1);
}

void epd_waitUntilIdle() {
	sleep_ms(2);
	while (true) {
		if (gpiod_line_get_value(epd_busy_line) == 0) {
			break;
		}
	}
}

void epd_HWreset() {
	sleep_ms(50);
	gpiod_line_set_value(epd_rst_line, 0);
	sleep_ms(50);
	gpiod_line_set_value(epd_rst_line, 1);
	sleep_ms(50);
}

int epd_quantize_gray4(uint8_t g) {
	if (g < 64)
		return 0;
	if (g < 128)
		return 1;
	if (g < 192)
		return 2;
	return 3;
}

void epd_init(int mode) {

	spi_fd = open(EPD_SPI_DEVICE, O_RDWR);
	if (spi_fd < 0) {
		perror("Error opening SPI device");
		exit(1);
	}

	uint8_t spi_mode = SPI_MODE_0;
	ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode);

	uint32_t speed = SPI_SPEED;
	ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

	chip = gpiod_chip_open_by_name(EPD_GPIO_CHIP);
	if (!chip) {
		perror("Error opening GPIO chip");
		exit(1);
	}

	epd_dc_line = gpiod_chip_get_line(chip, EPD_DC_PIN);
	epd_rst_line = gpiod_chip_get_line(chip, EPD_RST_PIN);
	epd_busy_line = gpiod_chip_get_line(chip, EPD_BUSY_PIN);

	if (!epd_dc_line || !epd_rst_line || !epd_busy_line) {
		perror("Error getting GPIO lines");
		gpiod_chip_close(chip);
		exit(1);
	}

	gpiod_line_request_output(epd_dc_line, "epd_dc", 0);
	gpiod_line_request_output(epd_rst_line, "epd_rst", 0);
	gpiod_line_request_input(epd_busy_line, "epd_busy");

	printf("start\n");
	epd_HWreset();
	sleep_ms(1000);
	epd_waitUntilIdle();
	printf("reset successed\n");


	epd_waitUntilIdle();
	epd_writeCommand(0x12);
	epd_waitUntilIdle();

	if (mode == EPD_MODE_GRAY4) {
		epd_writeCommand(0x74);
		epd_writeData(0x54);
		epd_writeCommand(0x7E);
		epd_writeData(0x3B);
	}

	epd_writeCommand(0x01);
	epd_writeData(0xF9);
	epd_writeData(0x00);
	epd_writeData(0x00);

	epd_writeCommand(0x11);
	epd_writeData(0x01);

	epd_writeCommand(0x44);
	epd_writeData(0x00);
	epd_writeData(0x0F);

	epd_writeCommand(0x45);
	epd_writeData(0xF9);
	epd_writeData(0x00);
	epd_writeData(0x00);
	epd_writeData(0x00);

	epd_writeCommand(0x3C);
	epd_writeData(mode == EPD_MODE_GRAY4 ? 0x00 : 0x05);

	if (mode == EPD_MODE_GRAY4) {
		epd_writeCommand(0x2C);
		epd_writeData(0x1C);
		epd_writeCommand(0x3F);
		epd_writeData(0x22);
		epd_writeCommand(0x03);
		epd_writeData(0x17);
		epd_writeCommand(0x04);
		epd_writeData(0x41);
		epd_writeData(0x00);
		epd_writeData(0x32);
	}

	epd_writeCommand(0x21);
	epd_writeData(0x00);
	epd_writeData(0x80);

	if (mode != EPD_MODE_GRAY4) {
		epd_writeCommand(0x18);
		epd_writeData(0x80);
	}

	if (mode == EPD_MODE_FAST) {
		epd_writeCommand(0x22);
		epd_writeData(0xB1);
		epd_writeCommand(0x20);
		epd_waitUntilIdle();

		epd_writeCommand(0x1A);
		epd_writeData(0x64);
		epd_writeData(0x00);

		epd_writeCommand(0x22);
		epd_writeData(0x91);
		epd_writeCommand(0x20);
		epd_waitUntilIdle();
	}

	if (mode == EPD_MODE_GRAY4) {
		epd_writeCommand(0x32);
		for (int i = 0; i < 153; i++)
			epd_writeData(lut_4G[i]);
	}

	epd_writeCommand(0x4E);
	epd_writeData(0x00);

	epd_writeCommand(0x4F);
	epd_writeData(0xF9);
	epd_writeData(0x00);

	epd_waitUntilIdle();
}

void epd_write_img(const uint8_t *img_src, uint8_t update) {

	epd_writeCommand(0x24);
	sleep_ms(10);

	for (uint x = 0; x < 250; x++) {
		for (uint y_group = 0; y_group < 16; y_group++) {
			uint8_t data = 0;
			for (int bit = 0; bit < 8; bit++) {
				uint y = y_group * 8 + bit;
				uint8_t pixel = 1;
				if (y < 122)
					pixel = img_src[y * 250 + x] >= 128 ? 1 : 0;
				data |= (pixel << (7 - bit));
			}
			epd_writeData(data);
		}
	}

	printf("write image successed, starting update image\n");

	epd_writeCommand(0x22);
	sleep_ms(10);
	epd_writeData(update);
	epd_writeCommand(0x20);
	epd_waitUntilIdle();

	printf("update image successed\n");
}

void epd_write_img_gray(const uint8_t *img_src) {

	epd_writeCommand(0x4E);
	epd_writeData(0x00);
	epd_writeCommand(0x4F);
	epd_writeData(0xF9);
	epd_writeData(0x00);

	epd_writeCommand(0x24);
	sleep_ms(10);

	for (uint x = 0; x < 250; x++) {
		for (uint y_group = 0; y_group < 16; y_group++) {
			uint8_t data = 0;
			for (int bit = 0; bit < 8; bit++) {
				uint y = y_group * 8 + bit;
				int level = 3;
				if (y < 122)
					level = epd_quantize_gray4(img_src[y * 250 + x]);
				data |= (gray_bit24[level] << (7 - bit));
			}
			epd_writeData(data);
		}
	}

	epd_writeCommand(0x4E);
	epd_writeData(0x00);
	epd_writeCommand(0x4F);
	epd_writeData(0xF9);
	epd_writeData(0x00);

	epd_writeCommand(0x26);
	sleep_ms(10);

	for (uint x = 0; x < 250; x++) {
		for (uint y_group = 0; y_group < 16; y_group++) {
			uint8_t data = 0;
			for (int bit = 0; bit < 8; bit++) {
				uint y = y_group * 8 + bit;
				int level = 3;
				if (y < 122)
					level = epd_quantize_gray4(img_src[y * 250 + x]);
				data |= (gray_bit26[level] << (7 - bit));
			}
			epd_writeData(data);
		}
	}

	printf("write image successed, starting update image\n");

	epd_writeCommand(0x22);
	sleep_ms(10);
	epd_writeData(0xC7);
	epd_writeCommand(0x20);
	epd_waitUntilIdle();

	printf("update image successed\n");
}

int main(int argc, char **argv) {
	int mode = EPD_MODE_MONO;

	if (argc > 1) {
		if (!strcmp(argv[1], "fast"))
			mode = EPD_MODE_FAST;
		else if (!strcmp(argv[1], "gray4") || !strcmp(argv[1], "gray"))
			mode = EPD_MODE_GRAY4;
	}

	epd_init(mode);
	while (true) {
		if (mode == EPD_MODE_GRAY4)
			epd_write_img_gray(&img0[0]);
		else if (mode == EPD_MODE_FAST)
			epd_write_img(&img0[0], 0xC7);
		else
			epd_write_img(&img0[0], 0xF7);
		sleep_ms(30 * 1000);
	}

	close(spi_fd);
	gpiod_line_release(epd_dc_line);
	gpiod_line_release(epd_rst_line);
	gpiod_line_release(epd_busy_line);
	gpiod_chip_close(chip);

	return 0;
}
