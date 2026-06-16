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
#define EPD_GPIO_CHIP "gpiochip15"

#define EPD_DC_PIN 5
#define EPD_RST_PIN 6
#define EPD_BUSY_PIN 26

#define SPI_SPEED 5000000

#define EPD_MODE_MONO 0
#define EPD_MODE_FAST 1
#define EPD_MODE_GRAY4 2

/*
 * Landscape logical frame for the partial/clock demos: DISP_W is the gate axis
 * (0x45/0x4F), DISP_H the RAM-X byte axis (0x44/0x4E), only 122 rows visible.
 * Pixel (x,y) -> buf[x * DISP_STRIDE + (y >> 3)], bit 0x80 >> (y & 7); sending
 * the buffer linearly matches the photo path's orientation.
 */
#define DISP_W 250
#define DISP_H 128
#define DISP_STRIDE (DISP_H / 8)		/* 16 bytes per gate line */
#define DISP_BUF_SIZE (DISP_W * DISP_STRIDE)
#define DISP_GATE_MAX (DISP_W - 1)	/* 249, max gate address */

int spi_fd;
struct gpiod_chip *chip;
struct gpiod_line_request *epd_dc_line, *epd_rst_line, *epd_busy_line;

static const unsigned int EPD_DC_OFFSET   = EPD_DC_PIN;
static const unsigned int EPD_RST_OFFSET  = EPD_RST_PIN;
static const unsigned int EPD_BUSY_OFFSET = EPD_BUSY_PIN;

/* libgpiod v2: request one line as output, return its request handle. */
static struct gpiod_line_request *
gpiod_request_output(struct gpiod_chip *c, unsigned int offset,
		     const char *consumer, int value)
{
	struct gpiod_line_settings *settings;
	struct gpiod_line_config *line_cfg;
	struct gpiod_request_config *req_cfg;
	struct gpiod_line_request *request = NULL;

	settings = gpiod_line_settings_new();
	if (!settings)
		return NULL;
	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_output_value(settings,
		value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);

	line_cfg = gpiod_line_config_new();
	if (!line_cfg) {
		gpiod_line_settings_free(settings);
		return NULL;
	}
	if (gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings) < 0)
		goto out;

	req_cfg = gpiod_request_config_new();
	if (req_cfg)
		gpiod_request_config_set_consumer(req_cfg, consumer);

	request = gpiod_chip_request_lines(c, req_cfg, line_cfg);

	if (req_cfg)
		gpiod_request_config_free(req_cfg);
out:
	gpiod_line_config_free(line_cfg);
	gpiod_line_settings_free(settings);
	return request;
}

/* libgpiod v2: request one line as input, return its request handle. */
static struct gpiod_line_request *
gpiod_request_input(struct gpiod_chip *c, unsigned int offset,
		    const char *consumer)
{
	struct gpiod_line_settings *settings;
	struct gpiod_line_config *line_cfg;
	struct gpiod_request_config *req_cfg;
	struct gpiod_line_request *request = NULL;

	settings = gpiod_line_settings_new();
	if (!settings)
		return NULL;
	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);

	line_cfg = gpiod_line_config_new();
	if (!line_cfg) {
		gpiod_line_settings_free(settings);
		return NULL;
	}
	if (gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings) < 0)
		goto out;

	req_cfg = gpiod_request_config_new();
	if (req_cfg)
		gpiod_request_config_set_consumer(req_cfg, consumer);

	request = gpiod_chip_request_lines(c, req_cfg, line_cfg);

	if (req_cfg)
		gpiod_request_config_free(req_cfg);
out:
	gpiod_line_config_free(line_cfg);
	gpiod_line_settings_free(settings);
	return request;
}

static inline void
gpiod_set_value(struct gpiod_line_request *req, unsigned int offset, int value)
{
	gpiod_line_request_set_value(req, offset,
		value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
}

static inline int
gpiod_get_value(struct gpiod_line_request *req, unsigned int offset)
{
	return gpiod_line_request_get_value(req, offset) == GPIOD_LINE_VALUE_ACTIVE
		? 1 : 0;
}

#define epd_dc_set(v)    gpiod_set_value(epd_dc_line,   EPD_DC_OFFSET,   (v))
#define epd_rst_set(v)   gpiod_set_value(epd_rst_line,  EPD_RST_OFFSET,  (v))
#define epd_busy_get()   gpiod_get_value(epd_busy_line, EPD_BUSY_OFFSET)

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
	epd_dc_set(0);
	sleep_us(1);
	spi_write(&command, 1);
}

void epd_writeData(uint8_t data) {
	epd_dc_set(1);
	sleep_us(1);
	spi_write(&data, 1);
}

/* Bulk data write: DC=1 once, then push the buffer in <=4096-byte transfers. */
void epd_writeData_bulk(const uint8_t *data, int len) {
	epd_dc_set(1);
	sleep_us(1);
	while (len > 0) {
		int chunk = len > 4096 ? 4096 : len;
		spi_write((uint8_t *)data, chunk);
		data += chunk;
		len -= chunk;
	}
}

void epd_waitUntilIdle() {
	sleep_ms(2);
	while (true) {
		if (epd_busy_get() == 0) {
			break;
		}
	}
}

void epd_HWreset() {
	sleep_ms(50);
	epd_rst_set(0);
	sleep_ms(50);
	epd_rst_set(1);
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

/*
 * Mono controller register setup, no fd reopen.  Mirrors the mono path of
 * epd_init(); used to re-init after a HW reset in the partial/clock refresh.
 */
void epd_reg_init(void) {
	epd_waitUntilIdle();
	epd_writeCommand(0x12);		/* SW reset */
	epd_waitUntilIdle();

	epd_writeCommand(0x01);		/* driver output: 250 gate lines */
	epd_writeData(0xF9);
	epd_writeData(0x00);
	epd_writeData(0x00);

	epd_writeCommand(0x11);		/* data entry: Y decrement, X increment */
	epd_writeData(0x01);

	epd_writeCommand(0x44);		/* RAM X window: 0..15 (16 bytes) */
	epd_writeData(0x00);
	epd_writeData(0x0F);

	epd_writeCommand(0x45);		/* RAM Y window: 249..0 */
	epd_writeData(0xF9);
	epd_writeData(0x00);
	epd_writeData(0x00);
	epd_writeData(0x00);

	epd_writeCommand(0x3C);		/* border waveform (full update) */
	epd_writeData(0x05);

	epd_writeCommand(0x21);		/* display update control 1 */
	epd_writeData(0x00);
	epd_writeData(0x80);

	epd_writeCommand(0x18);		/* internal temperature sensor */
	epd_writeData(0x80);

	epd_writeCommand(0x4E);		/* RAM X counter = 0 */
	epd_writeData(0x00);

	epd_writeCommand(0x4F);		/* RAM Y counter = 249 */
	epd_writeData(0xF9);
	epd_writeData(0x00);

	epd_waitUntilIdle();
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

	chip = gpiod_chip_open("/dev/" EPD_GPIO_CHIP);
	if (!chip) {
		perror("Error opening GPIO chip");
		exit(1);
	}

	epd_dc_line   = gpiod_request_output(chip, EPD_DC_OFFSET,   "epd_dc", 0);
	epd_rst_line  = gpiod_request_output(chip, EPD_RST_OFFSET,  "epd_rst", 0);
	epd_busy_line = gpiod_request_input(chip,  EPD_BUSY_OFFSET, "epd_busy");

	if (!epd_dc_line || !epd_rst_line || !epd_busy_line) {
		perror("Error requesting GPIO lines");
		gpiod_chip_close(chip);
		exit(1);
	}

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

/* xb* are RAM-X byte addresses (0x44/0x4E); g_* are 9-bit gate lines, counting
 * down (0x45/0x4F, data-entry mode 0x01). */
static void epd_set_window(int xb_start, int xb_end, int g_start, int g_end) {
	epd_writeCommand(0x44);
	epd_writeData(xb_start & 0xFF);
	epd_writeData(xb_end & 0xFF);

	epd_writeCommand(0x45);
	epd_writeData(g_start & 0xFF);
	epd_writeData((g_start >> 8) & 0xFF);
	epd_writeData(g_end & 0xFF);
	epd_writeData((g_end >> 8) & 0xFF);
}

static void epd_set_cursor(int xb, int g) {
	epd_writeCommand(0x4E);
	epd_writeData(xb & 0xFF);

	epd_writeCommand(0x4F);
	epd_writeData(g & 0xFF);
	epd_writeData((g >> 8) & 0xFF);
}

static void epd_set_full_window(void) {
	epd_set_window(0x00, DISP_STRIDE - 1, DISP_GATE_MAX, 0x00);
}

static void epd_set_full_cursor(void) {
	epd_set_cursor(0x00, DISP_GATE_MAX);
}

/* Seed both RAM banks (0x24 new, 0x26 old/diff) and full-refresh (0xF7). */
void epd_set_base_map(const uint8_t *buf) {
	epd_set_full_window();

	epd_set_full_cursor();
	epd_writeCommand(0x24);
	epd_writeData_bulk(buf, DISP_BUF_SIZE);

	epd_set_full_cursor();
	epd_writeCommand(0x26);
	epd_writeData_bulk(buf, DISP_BUF_SIZE);

	epd_writeCommand(0x22);
	epd_writeData(0xF7);
	epd_writeCommand(0x20);
	epd_waitUntilIdle();
}

/*
 * Partial-refresh waveform (the panel's built-in OTP Mode-2 waveform leaves
 * static areas grey).  153 LUT bytes + 6 voltage bytes.
 */
static const uint8_t WF_PARTIAL[159] = {
	0x0, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x80, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x40, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x14, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x0, 0x0, 0x0,	/* end of 153 LUT bytes */
	0x22, 0x17, 0x41, 0x0, 0x32, 0x36,			/* EOPT, VGH, VSH1, VSH2, VSL, VCOM */
};

/* Upload the partial waveform + voltages (0x37 enables RAM ping-pong). */
void epd_load_partial_lut(void) {
	epd_writeCommand(0x32);		/* LUT (153 bytes) */
	for (int i = 0; i < 153; i++)
		epd_writeData(WF_PARTIAL[i]);
	epd_waitUntilIdle();

	epd_writeCommand(0x3F);		/* EOPT */
	epd_writeData(WF_PARTIAL[153]);

	epd_writeCommand(0x03);		/* gate voltage */
	epd_writeData(WF_PARTIAL[154]);

	epd_writeCommand(0x04);		/* source voltage */
	epd_writeData(WF_PARTIAL[155]);
	epd_writeData(WF_PARTIAL[156]);
	epd_writeData(WF_PARTIAL[157]);

	epd_writeCommand(0x2C);		/* VCOM */
	epd_writeData(WF_PARTIAL[158]);

	epd_writeCommand(0x37);
	epd_writeData(0x00);
	epd_writeData(0x00);
	epd_writeData(0x00);
	epd_writeData(0x00);
	epd_writeData(0x00);
	epd_writeData(0x40);
	epd_writeData(0x00);
	epd_writeData(0x00);
	epd_writeData(0x00);
	epd_writeData(0x00);

	epd_writeCommand(0x3C);		/* partial border */
	epd_writeData(0x80);
}

/* TurnOnDisplay_Partial: display with Mode 2 (0x22=0x0F). */
void epd_partial_update(void) {
	epd_writeCommand(0x22);
	epd_writeData(0x0F);
	epd_writeCommand(0x20);
	epd_waitUntilIdle();
}

/* Re-assert orientation/source registers after the reset pulse (no 0x12, so
 * the RAM banks survive for the ping-pong differential). */
static void epd_partial_regs(void) {
	epd_writeCommand(0x01);		/* 250 gate lines */
	epd_writeData(0xF9);
	epd_writeData(0x00);
	epd_writeData(0x00);

	epd_writeCommand(0x11);		/* data entry: Y decrement, X increment */
	epd_writeData(0x01);

	epd_writeCommand(0x21);		/* display update control 1 */
	epd_writeData(0x00);
	epd_writeData(0x80);

	epd_writeCommand(0x18);		/* internal temperature sensor */
	epd_writeData(0x80);
}

/*
 * Full-frame partial refresh.  Write the whole frame to the new bank (0x24)
 * ONLY: hardware ping-pong keeps the last frame in the old bank and diffs
 * against it, so only changed pixels are driven.  Writing 0x26 here would
 * fight the ping-pong and scramble the image.
 */
void epd_display_partial_full(const uint8_t *image) {
	epd_rst_set(0);	/* reset pulse (RAM is preserved) */
	sleep_ms(2);
	epd_rst_set(1);
	sleep_ms(2);

	epd_partial_regs();
	epd_load_partial_lut();

	epd_writeCommand(0x22);			/* warm up: enable clock + analog */
	epd_writeData(0xC0);
	epd_writeCommand(0x20);
	epd_waitUntilIdle();

	epd_set_full_window();
	epd_set_full_cursor();
	epd_writeCommand(0x24);
	epd_writeData_bulk(image, DISP_BUF_SIZE);

	epd_partial_update();
}

static uint8_t demo_img_a[DISP_BUF_SIZE];
static uint8_t demo_img_b[DISP_BUF_SIZE];
static uint8_t clk_bg[DISP_BUF_SIZE];
static uint8_t clk_cur[DISP_BUF_SIZE];

/* 7-segment digit bits: a=0 b=1 c=2 d=3 e=4 f=5 g=6 */
static const uint8_t seg7[10] = {
	0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F,
};

/* set one black pixel (RAM bit = 0) */
static inline void fb_set(uint8_t *buf, int x, int y) {
	if (x < 0 || x >= DISP_W || y < 0 || y >= DISP_H)
		return;
	buf[x * DISP_STRIDE + (y >> 3)] &= ~(0x80 >> (y & 7));
}

static void fb_clear(uint8_t *buf) {
	memset(buf, 0xFF, DISP_BUF_SIZE);
}

/* filled black rectangle [x0,x1) x [y0,y1) */
static void fb_rect(uint8_t *buf, int x0, int y0, int x1, int y1) {
	for (int x = x0; x < x1; x++)
		for (int y = y0; y < y1; y++)
			fb_set(buf, x, y);
}

/* big 7-segment digit at (px,py); cell w x h, thickness t */
static void fb_digit(uint8_t *buf, int px, int py, int w, int h, int t, int d) {
	const int mid = (h - t) / 2;
	uint8_t s = seg7[d];

	if (s & 0x01) fb_rect(buf, px + t,     py,         px + w - t, py + t);       /* a */
	if (s & 0x02) fb_rect(buf, px + w - t, py + t,     px + w,     py + h / 2);   /* b */
	if (s & 0x04) fb_rect(buf, px + w - t, py + h / 2, px + w,     py + h - t);   /* c */
	if (s & 0x08) fb_rect(buf, px + t,     py + h - t, px + w - t, py + h);       /* d */
	if (s & 0x10) fb_rect(buf, px,         py + h / 2, px + t,     py + h - t);   /* e */
	if (s & 0x20) fb_rect(buf, px,         py + t,     px + t,     py + h / 2);   /* f */
	if (s & 0x40) fb_rect(buf, px + t,     py + mid,   px + w - t, py + mid + t); /* g */
}

static void build_demo_images(void) {
	const int w = 60, h = 100, t = 16;
	const int px = (DISP_W - w) / 2, py = (122 - h) / 2;

	fb_clear(demo_img_a);
	fb_digit(demo_img_a, px, py, w, h, t, 1);

	fb_clear(demo_img_b);
	fb_digit(demo_img_b, px, py, w, h, t, 2);
}

/* Demo: switch between two images via full-frame partial refresh. */
void epd_display_partial_demo(void) {
	build_demo_images();
	fb_clear(clk_bg);			/* white base seeds both RAM banks */

	epd_set_base_map(clk_bg);
	sleep_ms(2000);

	for (int round = 0; round < 5; round++) {
		epd_display_partial_full(demo_img_b);
		sleep_ms(2000);

		epd_display_partial_full(demo_img_a);
		sleep_ms(2000);
	}
}

#define CLK_DIGIT_W   24
#define CLK_DIGIT_H   60
#define CLK_SEG_T      6	/* segment thickness */
#define CLK_GAP        6	/* gap between glyphs */
#define CLK_COLON_W   12

/* clock region in landscape coords (well within the 122 visible rows) */
#define CLK_W        224
#define CLK_H         72
#define CLK_X         13
#define CLK_Y         28
/* glyph origin: HH:MM:SS spans 210 dots, centred in the region */
#define CLK_OX       (CLK_X + (CLK_W - 210) / 2)
#define CLK_OY       (CLK_Y + (CLK_H - CLK_DIGIT_H) / 2)

static void clk_colon(uint8_t *buf, int px, int oy) {
	const int h = CLK_DIGIT_H, t = CLK_SEG_T;
	const int cx = px + (CLK_COLON_W - t) / 2;

	fb_rect(buf, cx, oy + h / 3 - t / 2,     cx + t, oy + h / 3 + t / 2);
	fb_rect(buf, cx, oy + 2 * h / 3 - t / 2, cx + t, oy + 2 * h / 3 + t / 2);
}

/* draw HH:MM:SS into a buffer at origin (ox,oy); caller preps the canvas */
static void clk_paint(uint8_t *buf, int ox, int oy, int hh, int mm, int ss) {
	const int dw = CLK_DIGIT_W, dh = CLK_DIGIT_H, t = CLK_SEG_T;
	int px = ox;

	fb_digit(buf, px, oy, dw, dh, t, hh / 10); px += dw + CLK_GAP;
	fb_digit(buf, px, oy, dw, dh, t, hh % 10); px += dw + CLK_GAP;
	clk_colon(buf, px, oy);                    px += CLK_COLON_W + CLK_GAP;
	fb_digit(buf, px, oy, dw, dh, t, mm / 10); px += dw + CLK_GAP;
	fb_digit(buf, px, oy, dw, dh, t, mm % 10); px += dw + CLK_GAP;
	clk_colon(buf, px, oy);                    px += CLK_COLON_W + CLK_GAP;
	fb_digit(buf, px, oy, dw, dh, t, ss / 10); px += dw + CLK_GAP;
	fb_digit(buf, px, oy, dw, dh, t, ss % 10);
}

/*
 * Digital clock: white base map once, then redraw the full HH:MM:SS and push a
 * full-frame partial each second; ping-pong drives only the changed digits.
 */
void epd_display_clock(void) {
	int count = 0;

	fb_clear(clk_bg);
	epd_set_base_map(clk_bg);

	while (true) {
		time_t now = time(NULL);
		struct tm *t = localtime(&now);

		memcpy(clk_cur, clk_bg, DISP_BUF_SIZE);
		clk_paint(clk_cur, CLK_OX, CLK_OY,
			  t->tm_hour, t->tm_min, t->tm_sec);
		epd_display_partial_full(clk_cur);

		/* periodically full-refresh to clear accumulated ghosting */
		if (++count >= 300) {
			count = 0;
			epd_HWreset();
			epd_reg_init();
			epd_set_base_map(clk_bg);
		}

		/* re-align to the next wall-clock second so it ticks once/sec */
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		sleep_ms(1000 - ts.tv_nsec / 1000000);
	}
}

int main(int argc, char **argv) {
	const char *arg = argc > 1 ? argv[1] : "";
	int mode = EPD_MODE_MONO;

	if (!strcmp(arg, "fast"))
		mode = EPD_MODE_FAST;
	else if (!strcmp(arg, "gray4") || !strcmp(arg, "gray"))
		mode = EPD_MODE_GRAY4;

	epd_init(mode);

	if (!strcmp(arg, "partial")) {
		epd_display_partial_demo();	/* two-image partial swap */
	} else if (!strcmp(arg, "clock")) {
		epd_display_clock();		/* HH:MM:SS, partial each second */
	} else {
		while (true) {
			if (mode == EPD_MODE_GRAY4)
				epd_write_img_gray(&img0[0]);
			else if (mode == EPD_MODE_FAST)
				epd_write_img(&img0[0], 0xC7);
			else
				epd_write_img(&img0[0], 0xF7);
			sleep_ms(30 * 1000);
		}
	}

	close(spi_fd);
	gpiod_line_request_release(epd_dc_line);
	gpiod_line_request_release(epd_rst_line);
	gpiod_line_request_release(epd_busy_line);
	gpiod_chip_close(chip);

	return 0;
}
