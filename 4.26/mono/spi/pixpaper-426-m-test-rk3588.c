/*
 * Author: Wig Cheng <onlywig@gmail.com>
 * Date: 2025-07-30
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
#include "image_output.h"

#define EPD_SPI_DEVICE "/dev/spidev4.0"
#define EPD_GPIO_CHIP "gpiochip4"

#define EPD_DC_PIN 26
#define EPD_RST_PIN 27
#define EPD_BUSY_PIN 29

#define SPI_SPEED 5000000

#define EPD_WIDTH   800
#define EPD_HEIGHT  480
#define EPD_BUF_SIZE (EPD_WIDTH * EPD_HEIGHT / 8)

const unsigned char lut_color1_update[] = {
	// 20 bytes
	0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// LUT0: BB:     VS 0 ~ 11
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// LUT1: WB:     VS 0 ~ 11
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// LUT2: BW:     VS 0 ~ 11
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// LUT3: WW:     VS 0 ~ 11
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// LUT4: VCOM:   VS 0 ~ 11

	0x02, 0x01, 0x03, 0x01, 0x00,	// TP0[A] TP0[B] SR[0AB] TP0[C]
		// TP0[D] SR[0CD] RP0
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]

	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x09,
	0x32,
	0x32,
	0x26,
	0x0F,
	0x04,
};

const unsigned char lut_color2_update[] = {
	// 20 bytes
	0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// LUT0: BB:     VS 0 ~ 11
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// LUT1: WB:     VS 0 ~ 11
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// LUT2: BW:     VS 0 ~ 11
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// LUT3: WW:     VS 0 ~ 11
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// LUT4: VCOM:   VS 0 ~ 11

	0x01, 0x01, 0x01, 0x01, 0x00,	// TP0[A] TP0[B] SR[0AB] TP0[C]
		// TP0[D] SR[0CD] RP0
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]

	0x00,0x00,0x00,0x00, 0x00,	// TP1[A] TP1[B] SR[0AB] TP1[C]
	0x09,
	0x32,
	0x32,
	0x1E,
	0x0F,
	0x04,
};

static const uint8_t pass_mask[] = {
	0x80,	// pass 0: MSB
	0x40,	// pass 1
	0x20,	// pass 2
};

int spi_fd;
int pass = 0;
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

void epd_init() {

	spi_fd = open(EPD_SPI_DEVICE, O_RDWR);
	if (spi_fd < 0) {
		perror("Error opening SPI device");
		exit(1);
	}

	uint8_t mode = SPI_MODE_0;
	ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);

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

	epd_writeCommand(0x18);
	epd_writeData(0x80);

	epd_writeCommand(0x0C);
	epd_writeData(0xAE);
	epd_writeData(0xC7);
	epd_writeData(0xC3);
	epd_writeData(0xC0);
	epd_writeData(0x80);

	epd_writeCommand(0x01);
	epd_writeData((EPD_HEIGHT - 1) & 0xFF);
	epd_writeData((EPD_HEIGHT - 1) >> 8);
	epd_writeData(0x02);

	epd_writeCommand(0x3C);
	epd_writeData(0x01);

	epd_writeCommand(0x44);
	epd_writeData(0x00);
	epd_writeData(0x00);
	epd_writeData((EPD_WIDTH - 1) & 0xFF);
	epd_writeData((EPD_WIDTH - 1) >> 8);

	epd_writeCommand(0x45);
	epd_writeData(0x00);
	epd_writeData(0x00);
	epd_writeData((EPD_HEIGHT - 1) & 0xFF);
	epd_writeData((EPD_HEIGHT - 1) >> 8);

	epd_writeCommand(0x4E);
	epd_writeData(0x00);
	epd_writeData(0x00);
	epd_writeCommand(0x4F);
	epd_writeData(0x00);
	epd_writeData(0x00);
}

void epd_display_mono_image() {
	epd_writeCommand(0x24);

	for (int i = 0; i < EPD_BUF_SIZE; i++) {
		epd_writeData(epd_image[i]);
	}

	epd_writeCommand(0x21);
	epd_writeData(0x40);
	epd_writeData(0x00);

	epd_writeCommand(0x22);
	epd_writeData(0xF7);
	epd_writeCommand(0x20);
	epd_waitUntilIdle();
}

void epd_kick_pass_0(void)
{
	epd_writeCommand(0x22);
	epd_writeData(0xF4);
	epd_writeCommand(0x20);
	epd_waitUntilIdle();
}

void epd_kick_pass_1(void)
{
	epd_writeCommand(0x21);
	epd_writeData(0x40);
	epd_writeData(0x00);

	epd_writeCommand(0x22);
	epd_writeData(0xCF);
	epd_writeCommand(0x20);
	epd_waitUntilIdle();
}

void epd_kick_pass_2(void)
{
	epd_writeCommand(0x21);
	epd_writeData(0x40);
	epd_writeData(0x00);

	epd_writeCommand(0x22);
	epd_writeData(0xCF);
	epd_writeCommand(0x20);
	epd_waitUntilIdle();
}

void epd_reset_ram_counters(void)
{
	epd_writeCommand(0x4E);
	epd_writeData(0x00);
	epd_writeData(0x00);

	epd_writeCommand(0x4F);
	epd_writeData(0x00);
	epd_writeData(0x00);
}

void epd_full_clear(void)
{
	epd_reset_ram_counters();
	epd_writeCommand(0x24);
	for (int y = 0; y < EPD_HEIGHT; y++)
		for (int x = 0; x < (EPD_WIDTH / 8); x++)
			epd_writeData(0xFF);
	epd_kick_pass_0();
	epd_waitUntilIdle();
}

void epd_upload_lut(const uint8_t *lut)
{
	if(lut == NULL)
		return;

	epd_writeCommand(0x03);
	epd_writeData(lut[105]);

	epd_writeCommand(0x04);
	epd_writeData(lut[106]);
	epd_writeData(lut[107]);
	epd_writeData(lut[108]);

	epd_writeCommand(0x32);
	for (int count = 0; count < (EPD_WIDTH / 8); count++) {
		epd_writeData(lut[count]);
	}

	epd_waitUntilIdle();
	epd_writeCommand(0x22);
	epd_writeData(0xC0);
	epd_writeCommand(0x20);
	epd_waitUntilIdle();
}

void epd_grayscale_pass(const uint8_t *lut)
{
	epd_upload_lut(lut);
	epd_reset_ram_counters();

	epd_writeCommand(0x24);
	uint8_t mask = pass_mask[pass];

	for (int y = 0; y < EPD_HEIGHT; y++) {
		for (int x_byte = 0; x_byte < (EPD_WIDTH / 8); x_byte++) {
			uint8_t out = 0;
			for (int bit = 0; bit < 8; bit++) {
				int x = x_byte * 8 + bit;
				uint8_t gray = epd_image[y * EPD_WIDTH + x];
				out <<= 1;
				if (gray & mask)
					out |= 1;
			}
			epd_writeData(out);
		}
	}
}

void epd_display_grayscaled_image(void)
{
	epd_full_clear();

	pass = 0;
	epd_grayscale_pass(NULL);
	epd_kick_pass_0();

	pass = 1;
	epd_grayscale_pass(lut_color1_update);
	epd_kick_pass_1();

	pass = 2;
	epd_grayscale_pass(lut_color2_update);
	epd_kick_pass_2();
}

void epd_partial_update(void)
{
	epd_writeCommand(0x22);
	epd_writeData(0xFF);
	epd_writeCommand(0x20);
	epd_waitUntilIdle();
}

void epd_set_base_map(const uint8_t *data)
{
	epd_reset_ram_counters();

	epd_writeCommand(0x24);
	for (int i = 0; i < EPD_BUF_SIZE; i++)
		epd_writeData(data[i]);

	epd_writeCommand(0x26);
	for (int i = 0; i < EPD_BUF_SIZE; i++)
		epd_writeData(data[i]);

	epd_writeCommand(0x22);
	epd_writeData(0xF7);
	epd_writeCommand(0x20);
	epd_waitUntilIdle();
}

void epd_set_partial_window(int x_start, int y_start, int x_end, int y_end)
{
	epd_writeCommand(0x44);
	epd_writeData(x_start & 0xFF);
	epd_writeData((x_start >> 8) & 0xFF);
	epd_writeData(x_end & 0xFF);
	epd_writeData((x_end >> 8) & 0xFF);

	epd_writeCommand(0x45);
	epd_writeData(y_start & 0xFF);
	epd_writeData((y_start >> 8) & 0xFF);
	epd_writeData(y_end & 0xFF);
	epd_writeData((y_end >> 8) & 0xFF);

	epd_writeCommand(0x4E);
	epd_writeData(x_start & 0xFF);
	epd_writeData((x_start >> 8) & 0xFF);

	epd_writeCommand(0x4F);
	epd_writeData(y_start & 0xFF);
	epd_writeData((y_start >> 8) & 0xFF);
}

void epd_display_partial(int x_start, int y_start, const uint8_t *data,
			 int part_w, int part_h)
{
	int x_end, y_end;

	x_start -= x_start % 8;		/* X start must be byte aligned */
	x_end = x_start + part_w - 1;
	y_end = y_start + part_h - 1;

	epd_rst_set(0);
	sleep_ms(10);
	epd_rst_set(1);
	sleep_ms(10);
	epd_waitUntilIdle();

	epd_writeCommand(0x18);
	epd_writeData(0x80);

	epd_writeCommand(0x3C);		/* border waveform for partial */
	epd_writeData(0x80);

	epd_set_partial_window(x_start, y_start, x_end, y_end);

	epd_writeCommand(0x24);
	epd_writeData_bulk(data, part_w * part_h / 8);

	epd_partial_update();
}

void epd_display_partial_all(const uint8_t *data)
{
	epd_display_partial(0, 0, data, EPD_WIDTH, EPD_HEIGHT);
}

static uint8_t partial_buf[EPD_BUF_SIZE];
static uint8_t demo_img_a[EPD_BUF_SIZE];
static uint8_t demo_img_b[EPD_BUF_SIZE];

/* 7-segment digit bits: a=0 b=1 c=2 d=3 e=4 f=5 g=6 */
static const uint8_t seg7[10] = {
	0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F,
};

static void fb_clear(uint8_t *buf, int W, int H)
{
	memset(buf, 0xFF, W / 8 * H);
}

static void fb_rect(uint8_t *buf, int W, int H, int x0, int y0, int x1, int y1)
{
	for (int y = y0; y < y1; y++) {
		if (y < 0 || y >= H)
			continue;
		for (int x = x0; x < x1; x++) {
			int xm;
			if (x < 0 || x >= W)
				continue;
			xm = W - 1 - x;
			buf[y * (W / 8) + (xm >> 3)] &= ~(0x80 >> (xm & 7));
		}
	}
}

/* hollow rectangle of border thickness t */
static void fb_frame(uint8_t *buf, int W, int H, int x0, int y0, int x1, int y1, int t)
{
	fb_rect(buf, W, H, x0,     y0,     x1,     y0 + t);	/* top */
	fb_rect(buf, W, H, x0,     y1 - t, x1,     y1);		/* bottom */
	fb_rect(buf, W, H, x0,     y0,     x0 + t, y1);		/* left */
	fb_rect(buf, W, H, x1 - t, y0,     x1,     y1);		/* right */
}

/* big 7-segment digit at (px,py) in a W x H buffer; cell w x h, thickness t */
static void fb_digit(uint8_t *buf, int W, int H, int px, int py,
		     int w, int h, int t, int d)
{
	const int mid = (h - t) / 2;
	uint8_t s = seg7[d];

	if (s & 0x01) fb_rect(buf, W, H, px + t,     py,         px + w - t, py + t);       /* a */
	if (s & 0x02) fb_rect(buf, W, H, px + w - t, py + t,     px + w,     py + h / 2);   /* b */
	if (s & 0x04) fb_rect(buf, W, H, px + w - t, py + h / 2, px + w,     py + h - t);   /* c */
	if (s & 0x08) fb_rect(buf, W, H, px + t,     py + h - t, px + w - t, py + h);       /* d */
	if (s & 0x10) fb_rect(buf, W, H, px,         py + h / 2, px + t,     py + h - t);   /* e */
	if (s & 0x20) fb_rect(buf, W, H, px,         py + t,     px + t,     py + h / 2);   /* f */
	if (s & 0x40) fb_rect(buf, W, H, px + t,     py + mid,   px + w - t, py + mid + t); /* g */
}

/* two clearly-different full-screen images: a framed "1" and a framed "2" */
static void build_demo_images(void)
{
	const int w = 200, h = 360, t = 40;
	const int px = (EPD_WIDTH - w) / 2, py = (EPD_HEIGHT - h) / 2;

	fb_clear(demo_img_a, EPD_WIDTH, EPD_HEIGHT);
	fb_frame(demo_img_a, EPD_WIDTH, EPD_HEIGHT, 8, 8, EPD_WIDTH - 8, EPD_HEIGHT - 8, 8);
	fb_digit(demo_img_a, EPD_WIDTH, EPD_HEIGHT, px, py, w, h, t, 1);

	fb_clear(demo_img_b, EPD_WIDTH, EPD_HEIGHT);
	fb_frame(demo_img_b, EPD_WIDTH, EPD_HEIGHT, 8, 8, EPD_WIDTH - 8, EPD_HEIGHT - 8, 8);
	fb_digit(demo_img_b, EPD_WIDTH, EPD_HEIGHT, px, py, w, h, t, 2);
}

/* Demo: switch between two different images via flicker-free partial refresh. */
void epd_display_partial_demo(void)
{
	build_demo_images();

	epd_set_base_map(demo_img_a);
	sleep_ms(2000);

	for (int round = 0; round < 5; round++) {
		epd_display_partial_all(demo_img_b);
		sleep_ms(2000);

		epd_display_partial_all(demo_img_a);
		sleep_ms(2000);
	}
}

void epd_display_partial_2buf(int x_start, int y_start, const uint8_t *prev,
			      const uint8_t *cur, int part_w, int part_h)
{
	int x_end, y_end, n;

	x_start -= x_start % 8;
	x_end = x_start + part_w - 1;
	y_end = y_start + part_h - 1;
	n = part_w * part_h / 8;

	epd_rst_set(0);
	sleep_ms(10);
	epd_rst_set(1);
	sleep_ms(10);
	epd_waitUntilIdle();

	epd_writeCommand(0x18);
	epd_writeData(0x80);

	epd_writeCommand(0x3C);		/* border waveform for partial */
	epd_writeData(0x80);

	epd_set_partial_window(x_start, y_start, x_end, y_end);

	epd_writeCommand(0x26);		/* old frame */
	epd_writeData_bulk(prev, n);

	epd_writeCommand(0x4E);		/* rewind counters for the new frame */
	epd_writeData(x_start & 0xFF);
	epd_writeData((x_start >> 8) & 0xFF);
	epd_writeCommand(0x4F);
	epd_writeData(y_start & 0xFF);
	epd_writeData((y_start >> 8) & 0xFF);

	epd_writeCommand(0x24);		/* new frame */
	epd_writeData_bulk(cur, n);

	epd_partial_update();
}

/* ---- 7-segment clock, rendered into a bottom-right region buffer ---- */

#define CLK_DIGIT_W   44
#define CLK_DIGIT_H   80
#define CLK_SEG_T     10	/* segment thickness */
#define CLK_GAP        8	/* gap between glyphs */
#define CLK_COLON_W   20

/* "HH:MM:SS" -> 6 digits + 2 colons; advances sum to exactly 360 dots */
#define CLK_REGION_W 360	/* multiple of 8 */
#define CLK_REGION_H  80

/* This panel mirrors RAM-X (RAM-X 0 is the physical right edge): start the
 * window at 16 to land bottom-right, and fb_rect() pre-mirrors X. */
#define CLK_X        16
#define CLK_Y        (EPD_HEIGHT - CLK_REGION_H - 16)

static uint8_t clk_cur[CLK_REGION_W / 8 * CLK_REGION_H];
static uint8_t clk_prev[CLK_REGION_W / 8 * CLK_REGION_H];

static void clk_draw_digit(uint8_t *buf, int px, int d)
{
	fb_digit(buf, CLK_REGION_W, CLK_REGION_H, px, 0,
		 CLK_DIGIT_W, CLK_DIGIT_H, CLK_SEG_T, d);
}

static void clk_draw_colon(uint8_t *buf, int px)
{
	const int h = CLK_DIGIT_H, t = CLK_SEG_T;
	const int cx = px + (CLK_COLON_W - t) / 2;

	fb_rect(buf, CLK_REGION_W, CLK_REGION_H, cx, h / 3 - t / 2,     cx + t, h / 3 + t / 2);
	fb_rect(buf, CLK_REGION_W, CLK_REGION_H, cx, 2 * h / 3 - t / 2, cx + t, 2 * h / 3 + t / 2);
}

static void clk_render(uint8_t *buf, int hh, int mm, int ss)
{
	int px = 0;

	fb_clear(buf, CLK_REGION_W, CLK_REGION_H);
	clk_draw_digit(buf, px, hh / 10); px += CLK_DIGIT_W + CLK_GAP;
	clk_draw_digit(buf, px, hh % 10); px += CLK_DIGIT_W + CLK_GAP;
	clk_draw_colon(buf, px);          px += CLK_COLON_W + CLK_GAP;
	clk_draw_digit(buf, px, mm / 10); px += CLK_DIGIT_W + CLK_GAP;
	clk_draw_digit(buf, px, mm % 10); px += CLK_DIGIT_W + CLK_GAP;
	clk_draw_colon(buf, px);          px += CLK_COLON_W + CLK_GAP;
	clk_draw_digit(buf, px, ss / 10); px += CLK_DIGIT_W + CLK_GAP;
	clk_draw_digit(buf, px, ss % 10);
}

/* White base map, then partial-refresh the bottom-right HH:MM:SS once per
 * second; a full refresh every 5 minutes clears accumulated ghosting. */
void epd_display_clock_demo(void)
{
	int count = 0;

	memset(partial_buf, 0xFF, EPD_BUF_SIZE);
	epd_set_base_map(partial_buf);
	memset(clk_prev, 0xFF, sizeof(clk_prev));

	while (true) {
		time_t now = time(NULL);
		struct tm *t = localtime(&now);

		clk_render(clk_cur, t->tm_hour, t->tm_min, t->tm_sec);
		epd_display_partial_2buf(CLK_X, CLK_Y, clk_prev, clk_cur,
					 CLK_REGION_W, CLK_REGION_H);
		memcpy(clk_prev, clk_cur, sizeof(clk_cur));

		if (++count >= 300) {
			count = 0;
			epd_init();
			memset(partial_buf, 0xFF, EPD_BUF_SIZE);
			epd_set_base_map(partial_buf);
			memset(clk_prev, 0xFF, sizeof(clk_prev));
		}

		/* re-align to the next wall-clock second so it ticks once/sec */
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		sleep_ms(1000 - ts.tv_nsec / 1000000);
	}
}

int main(int argc, char **argv) {
	while (true) {
		epd_init();
		if (argc > 1 && !strcmp(argv[1], "gray"))
			epd_display_grayscaled_image();
		else if (argc > 1 && !strcmp(argv[1], "partial"))
			epd_display_partial_demo();
		else if (argc > 1 && !strcmp(argv[1], "clock"))
			epd_display_clock_demo();
		else
			epd_display_mono_image();
		sleep_ms(30 * 1000);
	}

	close(spi_fd);
	gpiod_line_request_release(epd_dc_line);
	gpiod_line_request_release(epd_rst_line);
	gpiod_line_request_release(epd_busy_line);
	gpiod_chip_close(chip);

	return 0;
}
