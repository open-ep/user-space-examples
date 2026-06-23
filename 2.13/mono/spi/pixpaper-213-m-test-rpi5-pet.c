/*
 * Author: LC Wang <zaq14760@gmail.com>
 * Date: 2026-06-17
 *
 * Open-EP Pet: a Tamagotchi-style virtual pet on the 2.13" mono PixPaper
 * (250x122 landscape).
 *
 * Orientation only changes the logical->RAM coordinate map in fb_set(); the
 * portrait map is a true 90deg rotation (transpose + x-flip, not a mirror).
 * If portrait comes out upside-down on your unit, swap the two lines in the
 * portrait branch of fb_set() to: gate = DISP_GATE_MAX - y; bit = x;
 */
#include <fcntl.h>
#include <gpiod.h>
#include <linux/spi/spidev.h>
#include <poll.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include "jpfont.h"
#include "petsprite.h"

#define EPD_SPI_DEVICE "/dev/spidev0.0"
#define EPD_GPIO_CHIP "gpiochip15"

#define EPD_DC_PIN 5
#define EPD_RST_PIN 6
#define EPD_BUSY_PIN 26

#define SPI_SPEED 5000000

#define DISP_W 250
#define DISP_H 128
#define DISP_VIS 122
#define DISP_STRIDE (DISP_H / 8)
#define DISP_BUF_SIZE (DISP_W * DISP_STRIDE)
#define DISP_GATE_MAX (DISP_W - 1)

static int g_portrait;
static int scr_w = DISP_W;
static int scr_h = DISP_VIS;

int spi_fd;
struct gpiod_chip *chip;
struct gpiod_line_request *epd_dc_line, *epd_rst_line, *epd_busy_line;

static const unsigned int EPD_DC_OFFSET   = EPD_DC_PIN;
static const unsigned int EPD_RST_OFFSET  = EPD_RST_PIN;
static const unsigned int EPD_BUSY_OFFSET = EPD_BUSY_PIN;

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
	while (epd_busy_get() != 0)
		;
}

void epd_HWreset() {
	sleep_ms(50);
	epd_rst_set(0);
	sleep_ms(50);
	epd_rst_set(1);
	sleep_ms(50);
}

void epd_reg_init(void) {
	epd_waitUntilIdle();
	epd_writeCommand(0x12);
	epd_waitUntilIdle();

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
	epd_writeData(0x05);

	epd_writeCommand(0x21);
	epd_writeData(0x00);
	epd_writeData(0x80);

	epd_writeCommand(0x18);
	epd_writeData(0x80);

	epd_writeCommand(0x4E);
	epd_writeData(0x00);
	epd_writeCommand(0x4F);
	epd_writeData(0xF9);
	epd_writeData(0x00);

	epd_waitUntilIdle();
}

void epd_init(void) {
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

	epd_HWreset();
	sleep_ms(1000);
	epd_waitUntilIdle();
	epd_reg_init();
}

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
	0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x0, 0x0, 0x0,
	0x22, 0x17, 0x41, 0x0, 0x32, 0x36,
};

void epd_load_partial_lut(void) {
	epd_writeCommand(0x32);
	for (int i = 0; i < 153; i++)
		epd_writeData(WF_PARTIAL[i]);
	epd_waitUntilIdle();

	epd_writeCommand(0x3F);
	epd_writeData(WF_PARTIAL[153]);
	epd_writeCommand(0x03);
	epd_writeData(WF_PARTIAL[154]);
	epd_writeCommand(0x04);
	epd_writeData(WF_PARTIAL[155]);
	epd_writeData(WF_PARTIAL[156]);
	epd_writeData(WF_PARTIAL[157]);
	epd_writeCommand(0x2C);
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

	epd_writeCommand(0x3C);
	epd_writeData(0x80);
}

void epd_partial_update(void) {
	epd_writeCommand(0x22);
	epd_writeData(0x0F);
	epd_writeCommand(0x20);
	epd_waitUntilIdle();
}

static void epd_partial_regs(void) {
	epd_writeCommand(0x01);
	epd_writeData(0xF9);
	epd_writeData(0x00);
	epd_writeData(0x00);

	epd_writeCommand(0x11);
	epd_writeData(0x01);

	epd_writeCommand(0x21);
	epd_writeData(0x00);
	epd_writeData(0x80);

	epd_writeCommand(0x18);
	epd_writeData(0x80);
}

void epd_display_partial_full(const uint8_t *image) {
	epd_rst_set(0);
	sleep_ms(2);
	epd_rst_set(1);
	sleep_ms(2);

	epd_partial_regs();
	epd_load_partial_lut();

	epd_writeCommand(0x22);
	epd_writeData(0xC0);
	epd_writeCommand(0x20);
	epd_waitUntilIdle();

	epd_set_full_window();
	epd_set_full_cursor();
	epd_writeCommand(0x24);
	epd_writeData_bulk(image, DISP_BUF_SIZE);

	epd_partial_update();
}

static inline void fb_set(uint8_t *buf, int x, int y) {
	int gate, bit;

	if (x < 0 || x >= scr_w || y < 0 || y >= scr_h)
		return;

	if (g_portrait) {
		gate = y;
		bit  = DISP_VIS - 1 - x;
	} else {
		gate = x;
		bit  = y;
	}
	buf[gate * DISP_STRIDE + (bit >> 3)] &= ~(0x80 >> (bit & 7));
}

static void fb_clear(uint8_t *buf) {
	memset(buf, 0xFF, DISP_BUF_SIZE);
}

static void fb_rect(uint8_t *buf, int x0, int y0, int x1, int y1) {
	for (int x = x0; x < x1; x++)
		for (int y = y0; y < y1; y++)
			fb_set(buf, x, y);
}

static void fb_hline(uint8_t *buf, int x, int y, int len, int th) {
	for (int i = 0; i < len; i++)
		for (int t = 0; t < th; t++)
			fb_set(buf, x + i, y + t);
}

static void fb_vline(uint8_t *buf, int x, int y, int len, int th) {
	for (int i = 0; i < len; i++)
		for (int t = 0; t < th; t++)
			fb_set(buf, x + t, y + i);
}

static void fb_frame(uint8_t *buf, int x0, int y0, int x1, int y1) {
	fb_hline(buf, x0, y0, x1 - x0 + 1, 1);
	fb_hline(buf, x0, y1, x1 - x0 + 1, 1);
	fb_vline(buf, x0, y0, y1 - y0 + 1, 1);
	fb_vline(buf, x1, y0, y1 - y0 + 1, 1);
}

static void fb_disc(uint8_t *buf, int cx, int cy, int r) {
	for (int dy = -r; dy <= r; dy++)
		for (int dx = -r; dx <= r; dx++)
			if (dx * dx + dy * dy <= r * r)
				fb_set(buf, cx + dx, cy + dy);
}

static const uint8_t font5x7[96][8] = {
	[' '  - 32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	['('  - 32] = {0x10, 0x20, 0x40, 0x40, 0x40, 0x20, 0x10, 0x00},
	[')'  - 32] = {0x40, 0x20, 0x10, 0x10, 0x10, 0x20, 0x40, 0x00},
	['-'  - 32] = {0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00},
	['['  - 32] = {0x30, 0x20, 0x20, 0x20, 0x20, 0x20, 0x30, 0x00},
	[']'  - 32] = {0x60, 0x20, 0x20, 0x20, 0x20, 0x20, 0x60, 0x00},
	['C'  - 32] = {0x70, 0x88, 0x80, 0x80, 0x80, 0x88, 0x70, 0x00},
	['E'  - 32] = {0xF8, 0x80, 0x80, 0xF0, 0x80, 0x80, 0xF8, 0x00},
	['F'  - 32] = {0xF8, 0x80, 0x80, 0xF0, 0x80, 0x80, 0x80, 0x00},
	['H'  - 32] = {0x88, 0x88, 0x88, 0xF8, 0x88, 0x88, 0x88, 0x00},
	['M'  - 32] = {0x88, 0xD8, 0xA8, 0x88, 0x88, 0x88, 0x88, 0x00},
	['O'  - 32] = {0x70, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70, 0x00},
	['P'  - 32] = {0xF0, 0x88, 0x88, 0xF0, 0x80, 0x80, 0x80, 0x00},
	['Z'  - 32] = {0xF8, 0x08, 0x10, 0x20, 0x40, 0x80, 0xF8, 0x00},
	['a'  - 32] = {0x00, 0x00, 0x70, 0x08, 0x78, 0x88, 0x78, 0x00},
	['d'  - 32] = {0x08, 0x08, 0x78, 0x88, 0x88, 0x88, 0x78, 0x00},
	['e'  - 32] = {0x00, 0x70, 0x88, 0xF8, 0x80, 0x70, 0x00, 0x00},
	['g'  - 32] = {0x00, 0x78, 0x88, 0x88, 0x78, 0x08, 0x70, 0x00},
	['l'  - 32] = {0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x30, 0x00},
	['n'  - 32] = {0x00, 0x00, 0xF0, 0x88, 0x88, 0x88, 0x88, 0x00},
	['o'  - 32] = {0x00, 0x00, 0x70, 0x88, 0x88, 0x88, 0x70, 0x00},
	['p'  - 32] = {0x00, 0x00, 0xF0, 0x88, 0x88, 0xF0, 0x80, 0x80},
	['r'  - 32] = {0x00, 0x00, 0xB0, 0xC0, 0x80, 0x80, 0x80, 0x00},
	['t'  - 32] = {0x40, 0x40, 0xE0, 0x40, 0x40, 0x40, 0x30, 0x00},
	['u'  - 32] = {0x00, 0x00, 0x88, 0x88, 0x88, 0x88, 0x78, 0x00},
	['y'  - 32] = {0x00, 0x00, 0x88, 0x88, 0x78, 0x08, 0x70, 0x00},
	['z'  - 32] = {0x00, 0x00, 0xF8, 0x10, 0x20, 0x40, 0xF8, 0x00},
};

static int draw_char_ex(uint8_t *buf, int x, int y, char c, int sc, int bold) {
	unsigned char uc = (unsigned char)c;
	if (uc < 32 || uc > 127)
		uc = ' ';
	const uint8_t *g = font5x7[uc - 32];
	for (int row = 0; row < 8; row++) {
		uint8_t bits = g[row];
		for (int col = 0; col < 8; col++)
			if (bits & (0x80 >> col)) {
				int px = x + col * sc, py = y + row * sc;
				fb_rect(buf, px, py, px + sc + (bold ? 1 : 0), py + sc);
			}
	}
	return 6 * sc + (bold ? 1 : 0);
}

static int draw_text_ex(uint8_t *buf, int x, int y, const char *s, int sc, int bold) {
	for (; *s; s++)
		x += draw_char_ex(buf, x, y, *s, sc, bold);
	return x;
}
static int text_w_latin(const char *s, int sc, int bold) {
	return (int)strlen(s) * (6 * sc + (bold ? 1 : 0));
}

#define JP_GAP 1

static uint32_t utf8_next(const char **s) {
	const unsigned char *p = (const unsigned char *)*s;
	uint32_t cp;
	if (p[0] < 0x80) { cp = p[0]; *s += p[0] ? 1 : 0; return cp; }
	if ((p[0] & 0xE0) == 0xC0) { cp = ((p[0] & 0x1F) << 6) | (p[1] & 0x3F); *s += 2; }
	else if ((p[0] & 0xF0) == 0xE0) {
		cp = ((p[0] & 0x0F) << 12) | ((p[1] & 0x3F) << 6) | (p[2] & 0x3F);
		*s += 3;
	} else { cp = '?'; *s += 4; }
	return cp;
}

static const struct jpglyph *jp_find(uint32_t cp, int size) {
	for (unsigned i = 0; i < JPGLYPH_COUNT; i++)
		if (jpglyphs[i].cp == cp && jpglyphs[i].size == size)
			return &jpglyphs[i];
	return NULL;
}

static void jp_blit(uint8_t *buf, int x, int y, const struct jpglyph *g) {
	int stride = (g->w + 7) / 8;
	for (int row = 0; row < g->h; row++)
		for (int col = 0; col < g->w; col++)
			if (jpbits[g->off + row * stride + (col >> 3)] & (0x80 >> (col & 7)))
				fb_set(buf, x + col, y + row);
}

static int draw_text_jp(uint8_t *buf, int x, int y, const char *s, int size) {
	while (*s) {
		uint32_t cp = utf8_next(&s);
		const struct jpglyph *g = jp_find(cp, size);
		if (g) {
			jp_blit(buf, x, y, g);
			x += g->w + JP_GAP;
		} else {
			x += size + JP_GAP;
		}
	}
	return x;
}

static int text_w_jp(const char *s, int size) {
	int w = 0;
	while (*s) {
		uint32_t cp = utf8_next(&s);
		const struct jpglyph *g = jp_find(cp, size);
		w += (g ? g->w : size) + JP_GAP;
	}
	return w > 0 ? w - JP_GAP : 0;
}

#define STAT_MAX 6

struct face_geo {
	int lx, rx;
	int eye_y;
	int mouth_y;
	int eye_r;
};

enum eye_mode  { EYE_OPEN, EYE_CLOSED, EYE_LEFT, EYE_RIGHT, EYE_HAPPY };
enum mouth_mode { M_SMILE, M_FLAT, M_OPEN, M_SAD };

static void draw_zzz_at(uint8_t *buf, int x, int y) {
	draw_char_ex(buf, x,      y,      'z', 1, 0);
	draw_char_ex(buf, x + 8,  y - 10, 'z', 2, 0);
	draw_char_ex(buf, x + 22, y - 26, 'Z', 2, 1);
}

static void draw_sparkle_at(uint8_t *buf, int cx, int cy, int r) {
	int px[4] = {cx - r, cx + r, cx - r + 4, cx + r - 4};
	int py[4] = {cy - r, cy - r + 4, cy + r, cy + r - 6};
	for (int i = 0; i < 4; i++) {
		fb_hline(buf, px[i] - 2, py[i], 5, 1);
		fb_vline(buf, px[i], py[i] - 2, 5, 1);
	}
}

static void draw_heart(uint8_t *buf, int cx, int cy, int r) {
	fb_disc(buf, cx - r / 2, cy, (r + 1) / 2);
	fb_disc(buf, cx + r / 2, cy, (r + 1) / 2);
	for (int dy = 0; dy <= r; dy++) {
		int half = r - dy;
		fb_rect(buf, cx - half, cy + dy, cx + half + 1, cy + dy + 1);
	}
}

enum petframe { PF_IDLE, PF_BLINK, PF_HAPPY, PF_SAD, PF_EAT, PF_LOOK, PF_SURPRISED };

struct anim {
	int frame;
	int flip;
	int dx, dy;
	int zzz, spark, heart;
};

static void draw_sprite(uint8_t *buf, const uint8_t *spr, int x, int y, int flip) {
	for (int row = 0; row < PET_H; row++)
		for (int col = 0; col < PET_W; col++)
			if (spr[row * PET_STRIDE + (col >> 3)] & (0x80 >> (col & 7)))
				fb_set(buf, x + (flip ? PET_W - 1 - col : col), y + row);
}

static void draw_face(uint8_t *buf, const struct face_geo *g0,
		      const struct anim *a) {
	static const uint8_t *const FR[] = {
		pet_idle, pet_blink, pet_happy, pet_sad, pet_eat, pet_look, pet_surprised,
	};
	int cx = (g0->lx + g0->rx) / 2 + a->dx;
	int cy = (g0->eye_y + g0->mouth_y) / 2 + a->dy;
	int x = cx - PET_W / 2, y = cy - PET_H / 2;

	draw_sprite(buf, FR[a->frame], x, y, a->flip);

	if (a->heart)
		draw_heart(buf, cx, y - 8, 7);
	if (a->zzz)
		draw_zzz_at(buf, x + PET_W - 4, y + 6);
	if (a->spark)
		draw_sparkle_at(buf, cx, cy, PET_W / 2 + 4);
}

static void draw_track(uint8_t *buf, int x, int y, int val,
		       int sw, int gap, int sh) {
	for (int i = 0; i < STAT_MAX; i++) {
		int sx = x + i * (sw + gap);
		fb_frame(buf, sx, y, sx + sw, y + sh);
		if (i < val)
			fb_rect(buf, sx + 1, y + 1, sx + sw, y + sh);
	}
}

static void draw_button_jp(uint8_t *buf, int x, int y, const char *label,
			   int size, int active) {
	int w = text_w_jp(label, size) + 8;
	int h = size + 6;
	fb_frame(buf, x, y, x + w, y + h);
	if (active)
		fb_frame(buf, x - 2, y - 2, x + w + 2, y + h + 2);
	draw_text_jp(buf, x + 4, y + 3, label, size);
}

enum action { ACT_NONE, ACT_FEED, ACT_PLAY, ACT_CLEAN };

enum gesture {
	G_NONE, G_BLINK, G_DBLINK, G_LOOKL, G_LOOKR, G_WIGGLE, G_YAWN, G_HEART,
};

#define ACTION_LEN    6
#define SLEEP_AFTER  40
#define DECAY_EVERY  20
#define GHOST_EVERY 200

static uint8_t frame_buf[DISP_BUF_SIZE];

struct pet {
	int hunger, energy, mood;
	enum action action;
	int action_frames;
	int last_interaction;

	enum gesture gesture;
	int gesture_frames;
	int gesture_total;
	int idle_gap;
};

static int clampi(int v, int lo, int hi) {
	return v < lo ? lo : (v > hi ? hi : v);
}

static void pet_trigger(struct pet *p, enum action a, int frame) {
	p->action = a;
	p->action_frames = ACTION_LEN;
	p->last_interaction = frame;
	p->gesture = G_NONE;
	p->gesture_frames = 0;
	switch (a) {
	case ACT_FEED:  p->hunger = clampi(p->hunger + 2, 0, STAT_MAX); break;
	case ACT_PLAY:  p->mood = clampi(p->mood + 2, 0, STAT_MAX);
			p->energy = clampi(p->energy - 1, 0, STAT_MAX); break;
	case ACT_CLEAN: p->mood = clampi(p->mood + 1, 0, STAT_MAX); break;
	default: break;
	}
}

static void gesture_start(struct pet *p, enum gesture g, int len) {
	p->gesture = g;
	p->gesture_frames = len;
	p->gesture_total = len;
}

static void pet_idle_tick(struct pet *p) {
	if (p->action != ACT_NONE || p->hunger == 0)
		return;

	if (p->gesture != G_NONE) {
		if (--p->gesture_frames <= 0) {
			p->gesture = G_NONE;
			p->idle_gap = rand() % 2;
		}
		return;
	}
	if (p->idle_gap > 0) {
		p->idle_gap--;
		return;
	}

	int r = rand() % 100;
	if (r < 12)       gesture_start(p, G_BLINK,  1);
	else if (r < 26)  gesture_start(p, G_DBLINK, 3);
	else if (r < 44)  gesture_start(p, G_LOOKL,  4);
	else if (r < 62)  gesture_start(p, G_LOOKR,  4);
	else if (r < 84)  gesture_start(p, G_WIGGLE, 6);
	else if (r < 92)  gesture_start(p, G_YAWN,   3);
	else              gesture_start(p, G_HEART,  5);
}

static void compute_face_state(const struct pet *p, int frame, struct anim *a) {
	memset(a, 0, sizeof(*a));
	a->frame = PF_IDLE;

	if (p->action != ACT_NONE) {
		int phase = ACTION_LEN - p->action_frames;
		if (phase == 0) {
			a->frame = PF_SURPRISED;
			return;
		}
		switch (p->action) {
		case ACT_FEED:  a->frame = (phase & 1) ? PF_EAT : PF_IDLE; break;
		case ACT_PLAY:  a->frame = PF_HAPPY; a->dy = (phase & 1) ? -5 : 0; break;
		case ACT_CLEAN: a->frame = PF_HAPPY; a->spark = 1; break;
		default: break;
		}
		return;
	}
	if (p->hunger == 0) {
		a->frame = PF_SAD;
		return;
	}
	if (frame - p->last_interaction > SLEEP_AFTER) {
		a->frame = PF_BLINK;
		a->zzz = 1;
		return;
	}

	int ph = p->gesture_total - p->gesture_frames;
	switch (p->gesture) {
	case G_BLINK:  a->frame = PF_BLINK; break;
	case G_DBLINK: a->frame = (ph == 1) ? PF_IDLE : PF_BLINK; break;
	case G_LOOKL:  a->frame = PF_LOOK; break;
	case G_LOOKR:  a->frame = PF_LOOK; a->flip = 1; break;
	case G_WIGGLE: a->frame = PF_HAPPY; a->dx = (ph & 1) ? 3 : -3; break;
	case G_YAWN:   a->frame = PF_SURPRISED; break;
	case G_HEART:  a->frame = PF_HAPPY; a->heart = 1; break;
	default:       a->frame = PF_IDLE; break;
	}
}

static void compose_landscape(const struct pet *p, const struct anim *a) {
	static const struct face_geo g = {62, 92, 46, 66, 7};
	static const struct { const char *name; int val_off; } row[3] = {
		{"空腹", offsetof(struct pet, hunger)},
		{"元気", offsetof(struct pet, energy)},
		{"気分", offsetof(struct pet, mood)},
	};

	fb_frame(frame_buf, 2, 0, 247, 121);
	int tx = draw_text_ex(frame_buf, 8, 2, "Open-EP", 2, 1);
	draw_text_jp(frame_buf, tx + 6, 2, "ペット", 16);
	fb_hline(frame_buf, 4, 20, 242, 1);

	draw_face(frame_buf, &g, a);

	for (int i = 0; i < 3; i++) {
		int val = *(const int *)((const char *)p + row[i].val_off);
		int ly = 24 + i * 26;
		draw_text_jp(frame_buf, 132, ly, row[i].name, 20);
		draw_track(frame_buf, 176, ly + 3, val, 9, 1, 14);
	}

	draw_button_jp(frame_buf, 8,   98, "エサ",   14, p->action == ACT_FEED);
	draw_button_jp(frame_buf, 92,  98, "あそぶ", 14, p->action == ACT_PLAY);
	draw_button_jp(frame_buf, 168, 98, "そうじ", 14, p->action == ACT_CLEAN);
}

static void compose_portrait(const struct pet *p, const struct anim *a) {
	static const struct face_geo g = {46, 76, 68, 92, 7};
	static const struct { const char *name; int val_off; } row[3] = {
		{"空腹", offsetof(struct pet, hunger)},
		{"元気", offsetof(struct pet, energy)},
		{"気分", offsetof(struct pet, mood)},
	};

	fb_frame(frame_buf, 3, 1, 118, 248);

	draw_text_ex(frame_buf, (122 - text_w_latin("Open-EP", 2, 1)) / 2, 3,
		     "Open-EP", 2, 1);
	draw_text_jp(frame_buf, (122 - text_w_jp("ペット", 16)) / 2, 22, "ペット", 16);
	fb_hline(frame_buf, 4, 40, 114, 1);

	draw_face(frame_buf, &g, a);

	draw_button_jp(frame_buf, 6,  126, "エサ",   14, p->action == ACT_FEED);
	draw_button_jp(frame_buf, 52, 126, "あそぶ", 14, p->action == ACT_PLAY);
	draw_button_jp(frame_buf, 35, 150, "そうじ", 14, p->action == ACT_CLEAN);

	for (int i = 0; i < 3; i++) {
		int val = *(const int *)((const char *)p + row[i].val_off);
		int ly = 176 + i * 24;
		draw_text_jp(frame_buf, 8, ly, row[i].name, 20);
		draw_track(frame_buf, 52, ly + 3, val, 9, 1, 14);
	}
}

static void compose_frame(const struct pet *p, int frame) {
	struct anim a;

	compute_face_state(p, frame, &a);

	fb_clear(frame_buf);
	if (g_portrait)
		compose_portrait(p, &a);
	else
		compose_landscape(p, &a);
}

static struct termios kbd_orig;
static int kbd_raw;

static void kbd_enable(void) {
	if (!isatty(STDIN_FILENO))
		return;
	tcgetattr(STDIN_FILENO, &kbd_orig);
	struct termios t = kbd_orig;
	t.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &t);
	int fl = fcntl(STDIN_FILENO, F_GETFL);
	fcntl(STDIN_FILENO, F_SETFL, fl | O_NONBLOCK);
	kbd_raw = 1;
}

static void kbd_restore(void) {
	if (kbd_raw)
		tcsetattr(STDIN_FILENO, TCSANOW, &kbd_orig);
}

static int kbd_get(void) {
	unsigned char c;
	return read(STDIN_FILENO, &c, 1) == 1 ? c : -1;
}

int main(int argc, char **argv) {
	int auto_mode = !isatty(STDIN_FILENO);

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "auto"))
			auto_mode = 1;
		else if (!strcmp(argv[i], "portrait"))
			g_portrait = 1;
		else if (!strcmp(argv[i], "landscape"))
			g_portrait = 0;
	}

	if (g_portrait) {
		scr_w = DISP_VIS;
		scr_h = DISP_W;
	} else {
		scr_w = DISP_W;
		scr_h = DISP_VIS;
	}

	struct pet pet = {
		.hunger = 4, .energy = 5, .mood = 5,
		.action = ACT_NONE, .action_frames = 0, .last_interaction = 0,
		.gesture = G_NONE, .gesture_frames = 0, .idle_gap = 0,
	};

	srand((unsigned)time(NULL));

	epd_init();

	compose_frame(&pet, 0);
	epd_set_base_map(frame_buf);
	sleep_ms(500);

	kbd_enable();

	const enum action demo_seq[] = {
		ACT_FEED, ACT_NONE, ACT_PLAY, ACT_NONE, ACT_CLEAN, ACT_NONE,
	};
	int demo_i = 0;
	int running = 1;

	for (int frame = 1; running; frame++) {

		if (auto_mode) {
			if (pet.action == ACT_NONE && frame % 4 == 0) {
				enum action a = demo_seq[demo_i++ % (int)(sizeof(demo_seq) / sizeof(demo_seq[0]))];
				if (a != ACT_NONE)
					pet_trigger(&pet, a, frame);
			}
		} else {
			int c;
			while ((c = kbd_get()) != -1) {
				switch (c) {
				case 'f': pet_trigger(&pet, ACT_FEED, frame); break;
				case 'p': pet_trigger(&pet, ACT_PLAY, frame); break;
				case 'c': pet_trigger(&pet, ACT_CLEAN, frame); break;
				case 'q': running = 0; break;
				}
			}
		}

		if (frame % DECAY_EVERY == 0) {
			pet.hunger = clampi(pet.hunger - 1, 0, STAT_MAX);
			if (frame % (DECAY_EVERY * 2) == 0)
				pet.energy = clampi(pet.energy - 1, 0, STAT_MAX);
			if (pet.hunger <= 1)
				pet.mood = clampi(pet.mood - 1, 0, STAT_MAX);
		}

		pet_idle_tick(&pet);

		compose_frame(&pet, frame);
		epd_display_partial_full(frame_buf);

		if (pet.action != ACT_NONE && --pet.action_frames <= 0)
			pet.action = ACT_NONE;

		if (frame % GHOST_EVERY == 0) {
			epd_HWreset();
			epd_reg_init();
			epd_set_base_map(frame_buf);
		}

		sleep_ms(40);
	}

	kbd_restore();
	close(spi_fd);
	gpiod_line_request_release(epd_dc_line);
	gpiod_line_request_release(epd_rst_line);
	gpiod_line_request_release(epd_busy_line);
	gpiod_chip_close(chip);
	return 0;
}
