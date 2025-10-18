/*
 * Author: Wig Cheng <onlywig@gmail.com>
 * Date: 2025-10-19
 *
 * Optimized version for TOPST D3-G platform
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
#define EPD_GPIO_CHIP "gpiochip4"

#define EPD_DC_PIN 1
#define EPD_RST_PIN 2
#define EPD_BUSY_PIN 6

#define SPI_SPEED 20000000

int spi_fd;
struct gpiod_chip *chip;
struct gpiod_line *epd_dc_line, *epd_rst_line, *epd_busy_line;

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
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("SPI write failed");
    }
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

void epd_writeDataBatch(uint8_t *data, int len) {
    gpiod_line_set_value(epd_dc_line, 1);
    sleep_us(1);

    int chunk_size = 4096;
    int offset = 0;

    while (offset < len) {
        int remaining = len - offset;
        int current_chunk = (remaining < chunk_size) ? remaining : chunk_size;

        struct spi_ioc_transfer tr = {
            .tx_buf = (unsigned long)(data + offset),
            .len = current_chunk,
            .speed_hz = SPI_SPEED,
            .bits_per_word = 8,
        };

        if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
            perror("SPI batch write failed");
            break;
        }

        offset += current_chunk;
    }
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

uint8_t img_fetch_hex_32(int y, int x, uint32_t *img_src) {
    int hsize = 8;
    if (y >= 122 || x >= 250) {
        return 0b1;
    } else {
        uint32_t target = *(img_src + hsize * y + (x / 32));
        return (target >> (31 - x % 32)) & 0b1;
    }
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

void epd_write_img(uint32_t *img_src) {
    epd_writeCommand(0x24);
    sleep_ms(10);

    const int total_bytes = 250 * 16;
    uint8_t *img_buffer = malloc(total_bytes);
    if (!img_buffer) {
        perror("Failed to allocate image buffer");
        return;
    }

    printf("Preparing image data...\n");

    int buffer_idx = 0;
    for (uint x = 0; x < 250; x++) {
        for (uint y_group = 0; y_group < 16; y_group++) {
            uint8_t data = 0;
            for (int bit = 0; bit < 8; bit++) {
                uint y = y_group * 8 + bit;
                if (y < 128) {
                    uint8_t pixel = img_fetch_hex_32(y, x, img_src);
                    data |= (pixel << (7 - bit));
                }
            }
            img_buffer[buffer_idx++] = data;
        }
    }

    printf("Writing %d bytes to display...\n", total_bytes);

    epd_writeDataBatch(img_buffer, total_bytes);

    free(img_buffer);

    printf("write image successed, starting update image\n");

    epd_writeCommand(0x22);
    sleep_ms(10);
    epd_writeData(0xF7);
    epd_writeCommand(0x20);
    epd_waitUntilIdle();

    printf("update image successed\n");
}

int main() {
    epd_init();

    while (true) {
        epd_write_img(&img0[0]);
        sleep_ms(3 * 1000);
    }

    close(spi_fd);
    gpiod_line_release(epd_dc_line);
    gpiod_line_release(epd_rst_line);
    gpiod_line_release(epd_busy_line);
    gpiod_chip_close(chip);

    return 0;
}
