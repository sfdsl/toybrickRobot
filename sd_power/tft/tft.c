/*
Copyright (c) 2023 Rockchip Electronics Co., Ltd.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

    (1) Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer. 

    (2) Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.  
    
    (3)The name of the author may not be used to
    endorse or promote products derived from this software without
    specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE. 
 * Change Logs:
 * Date           Author           Notes
 * 2023-07-14     Sisyphean Zhou   first implementation
 */

#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include "tft.h"
#include "font.h"

static int g_bl_fd = -1;
static int g_dc_fd = -1;
static int g_rst_fd = -1;
static int g_spi_fd = -1;
static uint16_t g_disp_buff[TFT_H][TFT_W];
static void tft_gpio_init(void)
{
    int fd;
    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0)
        goto fail;
    write(fd, BACK_LIGHT, strlen(BACK_LIGHT));
    write(fd, DC_PIN, strlen(DC_PIN));
    write(fd, RST_PIN, strlen(RST_PIN));
    close(fd);

    fd = open("/sys/class/gpio/gpio" BACK_LIGHT "/direction", O_WRONLY);
    if (fd < 0)
        goto fail;
    write(fd, "out", strlen("out"));
    close(fd);

    fd = open("/sys/class/gpio/gpio" DC_PIN "/direction", O_WRONLY);
    if (fd < 0)
        goto fail;
    write(fd, "out", strlen("out"));
    close(fd);

    fd = open("/sys/class/gpio/gpio" RST_PIN "/direction", O_WRONLY);
    if (fd < 0)
        goto fail;
    write(fd, "out", strlen("out"));
    close(fd);

    g_bl_fd = open("/sys/class/gpio/gpio" BACK_LIGHT "/value", O_WRONLY);
    if (g_bl_fd < 0)
        goto fail;
    write(g_bl_fd, "1", 2);

    g_dc_fd = open("/sys/class/gpio/gpio" DC_PIN "/value", O_WRONLY);
    if (g_dc_fd < 0)
        goto fail;

    g_rst_fd = open("/sys/class/gpio/gpio" RST_PIN "/value", O_WRONLY);
    if (g_rst_fd < 0)
        goto fail;
    return;
fail:
    g_bl_fd = -1;
    g_dc_fd = -1;
    g_rst_fd = -1;
    printf("open gpio fial\n");
    exit(-1);
}

static void tft_gpio_deinit(void)
{
    int fd;
    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd < 0)
    {
        printf("deinit gpio fial\n");
        exit(-1);
    }

    write(fd, BACK_LIGHT, strlen(BACK_LIGHT));
    write(fd, DC_PIN, strlen(DC_PIN));
    write(fd, RST_PIN, strlen(RST_PIN));
    write(fd, CS_PIN, strlen(CS_PIN));
    close(fd);
}

static uint32_t mode = SPI_MODE_0;
static uint8_t bits = 8;
static uint32_t speed = 2000000;

static void spi_init(void)
{
    int ret;
    g_spi_fd = open(TFT_SPI, O_RDWR);
    if (g_spi_fd < 0)
    {
        printf("open spi fail");
        exit(-1);
    }
    ret = ioctl(g_spi_fd, SPI_IOC_WR_MODE32, &mode);
    if (ret == -1)
        goto fail;

    ret = ioctl(g_spi_fd, SPI_IOC_RD_MODE32, &mode);
    if (ret == -1)
        goto fail;

    ret = ioctl(g_spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1)
        goto fail;

    ret = ioctl(g_spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1)
        goto fail;

    ret = ioctl(g_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1)
        goto fail;

    ret = ioctl(g_spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1)
        goto fail;

    return;
fail:
    printf("set spi mode fail\n");
    exit(-1);
}

static void spi_transfer(int fd, const uint8_t *tx, const uint8_t *rx, size_t len)
{
    struct spi_ioc_transfer tr =
    {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = len,
        .delay_usecs = 10,
        .speed_hz = speed,
        .bits_per_word = bits,
        .tx_nbits = 1,
        .rx_nbits = 1
    };
    int ret;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
    {
        printf("send spi message fail\n");
        exit(-1);
    }
}

static void tft_write_reg(uint8_t data)
{
    write(g_dc_fd, "0", 2);
    spi_transfer(g_spi_fd, &data, NULL, 1);
    write(g_dc_fd, "1", 2);
}

static void tft_write_data(const uint8_t *data, uint32_t len)
{
    spi_transfer(g_spi_fd, data, NULL, len);
}

void tft_init(void)
{
    uint8_t data[16];
    tft_gpio_init();
    spi_init();
    write(g_rst_fd, "0", 2);
    sleep(1);
    write(g_rst_fd, "1", 2);
    tft_write_reg(0x11);
    usleep(120 * 1000);
    tft_write_reg(0xB1);
    data[0] = 0x05;
    data[1] = 0x3C;
    data[2] = 0x3C;
    tft_write_data(data, 3);
    tft_write_reg(0xB2);
    data[0] = 0x05;
    data[1] = 0x3C;
    data[2] = 0x3C;
    tft_write_data(data, 3);
    tft_write_reg(0xB3);
    data[0] = 0x05;
    data[1] = 0x3C;
    data[2] = 0x3C;
    data[3] = 0x05;
    data[4] = 0x3C;
    data[5] = 0x3C;
    tft_write_data(data, 6);
    tft_write_reg(0xB4);
    data[0] = 0x03;
    tft_write_data(data, 1);
    tft_write_reg(0xC0);
    data[0] = 0xAB;
    data[1] = 0x0B;
    data[2] = 0x04;
    tft_write_data(data, 3);
    tft_write_reg(0xC1);
    data[0] = 0xC5;
    data[1] = 0xC2;
    data[2] = 0x0D;
    data[3] = 0x00;
    tft_write_data(data, 4);
    tft_write_reg(0xC3);
    data[0] = 0x8D;
    data[1] = 0x6A;
    tft_write_data(data, 2);
    tft_write_reg(0xC4);
    data[0] = 0x8D;
    data[1] = 0xEE;
    tft_write_data(data, 2);
    tft_write_reg(0xC5);
    data[0] = 0x0F;
    tft_write_data(data, 1);
    tft_write_reg(0xE0);
    data[0] = 0x07;
    data[1] = 0x0E;
    data[2] = 0x08;
    data[3] = 0x07;
    data[4] = 0x10;
    data[5] = 0x07;
    data[6] = 0x02;
    data[7] = 0x07;
    data[8] = 0x09;
    data[9] = 0x0F;
    data[10] = 0x25;
    data[11] = 0x36;
    data[12] = 0x00;
    data[13] = 0x08;
    data[14] = 0x04;
    data[15] = 0x10;
    tft_write_data(data, 16);
    tft_write_reg(0xE1);
    data[0] = 0x0A;
    data[1] = 0x0D;
    data[2] = 0x08;
    data[3] = 0x07;
    data[4] = 0x0F;
    data[5] = 0x07;
    data[6] = 0x02;
    data[7] = 0x07;
    data[8] = 0x09;
    data[9] = 0x0F;
    data[10] = 0x25;
    data[11] = 0x35;
    data[12] = 0x00;
    data[13] = 0x09;
    data[14] = 0x04;
    data[15] = 0x10;
    tft_write_data(data, 16);
    tft_write_reg(0xFC);
    data[0] = 0x80;
    tft_write_data(data, 1);
    tft_write_reg(0x3A);
    data[0] = 0x05;
    tft_write_data(data, 1);
    tft_write_reg(0x36);
    if (TFT_DISP_DIR == 0)data[0] = 0x08;
    else if (TFT_DISP_DIR == 1)data[0] = 0xC8;
    else if (TFT_DISP_DIR == 2)data[0] = 0x78;
    else data[0] = 0xA8;
    tft_write_data(data, 1);
    tft_write_reg(0x21);
    tft_write_reg(0x29);
    tft_write_reg(0x2A);
    data[0] = 0x00;
    data[1] = 0x1A;
    data[2] = 0x00;
    data[3] = 0x69;
    tft_write_data(data, 4);
    tft_write_reg(0x2B);
    data[0] = 0x00;
    data[1] = 0x01;
    data[2] = 0x00;
    data[3] = 0xA0;
    tft_write_data(data, 4);
    tft_write_reg(0x2C);
}

static void tft_addr_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint8_t data[2];
    if (TFT_DISP_DIR <= 1)
    {
        tft_write_reg(0x2a);
        data[0] = (x1 + 26) >> 8;
        data[1] = x1 + 26;
        data[2] = (x2 + 26) >> 8;
        data[3] = x2 + 26;
        tft_write_data(data, 4);
        tft_write_reg(0x2b);
        data[0] = (y1 + 1) >> 8;
        data[1] = y1 + 1;
        data[2] = (y2 + 1) >> 8;
        data[3] = y2 + 1;
        tft_write_data(data, 4);
        tft_write_reg(0x2c);
    }
    else
    {
        tft_write_reg(0x2a);
        data[0] = (x1 + 1) >> 8;
        data[1] = x1 + 1;
        data[2] = (x2 + 1) >> 8;
        data[3] = x2 + 1;
        tft_write_data(data, 4);
        tft_write_reg(0x2b);
        data[0] = (y1 + 26) >> 8;
        data[1] = y1 + 26;
        data[2] = (y2 + 26) >> 8;
        data[3] = y2 + 26;
        tft_write_data(data, 4);
        tft_write_reg(0x2c);
    }
}

void tft_fill(uint16_t x_s, uint16_t y_s, uint16_t x_e, uint16_t y_e, uint16_t color)
{
    for (uint16_t j = 0; j < y_e - y_s + 1; j++)
    {
        for (uint16_t i = 0; i < x_e - x_s + 1; i++)
        {
            g_disp_buff[y_s + j][x_s + i] = color;
        }
    }
}

void tft_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    g_disp_buff[y][x] = color;
}

void tft_refresh(void)
{
    tft_addr_set(0, 0, TFT_W - 1, TFT_H - 1);
    for (uint8_t i = 0; i < TFT_H; i += 10)
    {
        tft_write_data((const uint8_t *)(&g_disp_buff[i][0]), TFT_W * 20);

    }
}

void tft_show_char(uint16_t x, uint16_t y, uint8_t num,
                   uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
    uint8_t temp, sizex, t, m = 0;
    uint16_t i, TypefaceNum;
    uint16_t x0 = x;
    sizex = size / 2;
    TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * size;
    num = num - ' ';
    for (i = 0; i < TypefaceNum; i++)
    {
        if (size == 12)temp = ascii_1206[num][i];
        else if (size == 16)temp = ascii_1608[num][i];
        else if (size == 24)temp = ascii_2412[num][i];
        else if (size == 32)temp = ascii_3216[num][i];
        else return;
        for (t = 0; t < 8; t++)
        {
            if (!mode)
            {
                if (temp & (0x01 << t))tft_draw_point(x + sizex - 1 + i, y + size - 1, fc);
                else tft_draw_point(x + sizex - 1 + i, y + size - 1, bc);
                m++;
                if (m % sizex == 0)
                {
                    m = 0;
                    break;
                }
            }
            else
            {
                if (temp & (0x01 << t))tft_draw_point(x, y, fc);
                x++;
                if ((x - x0) == sizex)
                {
                    x = x0;
                    y++;
                    break;
                }
            }
        }
    }
}

void tft_show_string(uint16_t x, uint16_t y, const char *str,
                     uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
    while ((*str != '\0') && (*str != '\n'))
    {
        tft_show_char(x, y, *str, fc, bc, size, mode);
        x += size / 2;
        str++;
    }
}

void tft_show_image(uint16_t x, uint16_t y, uint16_t width, uint16_t high, const uint16_t *image)
{
    for (uint16_t i = 0; i < high; i++)
    {
        for (uint16_t j = 0; j < width; j++)
        {
            tft_fill(x + j, y + i, x + j, y + i, image[j * high + i]);
        }
    }

}