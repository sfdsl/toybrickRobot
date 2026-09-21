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
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "imu.h"

static int g_i2c_fd;
static inline int imu_switch_on_mclk(void)
{
    uint8_t buf;
    imu_i2c_read_regs(0x1F, &buf, 1);
    if (!(buf & 0x10))
    {
        uint32_t timeout = 0;
        buf |= 0x10;
        imu_i2c_write_regs(0x1F, &buf, 1);

        do
        {
            imu_i2c_read_regs(0x00, &buf, 1);
            timeout ++;
            if (timeout > 0xFFFFFFFE)
            {
                printf("imu switch on mclk timeout\n");
                exit(-1);
            }
        }
        while (!(buf & 0x08));
    }
    return 0;
}

static inline int imu_switch_off_mclk(void)
{
    uint8_t buf;
    imu_i2c_read_regs(0x1F, &buf, 1);
    if (buf & 0x10)
    {
        buf &= ~0x10;
        imu_i2c_write_regs(0x1F, &buf, 1);
    }
    return 0;
}

int imu_i2c_write_regs(uint16_t reg, uint8_t *buf, uint8_t len)
{
    if (reg & 0xFF00)
    {
        uint8_t w_buf;
        uint8_t blk_sel = (reg & 0xFF00) >> 8;
        if (blk_sel == 0x10)
        {
            blk_sel = 0x00;
        }
        if (imu_switch_on_mclk() == -1)
        {
            return  -1;
        }
        if (blk_sel)
        {
            w_buf = blk_sel;
            imu_i2c_write_regs(0x79, &w_buf, 1);
        }
        w_buf = reg & 0x00FF;
        imu_i2c_write_regs(0x7A, &w_buf, 1);
        for (uint8_t i = 0; i < len; i++)
        {
            w_buf = buf[i];
            imu_i2c_write_regs(0x7B, &w_buf, 1);
        }
        if (blk_sel)
        {
            w_buf = 0x00;
            imu_i2c_write_regs(0x79, &w_buf, 1);
        }

        imu_switch_off_mclk();
    }
    else
    {
        uint8_t *tmp = (uint8_t *)calloc(len + 1, sizeof(uint8_t));
        if (tmp == NULL)
        {
            printf("malloc i2c buff fail\n");
            exit(-1);
        }
        tmp[0] = (uint8_t)reg;
        memcpy(&tmp[1], buf, len);
        if (write(g_i2c_fd, tmp, len + 1) != len + 1)
        {
            printf("i2c write error\n");
            exit(-1);
        }
        free(tmp);
    }
    return 0;
}

int imu_i2c_read_regs(uint16_t reg, uint8_t *buf, uint8_t len)
{
    if (reg & 0xFF00)
    {
        if (imu_switch_on_mclk() == -1)
        {
            return  -1;
        }
        for (uint8_t i = 0; i < len; i++)
        {
            uint8_t r_buf;
            uint16_t reg_addr = (reg + i) & 0xFF00;
            uint8_t blk_sel = reg_addr >> 8;
            if (blk_sel == 0x10)
            {
                blk_sel = 0x00;
            }

            if (blk_sel)
            {
                imu_i2c_write_regs(0x7C, &blk_sel, 1);
            }
            r_buf = reg_addr & 0x00FF;
            imu_i2c_write_regs(0x7D, &r_buf, 1);
            imu_i2c_read_regs(0x7E, &buf[i], 1);
            if (blk_sel)
            {
                r_buf = 0x00;
                imu_i2c_write_regs(0x7C, &r_buf, 1);
            }
        }

        imu_switch_off_mclk();
    }
    else
    {
        if (write(g_i2c_fd, &reg, 1) != 1)
        {
            printf("i2c write error\n");
            exit(-1);
        }
        if (read(g_i2c_fd, buf, len) != len)
        {
            printf("i2c read error\n");
            exit(-1);
        }
    }

    return 0;
}

int imu_init(void)
{
    uint8_t reg;
    uint8_t buf;
    g_i2c_fd = open(IMU_I2C, O_RDWR);
    if (g_i2c_fd <= 0)
    {
        printf("open %s failed\n", IMU_I2C);
        exit(-1);
    }
    if (ioctl(g_i2c_fd, I2C_SLAVE, IMU_I2C_ADDR) < 0)
    {
        printf("i2c ioctl error");
        exit(-1);
    }
    reg = 0x75;
    imu_i2c_read_regs(reg, &buf, 1);
    if (buf != IMU_WHOAMI)
    {
        printf("get WHOAMI faild: 0x%02X\n", buf);
        exit(-1);
    }

    buf = 0x41;
    imu_i2c_write_regs(0x36, &buf, 1);

    buf = 0xB0;
    imu_i2c_write_regs(0x35, &buf, 1);

    usleep(3000);
    buf = 0x10;
    imu_i2c_write_regs(0x02, &buf, 1);
    buf = 0x41;
    imu_i2c_write_regs(0x36, &buf, 1);

    buf = 0xB0;
    imu_i2c_write_regs(0x35, &buf, 1);
    usleep(1000);

    imu_i2c_read_regs(0x35, &buf, 1);
    buf &= ~0x10;
    imu_i2c_write_regs(0x35, &buf, 1);
    imu_i2c_read_regs(0x3A, &buf, 1);
    if (!(buf & 0x10))
    {
        printf("soft reset faild\n");
        exit(-1);
    }

    buf = 0x66;
    imu_i2c_write_regs(0x21, &buf, 1);

    buf = 0x66;
    imu_i2c_write_regs(0x20, &buf, 1);

    buf = 0x30;
    imu_i2c_write_regs(0x22, &buf, 1);

    buf = 0x03;
    imu_i2c_write_regs(0x06, &buf, 1);

    imu_switch_on_mclk();
    buf = 0x00;
    imu_i2c_write_regs(0x1001, &buf, 1);
    imu_switch_off_mclk();
    imu_i2c_read_regs(0x1F, &buf, 1);
    buf |= 0x8F;
    imu_i2c_write_regs(0x1F, &buf, 1);
}