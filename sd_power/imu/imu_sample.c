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

int main(void)
{
    uint8_t accel_raw_x[2], accel_raw_y[2], accel_raw_z[2];
    uint8_t gyro_raw_x[2], gyro_raw_y[2], gyro_raw_z[2];
    uint8_t temp_raw[2];
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float temp;

    imu_init();
    while (1)
    {
        imu_i2c_read_regs(0x000B, accel_raw_x, 2);
        imu_i2c_read_regs(0x000D, accel_raw_y, 2);
        imu_i2c_read_regs(0x000F, accel_raw_z, 2);
        imu_i2c_read_regs(0x0011, gyro_raw_x, 2);
        imu_i2c_read_regs(0x0013, gyro_raw_y, 2);
        imu_i2c_read_regs(0x0015, gyro_raw_z, 2);
        imu_i2c_read_regs(0x0009, temp_raw, 2);

        accel_x = (float)(*((int16_t *)accel_raw_x)) / 16384.0;
        accel_y = (float)(*((int16_t *)accel_raw_y)) / 16384.0;
        accel_z = (float)(*((int16_t *)accel_raw_z)) / 16384.0;

        gyro_x = (float)(*((int16_t *)gyro_raw_x)) / 131.072;
        gyro_y = (float)(*((int16_t *)gyro_raw_y)) / 131.072;
        gyro_z = (float)(*((int16_t *)gyro_raw_z)) / 131.072;

        temp = (float)(*((int16_t *)temp_raw)) / 128.0 + 25.0;

        printf("accel: x:%f g/s, y:%f g/s, z:%f g/s\n", accel_x, accel_y, accel_z);
        printf("gyro: x:%f LSB, y:%f LSB, z:%f LSB\n", gyro_x, gyro_y, gyro_z);
        printf("temperature: %f °C\n", temp);
        sleep(1);
    }
}