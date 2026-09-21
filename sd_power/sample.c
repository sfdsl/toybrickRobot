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
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <ifaddrs.h>

#include "imu/imu.h"
#include "key_led/key_led.h"
#include "tft/tft.h"
#include "tft/font.h"

typedef struct
{
    char name[20];
    unsigned int user;
    unsigned int nice;
    unsigned int system;
    unsigned int idle;
    unsigned int iowait;
    unsigned int irq;
    unsigned int softirq;
}cpu_occupy_t;


void disp_sys(void)
{
    FILE *fd;
    char buff[256];
    cpu_occupy_t cpu_occupy1[9];
    cpu_occupy_t cpu_occupy2[9];
    uint8_t str[9][20];
    double od, nd;
    double id, sd;
    double cpu_use ;
    struct input_event key;
    uint8_t disp = KEY1_CODE;
    while(1)
    {
        fd = fopen ("/proc/stat", "r");
        if(fd == NULL)
        {
            printf("open fail\n");
            exit (-1);
        }
        for (uint8_t i = 0; i < 9; i++)
        {
            fgets (buff, sizeof(buff), fd);
            sscanf(buff, "%s %u %u %u %u %u %u %u", cpu_occupy1[i].name, &cpu_occupy1[i].user, 
                &cpu_occupy1[i].nice,&cpu_occupy1[i].system, &cpu_occupy1[i].idle ,&cpu_occupy1[i].iowait,
                &cpu_occupy1[i].irq,&cpu_occupy1[i].softirq);
            printf("%s %u %u %u %u %u %u %u\n", cpu_occupy1[i].name, cpu_occupy1[i].user, 
                cpu_occupy1[i].nice,cpu_occupy1[i].system, cpu_occupy1[i].idle ,cpu_occupy1[i].iowait,
                cpu_occupy1[i].irq,cpu_occupy1[i].softirq);
        }
        fclose(fd);
        usleep(1000);
        fd = fopen ("/proc/stat", "r");
        if(fd == NULL)
        {
            printf("open fail\n");
            exit (-1);
        }
        for (uint8_t i = 0; i < 9; i++)
        {
            fgets (buff, sizeof(buff), fd);
            sscanf(buff, "%s %u %u %u %u %u %u %u", cpu_occupy2[i].name, &cpu_occupy2[i].user, 
                &cpu_occupy2[i].nice,&cpu_occupy2[i].system, &cpu_occupy2[i].idle ,&cpu_occupy2[i].iowait,
                &cpu_occupy2[i].irq,&cpu_occupy2[i].softirq);
            printf("%s %u %u %u %u %u %u %u\n", cpu_occupy2[i].name, cpu_occupy2[i].user, 
                cpu_occupy2[i].nice,cpu_occupy2[i].system, cpu_occupy2[i].idle ,cpu_occupy2[i].iowait,
                cpu_occupy2[i].irq,cpu_occupy2[i].softirq);
            od = (double)(cpu_occupy1[i].user + cpu_occupy1[i].nice + cpu_occupy1[i].system
                        + cpu_occupy1[i].idle + cpu_occupy1[i].softirq + cpu_occupy1[i].iowait + cpu_occupy1[i].irq);
            nd = (double) (cpu_occupy2[i].user + cpu_occupy2[i].nice + cpu_occupy2[i].system
                        + cpu_occupy2[i].idle + cpu_occupy2[i].softirq + cpu_occupy2[i].iowait + cpu_occupy2[i].irq);
        
            id = (double) (cpu_occupy2[i].idle);
            sd = (double) (cpu_occupy1[i].idle);
            if((nd-od) != 0)
                cpu_use =100.0 - ((id-sd))/(nd-od)*100.00;
            else 
                cpu_use = 0;
            if (i == 0)
            {
                snprintf(str[i], 20, "CPU:%07.3f%%\n", cpu_use);
                printf("%s", str);
            }
            else
            {
                snprintf(str[i], 20, "CPU%d:%07.3f%%\n", i-1, cpu_use);
                printf("%s", str);
            }
        }
        read_key(&key);
        if (key.type != 0)
        {
            printf("Event: time %ld.%ld, type %d, code %d,value %d\n",
                   key.time.tv_sec, key.time.tv_usec,
                   key.type,
                   key.code,
                   key.value);
            if (key.value == 0)
            {
                disp = key.code;
            }
        }
        tft_fill(0, 0, TFT_W - 1, TFT_H - 1, 0x0000);
        switch (disp)
        {
        case KEY1_CODE:
        {
            tft_show_string(0, 0, str[1], 0xFFFF, 0x0000, 16, 1);
            tft_show_string(0, 20, str[2], 0xFFFF, 0x0000, 16, 1);
            tft_show_string(0, 40, str[3], 0xFFFF, 0x0000, 16, 1);
            tft_show_string(0, 60, str[4], 0xFFFF, 0x0000, 16, 1);
        }
        break;
        case KEY2_CODE:
        {
            tft_show_string(0, 0, str[5], 0xFFFF, 0x0000, 16, 1);
            tft_show_string(0, 20, str[6], 0xFFFF, 0x0000, 16, 1);
            tft_show_string(0, 40, str[7], 0xFFFF, 0x0000, 16, 1);
            tft_show_string(0, 60, str[8], 0xFFFF, 0x0000, 16, 1);
        }
        break;
        case KEY3_CODE:
            return;
        default:
            break;
        }
        tft_refresh();
    }
}

void disp_ip(void)
{
    struct ifaddrs *ifaddr, *p;
    int family, s;
    uint8_t buff[40];
    struct input_event key;
    if (getifaddrs(&ifaddr) == -1)
    {
        printf("get ifaddrs fail\n");
        exit(-1);
    }
    p = ifaddr;
    while(1)
    {
        if (p == NULL)
        {
            p = ifaddr;
        }
        if (p->ifa_addr == NULL)
        {
            p = p->ifa_next;
            continue;
        }
        family = p->ifa_addr->sa_family;
        p = p->ifa_next;
        if (family == AF_INET)
        {
            printf("interfac: %s, ip: %s\n", p->ifa_name, inet_ntoa(((struct sockaddr_in*)p->ifa_addr)->sin_addr));
            tft_fill(0, 0, TFT_W - 1, TFT_H - 1, 0x0000);
            tft_show_string(0, 0, "Toybrick", 0xFFFF, 0x0000, 16, 1);
            tft_show_string(0, 20, "TB-RK3588SD", 0xFFFF, 0x0000, 16, 1);

            snprintf(buff, 40, "interfac: %s", p->ifa_name);
            tft_show_string(0, 40, buff, 0xFFFF, 0x0000, 16, 1);

            snprintf(buff, 40, "ip:  %s", inet_ntoa(((struct sockaddr_in*)p->ifa_addr)->sin_addr));
            tft_show_string(0, 60, buff, 0xFFFF, 0x0000, 16, 1);
            tft_refresh();
            while (1)
            {
                read_key(&key);
                if (key.type != 0)
                {
                    printf("Event: time %ld.%ld, type %d, code %d,value %d\n",
                        key.time.tv_sec, key.time.tv_usec,
                        key.type,
                        key.code,
                        key.value);
                    if (key.value == 0)
                    {
                        if (key.code == KEY3_CODE)
                        {
                            freeifaddrs(ifaddr);
                            return;
                        }
                        else
                            break;
                    }
                }
            }
        }
        else
            continue;
    }
}

void disp_imu(void)
{
    uint8_t accel_raw_x[2], accel_raw_y[2], accel_raw_z[2];
    uint8_t gyro_raw_x[2], gyro_raw_y[2], gyro_raw_z[2];
    uint8_t temp_raw[2];
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float temp;
    char buff[40];
    struct input_event key;
    uint8_t disp = KEY1_CODE;
    while (1)
    {
        read_key(&key);
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

        tft_fill(0, 0, TFT_W - 1, TFT_H - 1, 0x0000);
        if (key.type != 0)
        {
            printf("Event: time %ld.%ld, type %d, code %d,value %d\n",
                   key.time.tv_sec, key.time.tv_usec,
                   key.type,
                   key.code,
                   key.value);
            if (key.value == 0)
            {
                disp = key.code;
            }
        }
        switch (disp)
        {
        case KEY1_CODE:
        {
            snprintf(buff, 40, "accel: x:%+0.4f g/s\n", accel_x);
            tft_show_string(0, 0, buff, 0xFFFF, 0x0000, 16, 1);
            printf("%s", buff);
            snprintf(buff, 40, "       y:%+0.4f g/s\n", accel_y);
            tft_show_string(0, 20, buff, 0xFFFF, 0x0000, 16, 1);
            printf("%s", buff);
            snprintf(buff, 40, "       z:%+0.4f g/s\n", accel_z);
            tft_show_string(0, 40, buff, 0xFFFF, 0x0000, 16, 1);
            printf("%s", buff);
            snprintf(buff, 40, "temp:  %+f\n", temp);
            tft_show_string(0, 60, buff, 0xFFFF, 0x0000, 16, 1);
            printf("temp:  %+0.3f °C\n", temp);
            tft_refresh();
        }
        break;
        case KEY2_CODE:
        {
            snprintf(buff, 40, "gyro: x:%+08.3f LSB\n", gyro_x);
            tft_show_string(0, 0, buff, 0xFFFF, 0x0000, 16, 1);
            printf("%s", buff);
            snprintf(buff, 40, "      y:%+08.3f LSB\n", gyro_y);
            tft_show_string(0, 20, buff, 0xFFFF, 0x0000, 16, 1);
            printf("%s", buff);
            snprintf(buff, 40, "      z:%+08.3f LSB\n", gyro_z);
            tft_show_string(0, 40, buff, 0xFFFF, 0x0000, 16, 1);
            printf("%s", buff);
            snprintf(buff, 40, "temp: %+f\n", temp);
            tft_show_string(0, 60, buff, 0xFFFF, 0x0000, 16, 1);
            printf("temp:  %+0.3f °C\n", temp);
            tft_refresh();
        }
        break;
        case KEY3_CODE:
            return;
        default:
            break;
        }
    }
}

int main(void)
{
    struct input_event key;
    imu_init();
    key_led_init();
    tft_init();
    tft_show_image(0, 0, 160, 76, image_toybrick);
    tft_refresh();
    while (1)
    {
        read_key(&key);
        if (key.type != 0)
        {
            printf("Event: time %ld.%ld, type %d, code %d,value %d\n",
                   key.time.tv_sec, key.time.tv_usec,
                   key.type,
                   key.code,
                   key.value);
            if (key.value == 0)
            {
                if (key.code == KEY1_CODE)
                {
                    led_r_set_light(100);
                    led_g_set_light(0);
                    led_b_set_light(0);
                    disp_ip();
                    tft_show_image(0, 0, 160, 76, image_toybrick);
                    tft_refresh();
                }
                else if (key.code == KEY2_CODE)
                {
                    led_r_set_light(0);
                    led_g_set_light(100);
                    led_b_set_light(0);
                    disp_sys();
                    tft_show_image(0, 0, 160, 76, image_toybrick);
                    tft_refresh();
                }
                else if (key.code == KEY3_CODE)
                {
                    led_r_set_light(0);
                    led_g_set_light(0);
                    led_b_set_light(100);
                    disp_imu();
                    tft_show_image(0, 0, 160, 76, image_toybrick);
                    tft_refresh();
                }
            }
        }
    }
}