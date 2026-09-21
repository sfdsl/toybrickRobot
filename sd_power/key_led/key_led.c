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
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "key_led.h"

static int g_key_fd;
static int g_led_r_fd;
static int g_led_g_fd;
static int g_led_b_fd;

void key_led_init(void)
{
    g_key_fd = open(KEY_EVENT, O_RDONLY | O_NONBLOCK);
    if (g_key_fd < 0)
    {
        printf("Fail to open device:%s\n", KEY_EVENT);
        exit(1);
    }

    g_led_r_fd = open(LED_R, O_RDWR);
    if (g_led_r_fd < 0)
    {
        printf("Fail to open device:%s\n", LED_R);
        exit(1);
    }

    g_led_g_fd = open(LED_G, O_RDWR);
    if (g_led_g_fd < 0)
    {
        printf("Fail to open device:%s\n", LED_G);
        exit(1);
    }

    g_led_b_fd = open(LED_B, O_RDWR);
    if (g_led_b_fd < 0)
    {
        printf("Fail to open device:%s\n", LED_B);
        exit(1);
    }
}

void led_r_set_light(uint8_t light)
{
    char buff[4];
    snprintf(buff, 4, "%d", light);
    write(g_led_r_fd, buff, strlen(buff));
}

void led_g_set_light(uint8_t light)
{
    char buff[4];
    snprintf(buff, 4, "%d", light);
    write(g_led_g_fd, buff, strlen(buff));
}

void led_b_set_light(uint8_t light)
{
    char buff[4];
    snprintf(buff, 4, "%d", light);
    write(g_led_b_fd, buff, strlen(buff));
}

void read_key(struct input_event *event)
{
    read(g_key_fd, event, sizeof(struct input_event));
}