# Copyright (c) 2023 Rockchip Electronics Co., Ltd.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

#     (1) Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer. 

#     (2) Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in
#     the documentation and/or other materials provided with the
#     distribution.  
    
#     (3)The name of the author may not be used to
#     endorse or promote products derived from this software without
#     specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
# IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE. 
# Change Logs:
# Date           Author           Notes
# 2023-07-14     Sisyphean Zhou   first implementation

export CC             = gcc           
export AS             = as
export LD             = ld
export OBJCOPY        = objcopy
export OBJDUMP        = objdump
export SIZE           = size

TOP=$(shell pwd)

INC_FLAGS= -I $(TOP)/imu \
           -I $(TOP)/key_led \
           -I $(TOP)/tft \

IMU_SRC=$(shell ls imu/*.c)
KEY_LED_SRC=$(shell ls key_led/*.c)
TFT_SRC=$(shell ls tft/*.c)
SAMPLE_SRC=$(shell find . -name "*.c" | grep -v "_sample.c")

.PHONY: all imu key_led tft sample clean

all: imu key_led tft sample

imu: $(IMU_SRC)
	$(CC) $(IMU_SRC) $(INC_FLAGS) -lm -o imu_sample

key_led: $(KEY_LED_SRC)
	$(CC) $(KEY_LED_SRC) $(INC_FLAGS) -lm -o key_led_sample

tft: $(TFT_SRC)
	$(CC) $(TFT_SRC) $(INC_FLAGS) -lm -o tft_sample

sample: $(SAMPLE_SRC)
	$(CC) $(SAMPLE_SRC) $(INC_FLAGS) -lm -o sample

clean:
	rm -rf *sample

