/*
Copyright (c) 2015, Intel Corporation. All rights reserved.
*Redistribution and use in source and binary forms, with or without
*modification, are permitted provided that the following conditions are met:
*
*1. Redistributions of source code must retain the above copyright notice,
*this list of conditions and the following disclaimer.
*
*2. Redistributions in binary form must reproduce the above copyright notice,
*this list of conditions and the following disclaimer in the documentation
*and/or other materials provided with the distribution.
*
*3. Neither the name of the copyright holder nor the names of its contributors
*may be used to endorse or promote products derived from this software without
*specific prior written permission.
*
*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
*AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
*ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
*LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
*ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef _CAMERA_H_
#define _CAMERA_H_
#include "uio.h"
#include "list.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <linux/videodev2.h>

#define YUV_ENABLE
#define WIDTH	640	
#define HEIGHT	480 


typedef struct {
    uint8_t* start;
    size_t length;
} buffer_t;

typedef struct {
    int fd;
    uint32_t width;
    uint32_t height;
    size_t buffer_count;
    buffer_t* buffers;
    buffer_t head;
} camera_t;



//int capture_frame();
#ifndef YUV_ENABLE
int read_frame (camera_t *camera);
#endif
char *yuyv2raw(unsigned char * yuv, int pixels);
uint8_t *yuyv2rgb_gray(unsigned char * yuv, int pixels);
int camera_frame(camera_t* camera, struct timeval timeout);
void camera_close(camera_t* camera);
void camera_finish(camera_t* camera);
void camera_start(camera_t* camera);
void camera_stop(camera_t* camera);
void camera_init(camera_t* camera) ;
camera_t* camera_open(const char * device, uint32_t width, uint32_t height);



#endif
