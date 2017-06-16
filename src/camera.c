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
/*
 * capturing from UVC cam
 * requires: libjpeg-dev
 * build: gcc -std=c99 capture.c -ljpeg -o capture
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <asm/types.h>
#include <linux/videodev2.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <jpeglib.h>
#include "jpeglib.h"
#include "camera.h"

void quit(const char * msg)
{
    fprintf(stderr, "[%s] %d: %s\n", msg, errno, strerror(errno));
    printf("[%s] %d: %s\n", msg, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

int xioctl(int fd, int request, void* arg)
{
    int i =0;
    for (i = 0; i < 100; i++) {
	int r = ioctl(fd, request, arg);
	if (r != -1 || errno != EINTR) return r;
    }
    return -1;
}

camera_t* camera_open(const char * device, uint32_t width, uint32_t height)
{
    int fd = open(device, O_RDWR | O_NONBLOCK, 0);
    if (fd == -1) quit("open");
#if 1
    /*获取驱动信息*/
    /* 得到描述摄像头信息的结构体 */
    struct v4l2_capability cap;
    int rel = ioctl(fd, VIDIOC_QUERYCAP, &cap);
    if ( rel == -1) {
        printf("error : %s\n", strerror(errno));
	return 0;
    }
    /* 判断改设备支不支持捕获图像和流输出功能 */
    if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE)
        printf("it's camer!\n");
    else {
        printf("it's not a camer!\n");
	return 0;
    }
    if ((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING)
        printf("it's stream device!\n");
    else {
        printf("it's not a stream device!\n");
        return 0;
    }

    printf("Driver Name : %s\n\
    Card Name : %s\nBus info : %s\n\
    Driver Version : %u.%u.%u\n ",\
    cap.driver, cap.card, cap.bus_info,\
     (cap.version>>16)&0xff, (cap.version>>8)&0xff, (cap.version)&0xff);

    /* 得到摄像头采集图像的格式信息 */
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rel = ioctl(fd, VIDIOC_G_FMT, &fmt);
    if (rel == -1) {
        printf("get fmt failed!\n");
	return 0;
    }

    printf("width : %d  height : %d\n\
    pixelformat : %d\n\
    V4L2_PIX_FMT_YUYV: %d\n\
    field : %d\n\
    bytesperline : %d\n\
    sizeimage : %d\n\
    colorspace : %d\n\
    priv : %d\n",\
    fmt.fmt.pix.width,\
     fmt.fmt.pix.height,\
    fmt.fmt.pix.pixelformat,\
    V4L2_PIX_FMT_YUYV,\
     fmt.fmt.pix.field, \
     fmt.fmt.pix.bytesperline, \
     fmt.fmt.pix.sizeimage, \
     fmt.fmt.pix.colorspace, \
     fmt.fmt.pix.priv);
    /* 得到摄像头所支持的所有格式 */
    struct v4l2_fmtdesc fmtdesc;
    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf(" Support format : \n");
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1) {
        printf("\t%d.%s\n", fmtdesc.index+1, fmtdesc.description);
        fmtdesc.index++;
    }

#endif

    camera_t* camera = malloc(sizeof (camera_t));
    camera->fd = fd;
    camera->width = width;
    camera->height = height;
    camera->buffer_count = 0;
    camera->buffers = NULL;
    camera->head.length = 0;
    camera->head.start = NULL;
    return camera;
}


void camera_init(camera_t* camera) {
    struct v4l2_capability cap;
    if (xioctl(camera->fd, VIDIOC_QUERYCAP, &cap) == -1) quit("VIDIOC_QUERYCAP");
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) quit("no capture");
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) quit("no streaming");

    struct v4l2_cropcap cropcap;
    memset(&cropcap, 0, sizeof cropcap);
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(camera->fd, VIDIOC_CROPCAP, &cropcap) == 0) {
	struct v4l2_crop crop;
	crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	crop.c = cropcap.defrect;
	if (xioctl(camera->fd, VIDIOC_S_CROP, &crop) == -1) {
		printf("[%s]:crop not supported\n",__func__);
	    // cropping not supported
	}
    }

    struct v4l2_format format;
    memset(&format, 0, sizeof format);
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = camera->width;
    format.fmt.pix.height = camera->height;
#ifdef YUV_ENABLE
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
#else
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
#endif
    format.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(camera->fd, VIDIOC_S_FMT, &format) == -1) quit("VIDIOC_S_FMT");

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof req);
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(camera->fd, VIDIOC_REQBUFS, &req) == -1) quit("VIDIOC_REQBUFS");
    camera->buffer_count = req.count;
    camera->buffers = calloc(req.count, sizeof (buffer_t));

    size_t buf_max = 0;
    size_t i =0;
    for (i = 0; i < camera->buffer_count; i++) {
	struct v4l2_buffer buf;
	memset(&buf, 0, sizeof buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = i;
	if (xioctl(camera->fd, VIDIOC_QUERYBUF, &buf) == -1)
	    quit("VIDIOC_QUERYBUF");
	if (buf.length > buf_max) buf_max = buf.length;
	camera->buffers[i].length = buf.length;
	camera->buffers[i].start =
	    mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
		    camera->fd, buf.m.offset);
	if (camera->buffers[i].start == MAP_FAILED) quit("mmap");
    }
    camera->head.start = malloc(buf_max);
}


void camera_start(camera_t* camera)
{
    size_t i =0;
    for (i = 0; i < camera->buffer_count; i++) {
	struct v4l2_buffer buf;
	memset(&buf, 0, sizeof buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = i;
	if (xioctl(camera->fd, VIDIOC_QBUF, &buf) == -1) quit("VIDIOC_QBUF");
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(camera->fd, VIDIOC_STREAMON, &type) == -1)
	quit("VIDIOC_STREAMON");
}

void camera_stop(camera_t* camera)
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(camera->fd, VIDIOC_STREAMOFF, &type) == -1)
	quit("VIDIOC_STREAMOFF");
}

void camera_finish(camera_t* camera)
{
    size_t i =0;
    for (i = 0; i < camera->buffer_count; i++) {
	munmap(camera->buffers[i].start, camera->buffers[i].length);
    }
    free(camera->buffers);
    camera->buffer_count = 0;
    camera->buffers = NULL;
    free(camera->head.start);
    camera->head.length = 0;
    camera->head.start = NULL;
}

void camera_close(camera_t* camera)
{
    if (close(camera->fd) == -1) quit("close");
    free(camera);
}


int camera_capture(camera_t* camera)
{
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (xioctl(camera->fd, VIDIOC_DQBUF, &buf) == -1) {
	printf("[%s]:error\n",__func__);
	return FALSE;
    }
    //memcpy(camera->head.start, camera->buffers[buf.index].start, buf.bytesused);
    memcpy(camera->head.start, camera->buffers[buf.index].start, camera->buffers[buf.index].length);
    printf("[%s]:buf.bytesused=%d\n",__func__,buf.bytesused);
    printf("[%s]:buf.len=%zd,index=%d\n",__func__,camera->buffers[buf.index].length,buf.index);
    camera->head.length = buf.bytesused;
    if (xioctl(camera->fd, VIDIOC_QBUF, &buf) == -1) return FALSE;
    return TRUE;
}

int camera_frame(camera_t* camera, struct timeval timeout) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(camera->fd, &fds);
    int r = select(camera->fd + 1, &fds, 0, 0, &timeout);
    printf("[%s]:data is coming\n",__func__);
    if (r == -1) quit("select");
    if (r == 0) {
	printf("[%s]:error\n",__func__);
	return FALSE;
    }
    return camera_capture(camera);
}


void jpeg(FILE* dest, uint8_t* rgb, uint32_t width, uint32_t height, int quality)
{
    JSAMPARRAY image;
    size_t i =0;
    size_t j =0;
    image = calloc(height, sizeof (JSAMPROW));
    for (i = 0; i < height; i++) {
	image[i] = calloc(width * 3, sizeof (JSAMPLE));
	for (j = 0; j < width; j++) {
	    image[i][j * 3 + 0] = rgb[(i * width + j) * 3 + 0];
	    image[i][j * 3 + 1] = rgb[(i * width + j) * 3 + 1];
	    image[i][j * 3 + 2] = rgb[(i * width + j) * 3 + 2];
	}
    }

    struct jpeg_compress_struct compress;
    struct jpeg_error_mgr error;
    compress.err = jpeg_std_error(&error);
    jpeg_create_compress(&compress);
    jpeg_stdio_dest(&compress, dest);

    compress.image_width = width;
    compress.image_height = height;
    compress.input_components = 3;
    compress.in_color_space = JCS_RGB;
    jpeg_set_defaults(&compress);
    jpeg_set_quality(&compress, quality, TRUE);
    jpeg_start_compress(&compress, TRUE);
    jpeg_write_scanlines(&compress, image, height);
    jpeg_finish_compress(&compress);
    jpeg_destroy_compress(&compress);

    for (i = 0; i < height; i++) {
	free(image[i]);
    }
    free(image);
}


int minmax(int min, int v, int max)
{
    return (v < min) ? min : (max < v) ? max : v;
}

/*yuyv is 16bit,we need to transform to 32bit rgb*/
uint8_t *yuyv2rgb_gray(unsigned char * yuv, int pixels)
{
    uint8_t* buf= calloc(pixels * 3, sizeof (uint8_t));
    int count;
    int y0,u0,y1,v1;
    for (count=0; count<pixels/2; count++) {
	/*gray pic*/
        y0 = yuv[count*4+0];
        u0 = 0x80;
        y1 = yuv[count*4+2];
        v1 = 0x80;

        buf[count*6+0] = minmax(0,(1.164*(y0-16) + 2.018*(u0-128)),255); //b
        buf[count*6+1] = minmax(0,1.164*(y0-16) - 0.380*(u0-128) + 0.813*(v1-128),255); //g
        buf[count*6+2] = minmax(0,1.164*(y0-16) + 1.159*(v1-128),255); //r
        //buf[count*8+3] = 0; //透明度

        buf[count*6+3] = minmax(0,1.164*(y1-16) + 2.018*(u0-128),255); //b
        buf[count*6+4] = minmax(0,1.164*(y1-16) - 0.380*(u0-128) + 0.813*(v1-128),255); //g
        buf[count*6+5] = minmax(0,1.164*(y1-16) + 1.159*(v1-128),255); //r
        //buf[count*8+7] = 0; //透明度
    }
    return buf;
}

uint8_t* yuyv2rgb(uint8_t* yuyv, uint32_t width, uint32_t height)
{
    uint8_t* rgb = calloc(width * height * 3, sizeof (uint8_t));
    size_t i = 0;
    size_t j = 0;
    for (i = 0; i < height; i++) {
	for (j = 0; j < width; j += 2) {
	    size_t index = i * width + j;
	    int y0 = yuyv[index * 2 + 0] << 8;
	    int u = yuyv[index * 2 + 1] - 128;
	    int y1 = yuyv[index * 2 + 2] << 8;
	    int v = yuyv[index * 2 + 3] - 128;
	    rgb[index * 3 + 0] = minmax(0, (y0 + 359 * v) >> 8, 255);//b
	    rgb[index * 3 + 1] = minmax(0, (y0 + 88 * v - 183 * u) >> 8, 255);//g
	    rgb[index * 3 + 2] = minmax(0, (y0 + 454 * u) >> 8, 255);//r
	    rgb[index * 3 + 3] = minmax(0, (y1 + 359 * v) >> 8, 255);
	    rgb[index * 3 + 4] = minmax(0, (y1 + 88 * v - 183 * u) >> 8, 255);
	    rgb[index * 3 + 5] = minmax(0, (y1 + 454 * u) >> 8, 255);
	}
    }
    return rgb;
}

/*yuyv is 16bit,we need to transform to 32bit raw data*/
char *yuyv2raw(unsigned char * yuv, int pixels)
{
    char* buf= calloc(pixels * 4, sizeof (char));
    int count;
    int y0,y1;
    for (count=0; count<pixels/2; count++) {
	/*gray pic*/
        y0 = yuv[count*4+0];
        //u0 = 0x80;
        y1 = yuv[count*4+2];
        //v1 = 0x80;

        buf[count*8+0] = y0; //b
        buf[count*8+1] = 0; //g
        buf[count*8+2] = 0; //r
        buf[count*8+3] = 0; //透明度

        buf[count*8+4] = y1; //b
        buf[count*8+5] = 0; //g
        buf[count*8+6] = 0; //r
        buf[count*8+7] = 0; //透明度
    }
    return buf;
}

#ifndef YUV_ENABLE
/*format as for mjpg*/
int read_frame (camera_t *camera)
{
     struct v4l2_buffer buf;
     fd_set fds;
     FD_ZERO(&fds);
     FD_SET(camera->fd, &fds);
     struct timeval timeout;
     timeout.tv_sec = 100;
     timeout.tv_usec = 0;
     int r = select(camera->fd + 1, &fds, 0, 0, &timeout);

     int file_fd = open("test2.jpg", O_RDWR | O_CREAT, 0777);
     if (r == -1) quit("select");
     if (r == 0) return FALSE;

     /*帧出列*/
     buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
     buf.memory = V4L2_MEMORY_MMAP;
     ioctl (camera->fd, VIDIOC_DQBUF, &buf);
     printf("[%s]:buf.bytesused=%d\n",__func__,buf.bytesused);
     printf("[%s]:buf.len=%zd,index=%d\n",__func__,camera->buffers[buf.index].length,buf.index);

     write(file_fd,camera->buffers[buf.index].start,camera->buffers[buf.index].length);
     //write(file_fd,camera->buffers[buf.index].start,buf.bytesused);

     /*buf入列*/
     ioctl(camera->fd, VIDIOC_QBUF, &buf);
     //sleep(1);
     close (file_fd);

     return 1;
}
#endif

