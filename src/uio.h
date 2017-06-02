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
#ifndef _UIO_H_
#define _UIO_H_
#include "list.h"

/*New protocol*/
struct raw_data {
    unsigned int data_width;
    unsigned int data_height;
    unsigned int data_depth;
    unsigned int data_len;
    char *data_buffer;
    struct list_head list;
    char name[60];
};

struct file_names{
    struct list_head list;
    char name[60];
};


enum mem_block_addr_e {
    MEM_BLOCK_BASE 	  = 0x3f000000,
    MEM_BLOCK_1_OFFSET    = 0x00000000,
    MEM_BLOCK_2_OFFSET    = 0x00300000,
    MEM_BLOCK_3_OFFSET    = 0x00600000,
    MEM_BLOCK_4_OFFSET    = 0x00900000,
} mem_block_addr_t __attribute__((unused));

static enum index_block_e {
    BLOCK_INDEX0 = 0,
    BLOCK_INDEX1 = 1,
    BLOCK_INDEX2,
    BLOCK_INDEX3,
    BLOCK_INDEX4,
} index_block_t __attribute__((unused));

static enum reg_index_e {
    REG_INDEX0	 = 0,
    REG_INDEX1,//24 bit including flag+Data Length
    REG_INDEX2,
    REG_INDEX3,
    REG_INDEX4,
    REG_INDEX5,
    REG_INDEX6,
    REG_INDEX7,
    REG_INDEX8,
} reg_index_t __attribute__((unused));

static enum io_status_e{
    IOS_PS_BLOCK1_WRITING_OVER = 0xf1,
    IOS_PL_BLOCK1_READING_OVER = 0xf2,
    IOS_PS_BLOCK2_WRITING_OVER = 0xf3,
    IOS_PL_BLOCK2_READING_OVER = 0xf4,

    IOS_PL_BLOCK3_WRITING_OVER = 0xf5,
    IOS_PS_BLOCK3_READING_OVER = 0xf6,
    IOS_PL_BLOCK4_WRITING_OVER = 0xf7,
    IOS_PS_BLOCK4_READING_OVER = 0xf8,
} io_status_t __attribute__((unused));

/*lib linux*/
//extern char *loadJpg(char* Name);
char *loadJpg(char* Name, struct raw_data *raw_data);

#endif
