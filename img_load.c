#include <stddef.h>
//#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "jpeglib.h"
#include "uio.h"
#pragma comment(lib,"jpeg.lib")


char *loadJpg(char* Name, struct bmp_data *bmp_data) {
  unsigned char a, r, g, b;
  int width, height, depth;
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;
  int x = 0;

  FILE * infile;        /* source file */
  JSAMPARRAY pJpegBuffer;       /* Output row buffer */
  int row_stride;       /* physical row width in output buffer */
  if ((infile = fopen(Name, "rb+")) == NULL) {
    printf("1can't open %s\n", Name);
    return 0;
  }
  printf("infile= %s \n",Name);

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);
  jpeg_stdio_src(&cinfo, infile);
  (void) jpeg_read_header(&cinfo, TRUE);
  (void) jpeg_start_decompress(&cinfo);
  width = cinfo.output_width;
  height = cinfo.output_height;
  depth = cinfo.output_components;
  printf("width = %d, height = %d, depth = %d!\n",width,height,depth);

  //unsigned char * pDummy = new unsigned char [width*height*4];
  unsigned char * pDummy = (unsigned char *)malloc(width*height*4);
  unsigned char * pTest = pDummy;
  if (!pDummy) {
    printf("NO MEM FOR JPEG CONVERT!\n");
    return 0;
  }
  row_stride = width * cinfo.output_components;
  pJpegBuffer = (*cinfo.mem->alloc_sarray)
    ((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);

  while (cinfo.output_scanline < cinfo.output_height) {
    (void) jpeg_read_scanlines(&cinfo, pJpegBuffer, 1);
    for (x = 0; x < width; x++) {
      a = 0; // alpha value is not supported on jpg
      r = pJpegBuffer[0][cinfo.output_components * x];
      if (cinfo.output_components > 2) {
        g = pJpegBuffer[0][cinfo.output_components * x + 1];
        b = pJpegBuffer[0][cinfo.output_components * x + 2];
      } else {
        g = r;
        b = r;
      }
      //printf("r = %d, g = %d, b = %d!\n",r,g,b);
      *(pDummy++) = b;
      *(pDummy++) = 0;
      *(pDummy++) = 0;
      *(pDummy++) = 0;
    }
  }

  //fwrite(pTest,1,width*height*4,outfile);

  fclose(infile);
  //free(pTest);

  (void) jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);

  //BMap = (int*)pTest;
  //Height = height;
  //Width = width;
  //Depth = 32;
  bmp_data->data_width=width;
  bmp_data->data_height=height;
  bmp_data->data_depth=depth;
  bmp_data->data_len=width*height*4;
  bmp_data->data_buffer=(char *)pTest;
  return (char *)pTest;
}

#if 0
int main( int argc, char **argv)
{
    char outfile_name[30];
    strncpy(outfile_name,argv[1],strlen(argv[1])-strlen(".jpg"));
    strcat(outfile_name,".bmp");
    loadJpg((char *)argv[1],outfile_name);
    //analyse_jpeg();
    printf("Good job!, done\n");
    //cin.get();
    return 0;
}
#endif
