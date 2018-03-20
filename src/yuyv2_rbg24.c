this config can solve the color distortion!!!!!
this config is best configurtion.
//yuyv转rgb32的算法实现
static int sign3 = 1;
/*
YUV到RGB的转换有如下公式：
R = 1.164*(Y-16) + 1.159*(V-128);
G = 1.164*(Y-16) - 0.380*(U-128)+ 0.813*(V-128);
B = 1.164*(Y-16) + 2.018*(U-128));
*/
int yuvtorgb(int y, int u, int v)
{
     unsigned int pixel32 = 0;
     unsigned char *pixel = (unsigned char *)&pixel32;
     int r, g, b;
     static long int ruv, guv, buv;

     if(1 == sign3)
     {
         sign3 = 0;
         ruv = 1159*(v-128);
         guv = -380*(u-128) + 813*(v-128);
         buv = 2018*(u-128);
     }

     r = (1164*(y-16) + ruv) / 1000;
     g = (1164*(y-16) - guv) / 1000;
     b = (1164*(y-16) + buv) / 1000;

     if(r > 255) r = 255;
     if(g > 255) g = 255;
     if(b > 255) b = 255;
     if(r < 0) r = 0;
     if(g < 0) g = 0;
     if(b < 0) b = 0;
#if 0
     pixel[0] = r;
     pixel[1] = g;
     pixel[2] = b;
#endif
     pixel[0] = b;
     pixel[1] = g;
     pixel[2] = r;
     return pixel32;
}

int yuyv422_2_abgr( unsigned char *rgb,unsigned char *yuv,int height, int
width)
{
     unsigned int in, out;
     int y0, u, y1, v;
     unsigned int pixel32;
     unsigned char *pixel = (unsigned char *)&pixel32;
     //分辨率描述像素点个数，而yuv2个像素点占有4个字符，所以这里计算总的字符个数，需要乘2
     unsigned int size = width*height*2;

     for(in = 0, out = 0; in < size; in += 4, out += 6)
     {
          y0 = yuv[in+0];
          u  = yuv[in+1];
          y1 = yuv[in+2];
          v  = yuv[in+3];

          sign3 = 1;
          pixel32 = yuvtorgb(y0, u, v);
          rgb[out+0] = pixel[0];
          rgb[out+1] = pixel[1];
          rgb[out+2] = pixel[2];
          //rgb[out+3] = 0;  //32位rgb多了一个保留位

          pixel32 = yuvtorgb(y1, u, v);
          rgb[out+3] = pixel[0];
          rgb[out+4] = pixel[1];
          rgb[out+5] = pixel[2];
          //rgb[out+7] = 0;

     }
     return 0;
}
