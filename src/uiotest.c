#include <stddef.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <pthread.h>
#include <termios.h>
#include <inttypes.h>


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/*Linux file*/
#include <dirent.h>
#include <unistd.h>

/*uio*/
#include "uio.h"
#include "list.h"
#include "camera.h"


// modes
#define bool int
#define TRUE 1
#define FALSE 0

void *map_addr,*map_addrCt;
volatile unsigned int *mapped;
int ThreadsExit=1;
int sockfd=0;

#define WRITE_SIZE (8 * 1024 * 1024)
#define BUFFER_SIZE (32 * 1024 * 1024)

/*Global list head*/
LIST_HEAD(data_list_h);
LIST_HEAD(bakdata_list_h);
static int wr_block_index = BLOCK_INDEX0;
static int rd_block_index = BLOCK_INDEX3;

#define NONE         "\033[m"
#define RED          "\033[0;32;31m"
#define LIGHT_RED    "\033[1;31m"
#define GREEN        "\033[0;32;32m"
#define LIGHT_GREEN  "\033[1;32m"
#define BLUE         "\033[0;32;34m"
#define LIGHT_BLUE   "\033[1;34m"
#define DARY_GRAY    "\033[1;30m"
#define CYAN         "\033[0;36m"
#define LIGHT_CYAN   "\033[1;36m"
#define PURPLE       "\033[0;35m"
#define LIGHT_PURPLE "\033[1;35m"
#define BROWN        "\033[0;33m"
#define YELLOW       "\033[1;33m"
#define LIGHT_GRAY   "\033[0;37m"
#define WHITE        "\033[1;37m"

#define uio_print(fmt,...) do {  printf(GREEN"[%s]:"NONE fmt,__func__,##__VA_ARGS__) ;} while(0)
#define uio_color_print(color,fmt,...) do {  printf(GREEN"[%s]:"color fmt NONE,__func__,##__VA_ARGS__) ;} while(0)

//volatile char *memBaseAddr;
static char *memBaseAddr;
//volatile char *image_ReadAddr;

pthread_t p_tid[2];

struct CaptureArgs {
   char *FileName;
   FILE *fp;
};


/*Function declartion*/
int ParseInput(char *iString);
int ReadDMA(int p1,int p2,char *p3);
#ifndef CAMERA_EN
static void parse_each_file();
#endif

#if 0
/* Read 1 character - echo defines echo mode */
char getch_(int echo)
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void) 
{
  return getch_(0);
}

/* Read 1 character with echo */
char getche(void) 
{
  return getch_(1);
}
#endif

void config_wr_over_regs(struct raw_data *data)
{
    mapped[REG_INDEX0]=184;
    //mapped[REG_INDEX1]=data->data_len;
    mapped[REG_INDEX1]=2452;
    mapped[REG_INDEX2]=data->data_width;
    mapped[REG_INDEX3]=data->data_height;
    mapped[REG_INDEX6]=data->data_width*data->data_height;
    mapped[REG_INDEX7]=1;
    //uio_print("Block %d writing over,width=%d.height=%d\n",wr_block_index,data->data_width,data->data_height);

    if( wr_block_index == BLOCK_INDEX2 )
	mapped[REG_INDEX4]=IOS_PS_BLOCK2_WRITING_OVER;
    else{
	/*make sure that BLOCK_INDEX0 change to BLOCK_INDEX1*/
	wr_block_index = BLOCK_INDEX1;
	mapped[REG_INDEX4]=IOS_PS_BLOCK1_WRITING_OVER;
    }
    uio_color_print(RED,"Block %d write over\n",wr_block_index);
   /*change the index*/
    wr_block_index = (wr_block_index==BLOCK_INDEX2?BLOCK_INDEX1:BLOCK_INDEX2);
}

void config_rd_over_regs()
{
    if(rd_block_index==BLOCK_INDEX3)
	mapped[REG_INDEX5] = IOS_PS_BLOCK3_READING_OVER;
    else if(rd_block_index==BLOCK_INDEX4)
	mapped[REG_INDEX5] = IOS_PS_BLOCK4_READING_OVER;
    /*reset the PL write over flag*/
    mapped[REG_INDEX1] = mapped[REG_INDEX1] && 0xffff;
    uio_color_print(RED,"Block %d read over\n",rd_block_index);

    /*change the rd_block_index*/
    rd_block_index = (rd_block_index==BLOCK_INDEX3?BLOCK_INDEX4:BLOCK_INDEX3);
}



static struct raw_data *get_first_node(struct list_head *head)
{
    if( (head->next != NULL) && ( !list_empty(head) ) ){
	return list_entry(head->next,struct raw_data,list);
    }
    return NULL;
}

static void rm_first_node(struct list_head *head)
{
    if( head->next != NULL ){
	struct raw_data *node = list_entry(head->next,struct raw_data,list);
	list_del_init(head->next);
	free(node->data_buffer);
	free(node);
    }
}

static struct file_names *get_first_node_name(struct list_head *head)
{
    if( (head->next != NULL) && ( !list_empty(head) ) ){
	return list_entry(head->next,struct file_names,list);
    }
    return NULL;
}

static void rm_first_node_name(struct list_head *head)
{
    if( head->next != NULL ){
	struct file_names *node = list_entry(head->next,struct file_names,list);
	list_del_init(head->next);
	//list_del(head->next);
	free(node);
    }
}

void writeData(int offset)
{
	char * bufferAddr = memBaseAddr + offset;
	struct raw_data *node = get_first_node(&data_list_h);
	if(node == NULL ){
	    uio_print("No pic to write, sleep 5 seconds to refind pics\n");
	    sleep(5);
	    return;
	}
	//uio_print("len=%d,width=%d,height=%d\n",node->data_len,node->data_width,node->data_height);

	char *buffer = node->data_buffer;
	unsigned int size = node->data_len;
#ifdef CAMERA_EN
    struct list_head *plist,*pnode;
    list_for_each_safe(plist,pnode,&data_list_h){
	struct raw_data *node = list_entry(plist,struct raw_data,list);
	uio_print("Write List file=%s\n",node->name);
    }
#endif

	if (size == 0)
	{
		uio_print("size is 0 \n");
		return;
	}
	if ((size + offset) > WRITE_SIZE)
	{
		uio_print("size is too large \n");
		return;
	}
	uio_print("write data into 0x%p size %d \n", bufferAddr, size);

	memset(bufferAddr,'\0',size);
	memcpy((char *)bufferAddr,buffer,size);

	config_wr_over_regs(node);

	rm_first_node(&data_list_h);
}

void readData(int offset,int size)
{
    char name[100];
    DIR *tmp_dir;
    char * bufferAddr = memBaseAddr + offset;
    if (size == 0)
    {
	printf("size is 0 \n");
	return;
    }
    if ((size) > (WRITE_SIZE))
    {
	printf("size is too large \n");
	return;
    }
    uio_print("read data from 0x%p size 0x%x \n",bufferAddr, size);
#if 1
    struct list_head *plist,*pnode;
    list_for_each_safe(plist,pnode,&bakdata_list_h){
	struct file_names *node = list_entry(plist,struct file_names,list);    
	uio_print("Read list file=%s\n",node->name);
    }
#endif

#if 0
    char *out_dir="feature_out/";
    char out_path[120];
    struct file_names *node = get_first_node_name(&bakdata_list_h);
    if( node == NULL ){
	uio_color_print(YELLOW,"NO name file size 0x%x(*4bytes) \n",size>>2);
	return;
    }
    uio_color_print(YELLOW,"name %s file size 0x%x(*4bytes) \n",node->name,size>>2);
    if( ( tmp_dir = opendir(out_path)) == NULL ){
	mkdir(out_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }
    closedir(tmp_dir);
    memset(name,0,100);
    memcpy(name,node->name,strlen(node->name));
    strcat(name,".feature");
    memcpy(out_path,out_dir,strlen(out_dir));
    strcat(out_path,name);
    FILE *fp=fopen(out_path, "w+");

    if(fp!=0)
    {
	fseek(fp, 0, SEEK_SET);    /* file pointer at the beginning of file */
	size_t writeSize=fwrite((const void *)bufferAddr,1,size,fp);
	if (writeSize != size)
	{
	    uio_print("write file %s size %d fail \n",name, size);
	}else{
	    uio_color_print(YELLOW,"Read Data into file %s size 0x%x(*4bytes) \n",name,size>>2);
	}
	fclose(fp);
    }
    else
    {
	uio_print("open file %s fail \n",name);
    }
    rm_first_node_name(&bakdata_list_h);
#else
    char readbuffer[size*2];
    memcpy(readbuffer,bufferAddr,size);
    //uio_print("Data:%s\n",readbuffer);
    static unsigned int index_num=0;
    socket_data_t *skt_data=malloc(sizeof(socket_data_t));
    skt_data->head=0x8888;
    skt_data->index=index_num;
    skt_data->data_len=size;
    skt_data->end=0x8888;
    index_num++;
    if(size < (4096*4096))
	memcpy(skt_data->data,bufferAddr,size);
    else
	uio_print("size if error\n");
    size_t write_size=write(sockfd,skt_data,sizeof(socket_data_t));
    if(write_size == sizeof(socket_data_t))
	uio_print("size write success %d,size=%d\n",write_size,size);
    free(skt_data);
#endif
    config_rd_over_regs();
}

int getInput(void)
{
	char iString[255];
	if(fgets(iString,200,stdin)!=NULL)
	{
	    return 1;
	}
	return 0;

}

void *Ps_send_handle (void *arg)
{
    char *rd_p;
    int i=0;
    int index_num=0;
    int index_bytes=0;
    char pl_rd_over;

    uio_print("init \n");
    while(1)
    {
	sleep(1);
	rd_p=(char *)mapped;
	if(rd_p[0] != 0x08 ){
	    uio_print("Fpga Data is error\n");
	    return 0;
	}
	index_num=rd_p[1];
	index_bytes=rd_p[2];
	for(i=0;i<index_num;i++){
	    if(rd_p[4+i]==REG_INDEX0){
		/*high 8 bit + low 16 bits data_len*/
		pl_rd_over = (char )mapped[1+index_bytes+i];
		uio_print("pl_rd_over=0x%x\n",pl_rd_over);
		break;
	    }
	}

	if((wr_block_index == BLOCK_INDEX1) && ( pl_rd_over == IOS_PL_BLOCK2_READING_OVER) ){
	    writeData(MEM_BLOCK_1_OFFSET);

	}else if((wr_block_index == BLOCK_INDEX2) && ( pl_rd_over == IOS_PL_BLOCK1_READING_OVER) ){
	    writeData(MEM_BLOCK_2_OFFSET);

	}else if( wr_block_index == BLOCK_INDEX0 ){
	/*case for first time*/
	    wr_block_index = BLOCK_INDEX1;
	    writeData(MEM_BLOCK_1_OFFSET);

	}
    }
}

/*Handle PL output result */
void *Ps_recv_handle(void *arg)
{
    char *rd_p;
    int i=0;
    int index_num=0;
    int index_bytes=0;
    char pl_wr_over;
    unsigned short pl_wr_lens=0;
    uio_print("init \n");
    while(1)
    {
	sleep(1);
#if 0
	uio_print("index0=0x%x.index1=0x%x.index2=0x%x.index3=0x%x.index4=0x%x\n",\
			    					mapped[REG_INDEX0],\
								mapped[REG_INDEX1],mapped[REG_INDEX2],\
								mapped[REG_INDEX3],mapped[REG_INDEX4]);
#endif
	rd_p=(char *)mapped;
	if(rd_p[0] != 0x08 ){
		uio_print("Index Data is error\n");
		return 0;
	}
	index_num=rd_p[1];
	index_bytes=rd_p[2];
	//uio_print("index_num=%d,index_bytes=%d\n",index_num,index_bytes);
	for(i=0;i<index_num;i++){
		//uio_print("rd_p[4+%d]=0x%x\n",i,rd_p[4+i]);
		if( rd_p[4+i]==REG_INDEX1 ){
			/*high 8 bit + low 16 bits data_len*/
			pl_wr_over = rd_p[4+4*index_bytes+i*4+2];
			pl_wr_lens = (unsigned short )mapped[1+index_bytes+i];
		        //uio_print("pl_wr_over=0x%x\n",pl_wr_over);
		        //uio_print("pl_wr_lens=0x%x\n",pl_wr_lens);
			break;
		}
	}

	if( (rd_block_index== BLOCK_INDEX3) && ((pl_wr_over&0xff) == IOS_PL_BLOCK3_WRITING_OVER) ){
	    readData(MEM_BLOCK_3_OFFSET,((int)pl_wr_lens)<<2);

	}else if((rd_block_index == BLOCK_INDEX4) && ((pl_wr_over&0xff) == IOS_PL_BLOCK4_WRITING_OVER) ){
	    readData(MEM_BLOCK_4_OFFSET,((int)pl_wr_lens)<<2);
	}
    }
}

#ifndef CAMERA_EN
static void parse_each_file(char *sdir)
{
    //char outfile_name[30];
    char source_dir[100];
    char img_path[150];
    DIR *dir;
    struct dirent *ptr;
    int index=0;
    memset(source_dir,'\0',sizeof(source_dir));
    if( sdir==NULL ){
	getcwd(source_dir,999);
    }else
	memcpy(source_dir,sdir,strlen(sdir));
    uio_print("Pics Source dir is : %s\n",source_dir);
    if ((dir=opendir(source_dir)) == NULL)
    {
	perror("Open dir error...");
	exit(1);
    }
    while( (ptr=readdir(dir)) != NULL ){
	if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
	    continue;
	else if(ptr->d_type == 8)    ///file
	{
	    memset(img_path,0,150);
	    strcpy(img_path,source_dir);
	    strcat(img_path,ptr->d_name);
	    if(strstr(ptr->d_name,".jpg")!=NULL || (strstr(ptr->d_name,".jpeg"))!=NULL ){
		uio_print("Load file[%d] is : %s\n",index++,img_path);
		//strncpy(outfile_name,ptr->d_name,strlen(ptr->d_name)-strlen(".jpg"));
		//strcat(outfile_name,".bmp");
		struct raw_data *data = malloc(sizeof(struct raw_data));
		struct file_names *name_node = malloc(sizeof(struct file_names));
		memset(data->name,0,sizeof(data->name));
		memset(name_node->name,0,sizeof(name_node->name));
		/*load pic attrs*/
		char *buff = loadJpg((char *)img_path,data);
		if( buff==NULL ){
		    continue;
		}
		data->data_buffer=buff;
		memcpy(name_node->name,ptr->d_name,strlen(ptr->d_name)-strlen(".jpg"));
		memcpy(data->name,ptr->d_name,strlen(ptr->d_name)-strlen(".jpg"));
		list_add(&data->list, &data_list_h);
		list_add(&name_node->list, &bakdata_list_h);
	    }
	}
    }
    closedir(dir);
}
#endif

int regs_init()
{
    mapped[REG_INDEX0]=184;
    //mapped[REG_INDEX1]=data->data_len;
    mapped[REG_INDEX1]=2452;
    mapped[REG_INDEX7]=0;
    sleep(1);
    return 0;
}

int capture_frame()
{
    camera_t* camera = camera_open("/dev/video0", WIDTH, HEIGHT);

    if(camera==NULL){
	uio_print("camera open error\n");
	return 0;
    }
    camera_init(camera);
    camera_start(camera);

    struct timeval timeout;
    timeout.tv_sec = 100;
    timeout.tv_usec = 0;
    /* skip 5 frames for booting a cam */
    int i = 0;
    sleep(1);
    /*just for yuv*/
    for (i = 0; i < 4; i++) {
	camera_frame(camera, timeout);
	char* rgb = yuyv2raw(camera->head.start, camera->width*camera->height);
	if(rgb==NULL){
	    printf("[%s]:raw from camera is null\n",__func__);
	    continue;
	}
	struct raw_data *data=malloc(sizeof(struct raw_data));
	data->data_buffer=rgb;
	data->data_width=WIDTH;
	data->data_height=HEIGHT;
	data->data_len=WIDTH*HEIGHT*4;
	list_add(&data->list,&data_list_h);

	struct file_names *name_node = malloc(sizeof(struct file_names));
	sprintf(name_node->name,"camera_feature%d",i);
	list_add(&name_node->list, &bakdata_list_h);
    }
    camera_stop(camera);
    camera_finish(camera);
    camera_close(camera);
    return 0;
}

void sig_pipe(int signo)
{
    uio_print("catch a signal...\n");
    sleep(3);
    if(signo == SIGTSTP){
	close(sockfd);
    }
    exit(-1);
}

typedef void (*sighandler_t)(int);

int socket_init(int argc, char **argv)
{
    struct sockaddr_in s_addr;
    unsigned int port;
    char ip_addr[20];

    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1){
        perror("socket");
        exit(errno);
    }else
        uio_print("socket create success!\n");

    sighandler_t ret;
    ret = signal(SIGTSTP,sig_pipe);
    if(ret<0){
	uio_print("Signal connect error\n");
    }

    memset(ip_addr,0,sizeof(ip_addr));
    if(argc==4){
	if(argv[3])
	    port = atoi(argv[3]);
	if(argv[2])
	    memcpy(ip_addr,argv[2],strlen(argv[2]));
    }
    uio_print("IP=%s,port=%d\n",ip_addr,port);
    bzero(&s_addr, sizeof(s_addr));
    s_addr.sin_family = AF_INET;
    s_addr.sin_port = htons(port);
    if (inet_aton(ip_addr, (struct in_addr *)&s_addr.sin_addr.s_addr) == 0) {
	perror(ip_addr);
	exit(errno);
	}
    if(connect(sockfd,(struct sockaddr*)&s_addr,sizeof(struct sockaddr)) == -1){
        perror("connect");
        exit(errno);
    }else
        uio_print("conncet success!\n");

    return 0;
}

int main(int argc, char *argv[]) {

  int fd,mfd,err;

  int size;
  int imageSize = BUFFER_SIZE;

  size = 4095;
  if (size <= 0) {
    fprintf(stderr, "Bad size: %d\n", size);
    exit(1);
  }

#ifndef CAMERA_EN
  parse_each_file(argv[1]);
#else
  capture_frame();
#endif

  fd = open("/dev/uio1", O_RDWR);
  if (fd < 0) {
    perror("Failed to open uio1 devfile");
    return 1;
  }

  map_addr = mmap( NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  if (map_addr == MAP_FAILED) {
    perror("Failed to mmap");
    return 1;
  }

  mapped = map_addr;


  mfd = open("/dev/mem", O_RDWR);
  map_addrCt = mmap(NULL, imageSize, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, 0x3f000000);

  if (map_addrCt == MAP_FAILED) {
      //printf("\nCant Map Callibration Tables \n");
      perror("Failed to mmap High memory");
      return 1;
  }
  memBaseAddr = (char *)map_addrCt;
  //image_ReadAddr = memBaseAddr + WRITE_SIZE;

  regs_init();

  socket_init(argc, argv);

  err = pthread_create(&p_tid[0], NULL, &Ps_send_handle,  (void*)&ThreadsExit);
  if (err != 0)
  {
      printf("\ncan't create thread :[%s]", strerror(err));
      return 1;
  }
  else
  {
      printf("\n Begin Ps send Thread\n");
  }

  err = pthread_create(&p_tid[1], NULL, &Ps_recv_handle,  (void*)&ThreadsExit);
  if (err != 0)
  {
      printf("\ncan't create thread :[%s]", strerror(err));
      return 1;
  }
  else
  {
      printf("\n Begin Ps recv Thread\n");
  }

  while(getInput()==0);

  ThreadsExit=0;
  sleep(1);

  printf("\nExiting \n");

  munmap(map_addr, size);
  munmap(map_addrCt, imageSize);

  close(fd);
  close(mfd);
  close(sockfd);
  return 0;
}
