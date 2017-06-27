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
#include<stdio.h>
#include<unistd.h>
#include<stdlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <termios.h>
/*Linux file*/
#include <dirent.h>
#include <unistd.h>
#include <termios.h>
#include <inttypes.h>
#include <sys/stat.h> 


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/*Linux file*/
#include <dirent.h>
#include <unistd.h>





typedef struct socket_data{
    int head;
    int index;
    unsigned int data_len;
    unsigned char data[4096*4096];
    int end;
} socket_data_t;



typedef void (*sighandler_t)(int);
#define BUFLEN (4096*4096*4)
int sockfd, newfd;
void sig_pipe(int signo);

void sig_pipe(int signo)
{
    printf("catch a signal...\n");
    sleep(2);
    if(signo == SIGTSTP){
	close(sockfd);
	close(newfd);
    }
    exit(-1);
}

void write_into_files(unsigned char *one_node,unsigned int data_len)
{

    char *out_dir="server_out/";
    char out_path[120];
    DIR *tmp_dir;
    if( ( tmp_dir = opendir(out_path)) == NULL ){
	mkdir(out_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }
    closedir(tmp_dir);
    char file_name[100];
    bzero(file_name,100);
    sprintf(file_name,"%d",node->index);
    strcat(file_name,".feature");
    memcpy(out_path,out_dir,strlen(out_dir));
    strcat(out_path,file_name);
    FILE *fp=fopen(out_path, "w+");

    if(fp!=0)
    {
	fseek(fp, 0, SEEK_SET);    /* file pointer at the beginning of file */
	size_t writeSize=fwrite((const void *)one_node,1,data_len,fp);
	if (writeSize != data_len)
	{
	    printf("write file %s size %d fail \n",file_name, data_len);
	}else{
	    printf("Read Data into file %s size 0x%x(*4bytes) \n",file_name,data_len>>2);
	}
	fclose(fp);
    }
    else
    {
	printf("open file %s fail \n",file_name);
    }
    return;
}

int main(int argc, char **argv)
{
    printf("line=%d!\n",__LINE__);
    struct sockaddr_in s_addr, c_addr;
    static unsigned char buf[BUFLEN];
    static unsigned char one_node[BUFLEN];
    socklen_t len;
    unsigned int port, listnum;

    printf("line=%d!\n",__LINE__);
    /*建立socket*/
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1){
        perror("socket");
        exit(errno);
    }else
        printf("socket create success!\n");

    sighandler_t ret;
    ret = signal(SIGTSTP,sig_pipe);
    if(ret<0){
	printf("Signal connect error\n");
    }


    /*设置服务器端口*/
    if(argv[2])
        port = atoi(argv[2]);
    else
        port = 4569;
    /*设置侦听队列长度*/
    if(argv[3])
        listnum = atoi(argv[3]);
    else
        listnum = 3;
    /*设置服务器ip*/
    bzero(&s_addr, sizeof(s_addr));
    s_addr.sin_family = AF_INET;
    s_addr.sin_port = htons(port);
    if(argv[1])
        s_addr.sin_addr.s_addr = inet_addr(argv[1]);
    else
        s_addr.sin_addr.s_addr = inet_addr("172.29.2.17");
    /*把地址和端口帮定到套接字上*/
    if((bind(sockfd, (struct sockaddr*) &s_addr,sizeof(struct sockaddr))) == -1){
        perror("bind");
        exit(errno);
    }else
        printf("bind success!\n");
    /*侦听本地端口*/
    if(listen(sockfd,listnum) == -1){
        perror("listen");
        exit(errno);
    }else
        printf("the server is listening!\n");


    printf("111111111111111111!\n");
    static unsigned int  skt_data_len=0;
    while(1){
        printf("*****************聊天开始***************\n");
        len = sizeof(struct sockaddr);
        if((newfd = accept(sockfd,(struct sockaddr*) &c_addr, &len)) == -1){
            perror("accept");
            exit(errno);
        }else
            printf("正在与您聊天的客户端是：%s: %d\n",inet_ntoa(c_addr.sin_addr),ntohs(c_addr.sin_port));
        while(1){
            /******接收消息*******/
            bzero(buf,BUFLEN);
            len = recv(newfd,buf,BUFLEN,0);
            if(len > 0){
		    skt_data_len+=len;
		    if(skt_data_len == sizeof(socket_data_t)){
			memcpy(one_node+skt_data_len-len,buf,len);
			socket_data_t *node=(socket_data_t*)one_node;
			printf("\n客户端发来的信息 data_len=0x%d,head=0x%x,index=%d,end=0x%x\n",node->data_len,node->head,node->index,node->end);
			if(node->data_len && node->head==0x8888)
			    write_into_files(node->data,node->data_len);

			skt_data_len=0;
		    }else if(skt_data_len > sizeof(socket_data_t)){
			printf("\nDiscard the message\n");
			skt_data_len=0;
			break;
		    }else{
			memcpy(one_node+skt_data_len-len,buf,len);
			continue;
		    }
	    }
            else{
                if(len < 0 )
                    printf("接受消息失败！\n");
                else
                    printf("客户端退出了，聊天终止！\n");
                break;
            }
        }
        /*关闭聊天的套接字*/
        close(newfd);
        /*是否退出服务器*/
        printf("服务器是否退出程序：y->是；n->否? ");
        bzero(buf, BUFLEN);
        fgets(buf,BUFLEN, stdin);
        if(!strncasecmp(buf,"y",1)){
            printf("server 退出!\n");
            break;
        }
    }
    /*关闭服务器的套接字*/
    close(sockfd);
    return 0;
}
