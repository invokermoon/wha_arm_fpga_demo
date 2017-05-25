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


#define INPUT_OFFSET 11

// Motor position index
#define BASE_POSITION 0
#define END_POSITON 1
#define PIVOT_POSITON 2
#define ANGLE_POSITON 3
#define ROT_POSITON 4

#define ACCELERATION_MAXSPEED 5

// DC bias paramaters
#define BASE_SIN_CENTER 6
#define BASE_COS_CENTER 7
#define END_SIN_CENTER 8
#define END_COS_CENTER 9
#define PIVOT_SIN_CENTER 10
#define PIVOT_COS_CENTER 11
#define ANGLE_SIN_CENTER 12
#define ANGLE_COS_CENTER 13
#define ROT_SIN_CENTER 14
#define ROT_COS_CENTER 15

//PID VALUES
#define PID_DELTATNOT 16
#define PID_DELTAT 17
#define PID_D 18
#define PID_I 19
#define PID_P 20
#define PID_ADDRESS 21

//FORCE PARAMATERS
#define BOUNDRY_BASE 22
#define BOUNDRY_END 23
#define BOUNDRY_PIVOT 24

#define BOUNDRY_ANGLE 25
#define BOUNDRY_ROT 26

#define SPEED_FACTORA 27
#define SPEED_FACTORB 28

#define FRICTION_BASE 29
#define FRICTION_END 30
#define FRICTION_PIVOT 31
#define FRICTION_ANGLE 32
#define FRICTION_ROT 33

#define MOVE_TRHESHOLD 34
#define F_FACTOR 35
#define MAX_ERROR 36

#define FORCE_BIAS_BASE 37
#define FORCE_BIAS_END 38
#define FORCE_BIAS_PIVOT 39
#define FORCE_BIAS_ANGLE 40
#define FORCE_BIAS_ROT 41

// control state register
#define COMMAND_REG 42
#define CMD_CAPCAL_BASE 1
#define CMD_CAPCAL_END 2
#define CMD_CAPCAL_PIVOT 4
#define CMD_MOVEEN 8
#define CMD_MOVEGO 16
#define CMD_ENABLE_LOOP 32
#define CMD_CLEAR_LOOP 64
#define CMD_CALIBRATE_RUN 128
#define CMD_RESET_POSITION 256
#define CMD_RESET_FORCE 512
#define CMD_CAPCAL_ANGLE 1024
#define CMD_CAPCAL_ROT 2048
#define CMD_ANGLE_ENABLE 4096
#define CMD_ROT_ENABLE 8196







//DMA 
#define DMA_READ_ADDRESS 48
#define DMA_READ_PARAMS 47
#define DMA_WRITE_ADDRESS 46
#define DMA_WRITE_PARAMS 45
#define DMA_WRITE_DATA 44
#define DMA_CONTROL 43

// DMA control bits breakdown

#define DMA_WRITE_ENQUEUE 1
#define DMA_WRITE_INITIATE 2
#define DMA_READ_DEQUEUE 4
#define DMA_READ_BLOCK 8
#define DMA_RESET_ALL 16

#define REC_PLAY_CMD 49
#define CMD_RESET_RECORD 1
#define CMD_RECORD 2
#define CMD_RESET_PLAY 4
#define CMD_PLAYBACK 8
#define CMD_RESET_PLAY_POSITION 16



#define REC_PLAY_TIMEBASE 50
#define DIFF_FORCE_TIMEBASE 51
#define DIFF_FORCE_BETA 52
#define DIFF_FORCE_MOVE_THRESHOLD 53
#define DIFF_FORCE_MAX_SPEED 54
#define DIFF_FORCE_SPEED_FACTOR_ANGLE 55
#define DIFF_FORCE_SPEED_FACTOR_ROT 56
#define DIFF_FORCE_ANGLE_COMPENSATE 57

#define FINE_ADJUST_BASE 58
#define FINE_ADJUST_END 59
#define FINE_ADJUST_PIVOT 60
#define FINE_ADJUST_ANGLE 61
#define FINE_ADJUST_ROT 62

#define RECORD_LENGTH 63



#define END_EFFECTOR_IO 64
#define SERVO_SETPOINT_A 65
#define SERVO_SETPOINT_B 66

#define BASE_FORCE_DECAY 67
#define END_FORCE_DECAY 68
#define PIVOT_FORCE_DECAY 69
#define ANGLE_FORCE_DECAY 70
#define ROTATE_FORCE_DECAY 71

#define PID_SCHEDULE_INDEX 72

#define GRIPPER_MOTOR_CONTROL 73
#define GRIPPER_MOTOR_ON_WIDTH 75
#define GRIPPER_MOTOR_OFF_WIDTH 74
#define START_SPEED 76





// DMA DATA IN

#define DMA_READ_DATA 30 + INPUT_OFFSET





// READ REGISTER DEFINITIONS

// POSITION REPORT
#define BASE_POSITION_AT 0 + INPUT_OFFSET
#define END_POSITION_AT 1 + INPUT_OFFSET
#define PIVOT_POSITION_AT 2 + INPUT_OFFSET
#define ANGLE_POSITION_AT 3 + INPUT_OFFSET
#define ROT_POSITION_AT 4 + INPUT_OFFSET

//TABLE CALCULATED DELTA

#define BASE_POSITION_DELTA 5 + INPUT_OFFSET
#define END_POSITION_DELTA 6 + INPUT_OFFSET
#define PIVOT_POSITION_DELTA 7 + INPUT_OFFSET
#define ANGLE_POSITION_DELTA 8 + INPUT_OFFSET
#define ROT_POSITION_DELTA 9 + INPUT_OFFSET


//PID CALCULATED DELTA

#define BASE_POSITION_PID_DELTA 10 + INPUT_OFFSET
#define END_POSITION_PID_DELTA 11 + INPUT_OFFSET
#define PIVOT_POSITION_PID_DELTA 12 + INPUT_OFFSET
#define ANGLE_POSITION_PID_DELTA 13 + INPUT_OFFSET
#define ROT_POSITION_PID_DELTA 14 + INPUT_OFFSET


// FORCE CALCULATED POSITION MODIFICATION

#define BASE_POSITION_FORCE_DELTA 15 + INPUT_OFFSET
#define END_POSITION_FORCE_DELTA 16 + INPUT_OFFSET
#define PIVOT_POSITION_FORCE_DELTA 17 + INPUT_OFFSET
#define ANGLE_POSITION_FORCE_DELTA 18 + INPUT_OFFSET
#define ROT_POSITION_FORCE_DELTA 19 + INPUT_OFFSET


// RAW ANALOG TO DIGITAL VALUES

#define BASE_SIN 20 + INPUT_OFFSET
#define BASE_COS 21 + INPUT_OFFSET
#define PIVOT_SIN 22 + INPUT_OFFSET
#define PIVOT_COS 23 + INPUT_OFFSET
#define ANGLE_SIN 24 + INPUT_OFFSET
#define ANGLE_COS 25 + INPUT_OFFSET
#define END_SIN 26 + INPUT_OFFSET
#define END_COS 27 + INPUT_OFFSET
#define ROT_SIN 28 + INPUT_OFFSET
#define ROT_COS 29 + INPUT_OFFSET

// RECORD AND PLAYBACK 

#define RECORD_BLOCK_SIZE 31 + INPUT_OFFSET
#define READ_BLOCK_COUNT 32 + INPUT_OFFSET
#define PLAYBACK_BASE_POSITION 33 + INPUT_OFFSET
#define PLAYBACK_END_POSITION 34 + INPUT_OFFSET
#define PLAYBACK_PIVOT_POSITION 35 + INPUT_OFFSET
#define PLAYBACK_ANGLE_POSITION 36 + INPUT_OFFSET
#define PLAYBACK_ROT_POSITION 37 + INPUT_OFFSET


#define END_EFFECTOR_IO_IN 38 + INPUT_OFFSET

#define SENT_BASE_POSITION 39 + INPUT_OFFSET
#define SENT_END_POSITION 40 + INPUT_OFFSET
#define SENT_PIVOT_POSITION 41 + INPUT_OFFSET
#define SENT_ANGLE_POSITION 42 + INPUT_OFFSET
#define SENT_ROT_POSITION 43 + INPUT_OFFSET

#define SLOPE_BASE_POSITION 44 + INPUT_OFFSET
#define SLOPE_END_POSITION 45 + INPUT_OFFSET
#define SLOPE_PIVOT_POSITION 46 + INPUT_OFFSET
#define SLOPE_ANGLE_POSITION 47 + INPUT_OFFSET
#define SLOPE_ROT_POSITION 48 + INPUT_OFFSET








int ADLookUp[5] = {BASE_SIN,END_SIN,PIVOT_SIN,ANGLE_SIN,ROT_SIN};

//commands

#define MOVE_CMD 1
#define READ_CMD 2
#define WRITE_CMD 3
#define EXIT_CMD 4
#define SLOWMOVE_CMD 5
#define MOVEALL_CMD 6
#define DMAREAD_CMD 7
#define DMAWRITE_CMD 8
#define CAPTURE_AD_CMD 9
#define FIND_HOME_CMD 10
#define FIND_HOME_REP_CMD 11
#define LOAD_TABLES 12
#define CAPTURE_POINTS_CMD 14
#define FIND_INDEX_CMD 15
#define SLEEP_CMD 16
#define RECORD_MOVEMENT 17
#define REPLAY_MOVEMENT 18
#define MOVEALL_RELATIVE 19
#define SET_PARAM 20
#define SEND_HEARTBEAT 21
#define SET_FORCE_MOVE_POINT 22
#define HEART_BEAT 23
#define WRITE_BUFFER_CMD 24
#define READ_BUFFER_CMD 25


// modes
#define BLOCKING_MOVE 1
#define NON_BLOCKING_MOVE 2
#define DEFAULT_MOVE_TIMEOUT 0
#define TRUE 1
#define FALSE 0


// defaults
#define DEFAULT_PID_SETTING_XYZ 0x3dcCCCCC
//#define DEFAULT_PID_SETTING_XYZ 0x3f000000
//#define DEFAULT_PID_SETTING_XYZ 0x3f800000
//#define DEFAULT_PID_SETTING_PY 0x3f000000
//#define DEFAULT_PID_SETTING_PY 0x3D4CCCCC
//#define DEFAULT_PID_SETTING_PY 0x3dcCCCCC
#define DEFAULT_PID_SETTING_PY 0x3cf5c28f


#define DEF_SPEED_FACTOR_A 12
#define DEF_SPEED_FACTOR_DIFF 8

#define ACCELERATION_MAXSPEED_DEF 250000+(3<<20)

#define BASE_SIN_LOW 0X760
#define BASE_COS_LOW 0X1
#define BASE_SIN_HIGH 0x83A
#define BASE_COS_HIGH 0X40

#define END_SIN_LOW 0Xc01
#define END_COS_LOW 0X0
#define END_SIN_HIGH 0xd5c
#define END_COS_HIGH 0X70

#define PIVOT_SIN_LOW 0X50
#define PIVOT_COS_LOW 0X880
#define PIVOT_SIN_HIGH 0x100
#define PIVOT_COS_HIGH 0X8d1

#define ANGLE_SIN_LOW 0X900
#define ANGLE_COS_LOW 0X1
#define ANGLE_SIN_HIGH 0x9A0
#define ANGLE_COS_HIGH 0X50

#define ROT_SIN_LOW 0XA90
#define ROT_COS_LOW 0X1
#define ROT_SIN_HIGH 0xcc2
#define ROT_COS_HIGH 0X50


#define DEFAULT_ANGLE_BOUNDRY_HI 3000
#define DEFAULT_ANGLE_BOUNDRY_LOW -3000

#define SERVO_LOW_BOUND 1142000
#define SERVO_HI_BOUND 1355000


#define bool int
#define TRUE 1
#define FALSE 0

int XLowBound[5]={BASE_COS_LOW,END_COS_LOW,PIVOT_COS_LOW,ANGLE_COS_LOW,ROT_COS_LOW};
int XHighBound[5]={BASE_COS_HIGH,END_COS_HIGH,PIVOT_COS_HIGH,ANGLE_COS_HIGH,ROT_COS_HIGH};
int YLowBound[5]={BASE_SIN_LOW,END_SIN_LOW,PIVOT_SIN_LOW,ANGLE_SIN_LOW,ROT_SIN_LOW};
int YHighBound[5]={BASE_SIN_HIGH,END_SIN_HIGH,PIVOT_SIN_HIGH,ANGLE_SIN_HIGH,ROT_SIN_HIGH};
int ForcePossition[5]={0,0,0,0,0};
int ForceDestination[5]={0,0,0,0,0};
int ThreadsExit=1;

void *map_addr,*map_addrCt;
volatile unsigned int *mapped;

#define WRITE_SIZE (8 * 1024 * 1024)
#define BUFFER_SIZE (32 * 1024 * 1024)

/*Global list head*/
LIST_HEAD(list_h);
static int wr_block_index = BLOCK_INDEX0;
static int rd_block_index = BLOCK_INDEX3;

#define uio_print(fmt,...) do {  printf("[%s]:"fmt,__func__,##__VA_ARGS__) ;} while(0)

volatile char *memBaseAddr;
volatile char *image_ReadAddr;

//char testAddr[WRITE_SIZE] = {0};
int CmdVal=0,maxSpeed=0,startSpeed=0,coupledAcceleration=0,forceMode=0,fa0=0,fa1=0,fa2=0,fa3=0,fa4=0,RunMode=0;
int BaseBoundryHigh,BaseBoundryLow,EndBoundryHigh,EndBoundryLow,PivotBoundryHigh,PivotBoundryLow,AngleBoundryHigh=DEFAULT_ANGLE_BOUNDRY_HI,AngleBoundryLow=DEFAULT_ANGLE_BOUNDRY_LOW,RotateBoundryHigh,RotateBoundryLow;
//int Cycle = 0;
struct BotPossition {
	int base;
	int end;
	int pivot;
	int angle;
	int rotate;
};
struct BotSetHomePossition {
	int base;
	int end;
	int pivot;
	int angle;
	int rotate;
};
char RemoteRobotAdd[255];
struct MasterSlaveMoveRatio{
	int base;
	int end;
	int pivot;
	int angle;
	int rotate;	
};
struct MasterSlaveMoveDelta{
	int base;
	int end;
	int pivot;
	int angle;
	int rotate;	
};
struct SlaveBotPossiton{
	int base;
	int end;
	int pivot;
	int angle;
	int rotate;	
};
float DiffCorrectionFactor;
int AngleHIBoundry;
int AngleLOWBoundry;

int DexError=0;

struct MasterSlaveMoveRatio MSMoveRatio = {1,1,1,1,1};
struct MasterSlaveMoveDelta MSMoveDelta = {0,0,0,0,0};
struct BotSetHomePossition SetHome = {0,0,0,0,0};
struct SlaveBotPossiton SlaveBotPos = {0,0,0,0,0};
pthread_t tid[10];
pthread_t p_tid[2];

struct CaptureArgs {
   char *FileName;
   FILE *fp;
};


/*Function declartion*/
int GetAxisCurrent(int Axis);
bool ProcessServerReceiveDataDDE(char *recBuff);
bool ProcessServerSendDataDDE(char *,char *);
bool ProcessServerSendData(char *sendBuff);
bool ProcessServerReceiveData(char *recBuff);
int getNormalizedInput(int param);
int ParseInput(char *iString);
int GetAxisRemote(int Axis);
int MoveRobot(int a1,int a2,int a3,int a4,int a5, int mode);
int ReadDMA(int p1,int p2,char *p3);
static void parse_each_file();

struct CaptureArgs CptA,CptMove;
#define MAX_PARAMS 25
#define PARAM_LENGTH 20
char Params[MAX_PARAMS][PARAM_LENGTH+1] = {"MaxSpeed", "Acceleration", "BaseForce","EndForce","PivotForce","AngleForce","RotateForce","BaseFriction","EndFriction","PivotFriction","AngleFriction","RotateFriction","BaseBoundryHigh","BaseBoundryLow","EndBoundryHigh","EndBoundryLow","PivotBoundryHigh","PivotBoundryLow","AngleBoundryHigh","AngleBoundryLow","RotateBoundryHigh","RotateBoundryLow","GripperMotor","EERoll","End"};

static struct termios old, new;

int PositionAdjust[5]={0,0,0,0,0};
int ForceLimit[5]={800,800,800,300,300};
float ForceAdjustPossition[5]={-.01,-.01,-.01,-.01,-.01};
int sign(int i)
{
	if(i<0)
		return -1;
	if(i>0)
		return 1;
	return 0;
}

void *RealtimeMonitor(void *arg)
{
	int* ExitState = arg;
	//int i,j,ForceDelta,disTime=0;
	int HiBoundry,LowBoundry;
	int Hbd,Lbd;
	int EndAxisLoc,BoundryACC,AngleCorrectionFactor;
	float EndBoundryScale=DiffCorrectionFactor/16;//0.0666;
	while(*ExitState)
	{

		EndAxisLoc=GetAxisCurrent(1);
		AngleCorrectionFactor=EndAxisLoc*EndBoundryScale;
		Hbd=AngleBoundryHigh+AngleCorrectionFactor;
		Lbd=AngleBoundryLow+AngleCorrectionFactor;
		HiBoundry=Hbd/8;
		LowBoundry=Lbd/8;
		BoundryACC=(HiBoundry << 16) | (0x0000ffff & LowBoundry);
		//printf("%x %d %d \n",BoundryACC,Hbd,Lbd);
		mapped[BOUNDRY_ANGLE]=BoundryACC;//0x01f4 fe0c; //

		/*
		// do force based movement
		for(i=0;i<5;i++)
		{
			ForceDelta=ForcePossition[i]-getNormalizedInput(BASE_POSITION_FORCE_DELTA+i);
			if(disTime>100)
			{
				//printf("%d Delta %d ",getNormalizedInput(BASE_POSITION_FORCE_DELTA+i),ForcePossition[i]);
			}
			if(abs(ForceDelta)<ForceLimit[i])
			{
				mapped[FORCE_BIAS_BASE+i]=ForceDelta;
			}
			else
			{
				mapped[FORCE_BIAS_BASE+i]=sign(ForceDelta)*ForceLimit[i];
			}
			//ForcePossition[i]=ForcePossition[i]+(int)((float)ForceDelta*ForceAdjustPossition[i]);
			if(ForceDestination[i]>ForcePossition[i])
			{
				ForcePossition[i]=ForcePossition[i]+10;
			}
			else
			{
				ForcePossition[i]=ForcePossition[i]-10;
			}
		}
		if(disTime>100)
		{
			//printf("\n");
			disTime=0;
		}
		disTime++;*/
		usleep(1000);
	}
	printf("\nMonitor Thread Exiting\n");
	return (void*)1;
}
void SetGripperRoll(int Possition)
{
	int ServoSpan=(SERVO_HI_BOUND-SERVO_LOW_BOUND)/360;
	mapped[END_EFFECTOR_IO]=64;

	mapped[SERVO_SETPOINT_B]=(ServoSpan*Possition)+SERVO_LOW_BOUND;
//	mapped[SERVO_SETPOINT_B]=Possition;

}
void SetGripperMotor(int state)
{
	mapped[GRIPPER_MOTOR_ON_WIDTH]=12000;
	mapped[GRIPPER_MOTOR_OFF_WIDTH]=200000;
	mapped[GRIPPER_MOTOR_CONTROL]=state;
}

void *StartServerSocketDDE(void *arg)
{
	struct timeval tv;
	tv.tv_sec = 6;  /* 30 Secs Timeout */
	tv.tv_usec = 0;  // Not init'ing this can cause strange errors

	int* ExitState = arg;
	int listenfd = 0, connfd = 0,RLength = 0;
    struct sockaddr_in serv_addr; 

    char sendBuff[256];
    char recBuff[256];
    //time_t ticks; 
    bool SocketLive=0;
    printf("\n Start DDE Socket Server \n");

    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
    memset(sendBuff, '0', sizeof(sendBuff)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(50000); 

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)); 

    listen(listenfd, 10); 

    setsockopt(listenfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
    while(*ExitState)
    {
        // connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);
        while ( (connfd = accept(listenfd, (struct sockaddr*)NULL, NULL)) < 0)
		{
			printf("error on accept()! %s\n", strerror(errno));
			//printf("\n Timeout no connect \n");
		} 
		printf("\n new connect \n");


		//ticks = time(NULL);
		//strcpy(sendBuff,"Dexter Connect Service Connected\n");
		//snprintf(sendBuff, sizeof(sendBuff), "%.24s\r\n", ctime(&ticks));
		//write(connfd, sendBuff, strlen(sendBuff)); 
		SocketLive=TRUE;
//		while(SocketLive==TRUE)
		{
			while((errno = 0, (RLength = recv(connfd, recBuff, /*sizeof(recBuff)*/128, 0))>0) || 
			errno == EINTR)
			{
				if(RLength>0)
				{
					//printf("Receive size %i ",RLength);
					//output.append(recBuff, RLength);
				}


				if(RLength < 0)
				{
					printf("%s \n",strerror(errno));
					/* handle error - for example throw an exception*/
					SocketLive = FALSE;
				}


//			while ( (RLength = recv (connfd,recBuff,sizeof(recBuff),0 )) > 0)
				{
					//recBuff[RLength]=0;
					ProcessServerReceiveDataDDE(recBuff);

					ProcessServerSendDataDDE(sendBuff,recBuff);/*==TRUE)*/
						write (connfd,sendBuff,sizeof(sendBuff)); 
				}
			}
			printf("error code %s \n",strerror(errno));
		}
		printf("\n Socket Read Zero closing socket\n");
        close(connfd);
        //sleep(1);
     }
    if(SocketLive)
	printf("\n SocketLive is true\n");
    return (void*)1;
}

void *StartServerSocket(void *arg)
{
	int* ExitState = arg;
	int listenfd = 0, connfd = 0,RLength = 0;
    struct sockaddr_in serv_addr; 

    char sendBuff[64];
    char recBuff[64];
    //time_t ticks; 
    bool SocketLive=0;

    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
    memset(sendBuff, '0', sizeof(sendBuff)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(40000); 

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)); 

    listen(listenfd, 10); 

    while(*ExitState)
    {
        connfd = accept(listenfd, (struct sockaddr*)NULL, NULL); 
		printf("\n new connect \n");

		//ticks = time(NULL);
		strcpy(sendBuff,"Dexter Connect Service Connected\n");
		//snprintf(sendBuff, sizeof(sendBuff), "%.24s\r\n", ctime(&ticks));
		//write(connfd, sendBuff, strlen(sendBuff)); 
		SocketLive=TRUE;
		while(SocketLive==TRUE)
		{
			while ( (RLength = recv (connfd,recBuff,sizeof(recBuff),0 )) > 0)
			{
				//recBuff[RLength]=0;
				ProcessServerReceiveData(recBuff);
				SocketLive = ProcessServerSendData(sendBuff);
				write (connfd,sendBuff,sizeof(sendBuff)); 
			}
		}
		printf("\n Socket Read Zero closing socket\n");
        close(connfd);

    }
    if(SocketLive)
	printf("\n SocketLive is true\n");
    return (void*)1;
}
bool ProcessServerReceiveData(char *recBuff)
{
	struct BotPossition MyBot;
	memcpy(&MyBot,recBuff,sizeof(MyBot));
	printf("Server %d %d %d %d %d \n",MyBot.base,MyBot.end,MyBot.pivot,MyBot.angle,MyBot.rotate);
	mapped[FINE_ADJUST_BASE]=MyBot.base;
	mapped[FINE_ADJUST_END]=MyBot.end;
	mapped[FINE_ADJUST_PIVOT]=MyBot.pivot;
	mapped[FINE_ADJUST_ANGLE]=MyBot.angle;
	mapped[FINE_ADJUST_ROT]=MyBot.rotate;

	//MoveRobot(MyBot.base,MyBot.end,MyBot.pivot,MyBot.angle,MyBot.rotate,BLOCKING_MOVE);
/*	Cycle++;
	if(Cycle>100)
		return FALSE;
	else	
		return TRUE;*/
	/*rm warnning by hui*/
	return TRUE;
}
bool ProcessServerSendData(char *sendBuff)
{
	
	struct BotPossition MyBot;
	//static int Cycle = 0;
	MyBot.base=getNormalizedInput(BASE_POSITION_DELTA);
	MyBot.end=getNormalizedInput(END_POSITION_DELTA);
	MyBot.pivot=getNormalizedInput(PIVOT_POSITION_DELTA);
	MyBot.angle=getNormalizedInput(ANGLE_POSITION_DELTA);
	MyBot.rotate=getNormalizedInput(ROT_POSITION_DELTA);
	memcpy(sendBuff,&MyBot,sizeof(MyBot));
//	
//	sprintf(sendBuff, "%d \n", Cycle);
//	Cycle++;
	return TRUE;
}
bool ProcessServerSendDataDDE(char *sendBuff,char *recBuff)
{
	int i;
	int *sendBuffReTyped;
	const char delimiters[] = " ,";
	char *token;

/*	for(i=0;i<25;i++)
	{
		printf("%c",recBuff[i]);
	}*/
//	if(recBuff[0]=="g")
	token = strtok (recBuff, delimiters);

	{
		//printf("returning heartbeat\n");
		sendBuffReTyped=(int *)sendBuff;
		sendBuffReTyped[0]=atoi(token);
		token = strtok (NULL, delimiters);
		//printf("\n %s \n",token);
		sendBuffReTyped[1]=token[0];
		sendBuffReTyped[2]=DexError;
		
		for(i=3;i<48+3;i++)
		{
			sendBuffReTyped[i]=getNormalizedInput(i+INPUT_OFFSET-3);
		}
		return TRUE;
	}
	return FALSE;
}
bool ProcessServerReceiveDataDDE(char *recBuff)
{
	//struct BotPossition MyBot;
	char CmdString[255];
	int i,FoundStart=0,j=0;
	bool GotDelim=FALSE;
	//memcpy(&MyBot,recBuff,sizeof(MyBot));
	//printf("Server %d %d %d %d %d \n",MyBot.base,MyBot.end,MyBot.pivot,MyBot.angle,MyBot.rotate);
	//mapped[FINE_ADJUST_BASE]=MyBot.base;
	//mapped[FINE_ADJUST_END]=MyBot.end;
	//mapped[FINE_ADJUST_PIVOT]=MyBot.pivot;
	//mapped[FINE_ADJUST_ANGLE]=MyBot.angle;
	//mapped[FINE_ADJUST_ROT]=MyBot.rotate;

	//MoveRobot(MyBot.base,MyBot.end,MyBot.pivot,MyBot.angle,MyBot.rotate,BLOCKING_MOVE);
	for(i=0;i<255;i++)
	{
		if(FoundStart==1)
		{
			CmdString[j]=recBuff[i];
			j++;
		}
		if(recBuff[i]==' ')
		{
			FoundStart=1;
		}
		if(recBuff[i]==0x3b)
		{
			recBuff[i]=0;
			GotDelim=TRUE;
			break;
		}
	}
	if(GotDelim==FALSE)
	{
		DexError=2;
		printf("%s",recBuff);
		return FALSE;
	}
	CmdString[j]=0;
	printf("\n%s \n",recBuff);
	DexError=ParseInput(CmdString);
	printf("\nsent parse\n");
	return TRUE;
}

void *StartClientSocket()
{
    int sockfd = 0, n = 0;
    //int j;
    char recvBuff[64];
    char sendBuff[64];
    struct sockaddr_in serv_addr; 
    int Cycle = 100;
    struct BotPossition MyBot;
    float DiffDelta;
    //float ForceScale=0;

    memset(recvBuff, '0',sizeof(recvBuff));
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Error : Could not create socket \n");
        return (void*)1;
    }

    memset(&serv_addr, '0', sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(40000);
	RemoteRobotAdd[strcspn(RemoteRobotAdd,"\n")]= 0;

    if(inet_pton(AF_INET, RemoteRobotAdd/*"192.168.1.145"*/, &serv_addr.sin_addr)<=0)
    {
        printf("\n inet_pton error occured\n");
	return (void*)1;
    } 
    if( connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
       printf("\n Error : Connect Failed \n");
       return (void*)1;
    } 
//	printf("\n Socket connect\n");
//	while(1)//for(j=0;j<1000;j++)
	{
		//printf("\n next\n");
		while( (n = recv(sockfd, recvBuff, sizeof(recvBuff),0 ) ) > 0)
		{
			//recvBuff[n] = 0;
			//printf("Socket Read\n");
			//printf("%s ",recvBuff);

			if(n < 0)
			{
				printf("\n Read error \n");
			} 
			memcpy(&MyBot,recvBuff,sizeof(MyBot));
			/*mapped[FORCE_BIAS_BASE]=(int)(((float)MyBot.base)*ForceScale);
			mapped[FORCE_BIAS_END]=(int)(((float)MyBot.end)*ForceScale);
			mapped[FORCE_BIAS_PIVOT]=(int)(((float)MyBot.pivot)*ForceScale);
			mapped[FORCE_BIAS_ANGLE]=(int)(((float)MyBot.angle)*ForceScale);
			mapped[FORCE_BIAS_ROT]=(int)(((float)MyBot.rotate)*ForceScale);*/
			//printf("%d %d %d %d %d \n",MyBot.base,MyBot.end,MyBot.pivot,MyBot.angle,MyBot.rotate);
			DiffDelta=1.5;
			MyBot.base=SlaveBotPos.base + ((GetAxisCurrent(0) - SetHome.base)/MSMoveRatio.base);
			MyBot.end=SlaveBotPos.end + ((GetAxisCurrent(1) - SetHome.end)/MSMoveRatio.end);
			MyBot.pivot=SlaveBotPos.pivot + ((GetAxisCurrent(2) - SetHome.pivot)/MSMoveRatio.pivot);
			MyBot.angle=SlaveBotPos.angle + (((GetAxisRemote(3) - SetHome.angle)/MSMoveRatio.angle)*DiffDelta);
			MyBot.rotate=SlaveBotPos.rotate + (((GetAxisCurrent(4) - SetHome.rotate)/MSMoveRatio.rotate)*DiffDelta);
/*			MyBot.base=0;
			MyBot.end=0;
			MyBot.pivot=0;
			MyBot.angle=0;
			MyBot.rotate=0;*/

			/*
			SlaveBotPos.base=MyBot.base;
			SlaveBotPos.end=MyBot.base;
			SlaveBotPos.pivot=MyBot.base;
			SlaveBotPos.angle=MyBot.base;
			SlaveBotPos.rotate=MyBot.base;
			*/
			
			//usleep(25000);

			/*MyBot.base=BASE_POSITION_FORCE_DELTA;
			MyBot.end=12300;
			MyBot.pivot=5123;
			MyBot.angle=10;
			MyBot.rotate=15;*/
			memcpy(sendBuff,&MyBot,sizeof(MyBot));
			Cycle++;
			write(sockfd,sendBuff,sizeof(sendBuff));
		}
	}
       return (void*)1;
}
int GetAxisCurrent(int Axis)
{
	int AngleCorrected,AngleAt;
	int EndAxisAt;
	float EndAxisCorrectionFactor=DiffCorrectionFactor/16;//.06666;
	if(Axis==3)
	{
		//calculate the correction factor for the angle
		AngleAt=getNormalizedInput(BASE_POSITION_AT+Axis)+getNormalizedInput(BASE_POSITION_FORCE_DELTA+Axis);
		EndAxisAt=GetAxisCurrent(1);
		AngleCorrected=AngleAt+(EndAxisAt*EndAxisCorrectionFactor);
		return AngleCorrected;
	}
	return getNormalizedInput(BASE_POSITION_AT+Axis)+getNormalizedInput(BASE_POSITION_FORCE_DELTA+Axis);
}
int GetAxisRemote(int Axis)
{
	int AngleCorrected,AngleAt;
	int EndAxisAt;
	float EndAxisCorrectionFactor=.60682/16;//.06666;
	if(Axis==3)
	{
		//calculate the correction factor for the angle
		AngleAt=getNormalizedInput(BASE_POSITION_AT+Axis)+getNormalizedInput(BASE_POSITION_FORCE_DELTA+Axis);
		EndAxisAt=GetAxisCurrent(1);
		AngleCorrected=AngleAt+(EndAxisAt*EndAxisCorrectionFactor);
		return AngleCorrected;
	}
	return getNormalizedInput(BASE_POSITION_AT+Axis)+getNormalizedInput(BASE_POSITION_FORCE_DELTA+Axis);
}
/* Initialize new terminal i/o settings */
void initTermios(int echo) 
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  new = old; /* make new settings same as old settings */
  new.c_lflag &= ~ICANON; /* disable buffered i/o */
  new.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &new); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void) 
{
  tcsetattr(0, TCSANOW, &old);
}

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

int FindIndex(int Axis,int Start,int Length,int Delay)
{
    //int i;
    int j,k,ADVal,AvgCOS,AvgSIN;
    switch(Axis)
    {
	case 0  :
	    MoveRobot(Start,0,0,0,0,BLOCKING_MOVE);
	    break; 
	case 1  :
	    MoveRobot(0,Start,0,0,0,BLOCKING_MOVE);
	    break; 
	case 2  :
	    MoveRobot(0,0,Start,0,0,BLOCKING_MOVE);
	    break; 
	case 3  :
	    MoveRobot(0,0,0,Start,0,BLOCKING_MOVE);
	    break; 
	case 4  :
	    MoveRobot(0,0,0,0,Start,BLOCKING_MOVE);
	    break; 
    }
    for(k=0;k<abs(Length);k++) 
    {	
	if(Length>0)
	    mapped[Axis]=Start+k;
	else	
	    mapped[Axis]=Start-k;
	for(j=0;j<Delay;j++)
	{	
	    ADVal=mapped[ADLookUp[Axis]];
	    AvgSIN=AvgSIN+ADVal;
	    AvgCOS=AvgCOS+mapped[ADLookUp[Axis]+1];
	}	
	AvgSIN=AvgSIN/Delay;
	AvgCOS=AvgCOS/Delay;
	//printf("\n SIN %x COS %x",AvgSIN,AvgCOS);
	if((AvgCOS>XLowBound[Axis]) && (AvgCOS<XHighBound[Axis]) && (AvgSIN>YLowBound[Axis]) && (AvgSIN<YHighBound[Axis]))
	{
	    // we found the index 
	    return 0;
	}
    }
    return 1; // did not find index
}
void SetNewBotRef()
{
	SlaveBotPos.base=SlaveBotPos.base + ((GetAxisCurrent(0) - SetHome.base)/MSMoveRatio.base);
	SlaveBotPos.end=SlaveBotPos.end + ((GetAxisCurrent(1) - SetHome.end)/MSMoveRatio.end);
	SlaveBotPos.pivot=SlaveBotPos.pivot + ((GetAxisCurrent(2) - SetHome.pivot)/MSMoveRatio.pivot);
	SlaveBotPos.angle=SlaveBotPos.angle + ((GetAxisRemote(3) - SetHome.angle)/MSMoveRatio.angle);
	SlaveBotPos.rotate=SlaveBotPos.rotate + ((GetAxisCurrent(4) - SetHome.rotate)/MSMoveRatio.rotate);
	SetHome.base=GetAxisCurrent(0);
	SetHome.end=GetAxisCurrent(1);
	SetHome.pivot=GetAxisCurrent(2);
	SetHome.angle=GetAxisRemote(3);
	SetHome.rotate=GetAxisCurrent(4);
	
}
void* CapturePoints(void *arg)
{
    //unsigned long i = 0;
	int a1,a2,a3,a4,a5,f1,f2,f3,f4,f5,BGM=10,EmbedRec=1,RecState=0,Length=0,GripperToggle=0,ServoSet=180;
	const char delimiters[] = ".";

	char c,*FileNameBase,*tmpSt,RecordFilename[256],iString[256];
	//char *FileNameExt;
//    pthread_t id = pthread_self();
	initTermios(0);
	f1=0;
	mapped[FINE_ADJUST_BASE]=f1;
	f2 =0;
	mapped[FINE_ADJUST_END]=f2;
	f3=0;
	mapped[FINE_ADJUST_PIVOT]=f3;
	f4=0;
	mapped[FINE_ADJUST_ANGLE]=f4;
	f5=0;
	mapped[FINE_ADJUST_ROT]=f5;


	mapped[COMMAND_REG]=12448;
	printf("/n Thread running");
	if(RunMode==1)
		forceMode=3;
	c=' ';
	//mapped[MAX_ERROR]=(2000 ^ 6000);
	while(c!='k') // kill
	{
		mapped[FINE_ADJUST_BASE]=f1;
		mapped[FINE_ADJUST_END]=f2;
		mapped[FINE_ADJUST_PIVOT]=f3;
		mapped[FINE_ADJUST_ANGLE]=f4;
		mapped[FINE_ADJUST_ROT]=f5;
		c=getchar();
		if(c=='l')  // learn movement
		{
			if(RecState==0)
			{
				RecState=1;
				strcpy(iString,CptA.FileName);
				FileNameBase = strtok (iString, delimiters);
				//FileNameExt=strtok (NULL, delimiters);
				strcpy(RecordFilename,FileNameBase);
				strcat(RecordFilename,"ERM");
				asprintf(&tmpSt, "%d", EmbedRec++);
				strcat(RecordFilename,tmpSt);
				free(tmpSt);
				strcat(RecordFilename,".dta");
				a1=getNormalizedInput(BASE_POSITION_AT)+getNormalizedInput(BASE_POSITION_FORCE_DELTA)+f1;
				a2=getNormalizedInput(END_POSITION_AT)+getNormalizedInput(END_POSITION_FORCE_DELTA)+f2;
				a3=getNormalizedInput(PIVOT_POSITION_AT)+getNormalizedInput(PIVOT_POSITION_FORCE_DELTA)+f3;
				a4=getNormalizedInput(ANGLE_POSITION_AT)+getNormalizedInput(ANGLE_POSITION_FORCE_DELTA)+f4;
				a5=getNormalizedInput(ROT_POSITION_AT)+getNormalizedInput(ROT_POSITION_FORCE_DELTA)+f5;
				fprintf(CptA.fp,"a %d,%d,%d,%d,%d\n",a1,a2,a3,a4,a5);
				printf("\na %d,%d,%d,%d,%d",a1,a2,a3,a4,a5);
				fprintf(CptA.fp,"o %s\n",RecordFilename);
				mapped[REC_PLAY_TIMEBASE]=5;
				mapped[REC_PLAY_CMD]=CMD_RESET_RECORD;
				mapped[REC_PLAY_CMD]=0;				
				mapped[REC_PLAY_CMD]=CMD_RECORD; // start recording

				//InitCaptureMovement(RecordFilename);
			}
			else
			{
				RecState=0;
				a1=getNormalizedInput(BASE_POSITION_AT)+getNormalizedInput(BASE_POSITION_FORCE_DELTA)+f1;
				a2=getNormalizedInput(END_POSITION_AT)+getNormalizedInput(END_POSITION_FORCE_DELTA)+f2;
				a3=getNormalizedInput(PIVOT_POSITION_AT)+getNormalizedInput(PIVOT_POSITION_FORCE_DELTA)+f3;
				a4=getNormalizedInput(ANGLE_POSITION_AT)+getNormalizedInput(ANGLE_POSITION_FORCE_DELTA)+f4;
				a5=getNormalizedInput(ROT_POSITION_AT)+getNormalizedInput(ROT_POSITION_FORCE_DELTA)+f5;
				//fprintf(CptA.fp,"a %d,%d,%d,%d,%d\n",a1,a2,a3,a4,a5);
				printf("\na %d,%d,%d,%d,%d",a1,a2,a3,a4,a5);
				//fprintf(CptA.fp,"o %s\n",RecordFilename);

				mapped[REC_PLAY_CMD]=0; // stop recording
				Length=mapped[RECORD_BLOCK_SIZE];
				ReadDMA(0x3f000000,Length,RecordFilename);
				mapped[REC_PLAY_CMD]=CMD_RESET_RECORD;
				mapped[REC_PLAY_CMD]=0;
			}
			
		}
		if(c=='+')
		{
			BGM=BGM+10;
			SetNewBotRef();
			/*SetHome.base = SlaveBotPos.base;
			SetHome.end = SlaveBotPos.end;
			SetHome.pivot = SlaveBotPos.pivot;
			SetHome.angle = SlaveBotPos.angle;
			SetHome.rotate = SlaveBotPos.rotate;*/
			
			MSMoveRatio.base++;
			MSMoveRatio.end++;
			MSMoveRatio.pivot++;
			MSMoveRatio.angle++;
			MSMoveRatio.rotate++;
			printf("\n big move %d MasterSlave Move ratio %d\n",BGM,MSMoveRatio.base);
		}
		if(c=='-')
		{
			BGM=BGM-10;
			if(BGM < 10)
			{
				BGM=10;
			}
			if(MSMoveRatio.base != 1)
			{
				/*SetHome.base = SlaveBotPos.base;
				SetHome.end = SlaveBotPos.end;
				SetHome.pivot = SlaveBotPos.pivot;
				SetHome.angle = SlaveBotPos.angle;
				SetHome.rotate = SlaveBotPos.rotate;*/
				SetNewBotRef();
				MSMoveRatio.base--;
				MSMoveRatio.end--;
				MSMoveRatio.pivot--;
				MSMoveRatio.angle--;
				MSMoveRatio.rotate--;
			}
			printf("\n big move %d MasterSlave Move ratio %d\n",BGM,MSMoveRatio.base);
		}
		if(c=='h')
		{
			SetNewBotRef();
			/*
			SetHome.base = getNormalizedInput(BASE_POSITION_AT)+getNormalizedInput(BASE_POSITION_FORCE_DELTA);
			SetHome.end = getNormalizedInput(END_POSITION_AT)+getNormalizedInput(END_POSITION_FORCE_DELTA);
			SetHome.pivot = getNormalizedInput(PIVOT_POSITION_AT)+getNormalizedInput(PIVOT_POSITION_FORCE_DELTA);
			SetHome.angle = getNormalizedInput(ANGLE_POSITION_AT)+getNormalizedInput(ANGLE_POSITION_FORCE_DELTA);
			SetHome.rotate = getNormalizedInput(ROT_POSITION_AT)+getNormalizedInput(ROT_POSITION_FORCE_DELTA);
			*/
		}	
		if(c=='p')
		{
			a1=getNormalizedInput(BASE_POSITION_AT)+getNormalizedInput(BASE_POSITION_FORCE_DELTA)+f1;
			a2=getNormalizedInput(END_POSITION_AT)+getNormalizedInput(END_POSITION_FORCE_DELTA)+f2;
			a3=getNormalizedInput(PIVOT_POSITION_AT)+getNormalizedInput(PIVOT_POSITION_FORCE_DELTA)+f3;
			a4=getNormalizedInput(ANGLE_POSITION_AT)+getNormalizedInput(ANGLE_POSITION_FORCE_DELTA)+f4;
			a5=getNormalizedInput(ROT_POSITION_AT)+getNormalizedInput(ROT_POSITION_FORCE_DELTA)+f5;
			fprintf(CptA.fp,"a %d,%d,%d,%d,%d\n",a1,a2,a3,a4,a5);
			fprintf(CptA.fp,"S EERoll,%d\n",ServoSet);
			printf("\na %d,%d,%d,%d,%d",a1,a2,a3,a4,a5);
		}
		if(c==' ')
		{ 
			fprintf(CptA.fp,"z 500000\n");
			printf("\n insert dwell .5 seconds\n");
		}
		if(c=='g')
		{ 
			if(GripperToggle==0)
			{
				fprintf(CptA.fp,"S GripperMotor,1\n");
				printf("\n GripperMotor On\n");
				GripperToggle=1;
				SetGripperMotor(GripperToggle);
			}
			else
			{
				fprintf(CptA.fp,"S GripperMotor,0\n");
				printf("\n GripperMotor Off\n");
				GripperToggle=0;
				SetGripperMotor(GripperToggle);
			}
		}
		if(c=='t')
		{
			if(forceMode==1)
			{
				mapped[DIFF_FORCE_SPEED_FACTOR_ANGLE]=0;
				mapped[DIFF_FORCE_SPEED_FACTOR_ROT]=0;
				mapped[PID_ADDRESS]=3;
				mapped[PID_P]=DEFAULT_PID_SETTING_PY;
				mapped[PID_ADDRESS]=4;
				mapped[PID_ADDRESS]=0;
				mapped[PID_P]=0;
				mapped[PID_ADDRESS]=1;
				mapped[PID_ADDRESS]=2;
				mapped[SPEED_FACTORA]=DEF_SPEED_FACTOR_A ;
				forceMode=2;
				printf("\n Move XYZ yo \n");
				
			}else if(forceMode==0)
			{

				mapped[DIFF_FORCE_SPEED_FACTOR_ANGLE]=DEF_SPEED_FACTOR_DIFF;
				mapped[DIFF_FORCE_SPEED_FACTOR_ROT]=DEF_SPEED_FACTOR_DIFF;
				mapped[PID_ADDRESS]=3;
				mapped[PID_P]=0;
				mapped[PID_ADDRESS]=4;
				mapped[PID_ADDRESS]=0;
				mapped[PID_P]=DEFAULT_PID_SETTING_XYZ;
				mapped[PID_ADDRESS]=1;
				mapped[PID_ADDRESS]=2;
				mapped[SPEED_FACTORA]=0;
				forceMode=1;
				
				printf("\n Move Pitch and Yaw \n");
				
			}else if(forceMode==2)
			{
				mapped[DIFF_FORCE_SPEED_FACTOR_ANGLE]=0;
				mapped[DIFF_FORCE_SPEED_FACTOR_ROT]=0;
				mapped[PID_ADDRESS]=0;
				mapped[PID_P]=DEFAULT_PID_SETTING_XYZ;
				mapped[PID_ADDRESS]=1;
				mapped[PID_ADDRESS]=2;
				mapped[PID_ADDRESS]=3;
				
//				mapped[PID_P]=0; //DEFAULT_PID_SETTING_PY;
				mapped[PID_P]=DEFAULT_PID_SETTING_PY;
				mapped[PID_ADDRESS]=4;
				mapped[SPEED_FACTORA]=0;
				if(RunMode==1)
					forceMode=3;
				else
					forceMode=0;
				printf("\n Move fine adjust all \n");
			}else if(forceMode==3)
			{
				//printf("\n Move all \n");
				mapped[DIFF_FORCE_SPEED_FACTOR_ANGLE]=DEF_SPEED_FACTOR_DIFF;
				mapped[DIFF_FORCE_SPEED_FACTOR_ROT]=DEF_SPEED_FACTOR_DIFF;
				mapped[PID_P]=0;
				mapped[PID_ADDRESS]=3;
				mapped[PID_ADDRESS]=4;
				mapped[PID_ADDRESS]=0;
				mapped[PID_ADDRESS]=1;
				mapped[PID_ADDRESS]=2;
				mapped[SPEED_FACTORA]=DEF_SPEED_FACTOR_A ;
				forceMode=2;
				printf("\n Move All \n");
			}
		}
		if(c=='q')
		{
			f1--;
		}
		if(c=='w')
		{
			f1++;
		}
		if(c=='a')
		{
			f2--;
		}
		if(c=='s')
		{
			f2++;
		}
		if(c=='z')
		{
			f3--;
		}
		if(c=='x')
		{
			f3++;
		}
		if(c=='e')
		{
			f4--;
		}
		if(c=='r')
		{
			f4++;
		}
		if(c=='d')
		{
			f5--;
		}
		if(c=='f')
		{
			f5++;
		}
		if(c=='Q')
		{
			f1=f1-BGM;
		}
		if(c=='W')
		{
			f1=f1+BGM;
		}
		if(c=='A')
		{
			f2=f2-BGM;
		}
		if(c=='S')
		{
			f2=f2+BGM;
		}
		if(c=='Z')
		{
			f3=f3-BGM;
		}
		if(c=='X')
		{
			f3=f3+BGM;
		}
		if(c=='E')
		{
			f4=f4-BGM;
		}
		if(c=='R')
		{
			f4=f4+BGM;
		}
		if(c=='D')
		{
			f5=f5-BGM;
		}
		if(c=='F')
		{
			f5=f5+BGM;
		}
		if(c==',')
		{
			ServoSet=ServoSet+1;
			SetGripperRoll(ServoSet);
		}
		if(c=='.')
		{
			ServoSet=ServoSet-1;
			SetGripperRoll(ServoSet);
		}
	}
	mapped[MAX_ERROR]=(2000 ^ 4000);
	mapped[DIFF_FORCE_SPEED_FACTOR_ANGLE]=0;
	mapped[DIFF_FORCE_SPEED_FACTOR_ROT]=0;
	mapped[PID_ADDRESS]=0;
	mapped[PID_P]=DEFAULT_PID_SETTING_XYZ;
	mapped[PID_ADDRESS]=1;
	mapped[PID_ADDRESS]=2;
	mapped[PID_ADDRESS]=3;
	mapped[PID_P]=DEFAULT_PID_SETTING_PY;
	mapped[PID_ADDRESS]=4;
	mapped[SPEED_FACTORA]=0;
	
	
	mapped[FINE_ADJUST_BASE]=0;
	mapped[FINE_ADJUST_END]=0;
	mapped[FINE_ADJUST_PIVOT]=0;
	mapped[FINE_ADJUST_ANGLE]=0;
	mapped[FINE_ADJUST_ROT]=0;
        mapped[COMMAND_REG]=3158688;	
	printf("\nAll done\n");
	fprintf(CptA.fp,"x\n");
			
	fclose(CptA.fp);
	resetTermios();
	
    return NULL;
}
int InitCapturePoints(char *FileName)
{
    //int i = 0;
    //int err;
    CptA.FileName=FileName;
    CptA.fp=fopen(FileName, "w");
    if(CptA.fp!=0)
    {
	CapturePoints((void*)&CptA);
    }

    /*		err = pthread_create(&(tid[i]), NULL, &CapturePoints, (void*)&CptA);
		if (err != 0)
		{
		printf("\ncan't create thread :[%s]", strerror(err));
		return 1;
		}
		else
		{
		printf("\n Begin Program\n");
		}
		}*/
    return 0;
}


int InitCaptureMovement(char *FileName)
{
    int Length=0;
    char c;
    //initTermios(0);
    //	CptMove.FileName=FileName;
    //	CptMove.fp=fopen(FileName, "w");
    //	if(CptMove.fp!=0)
    {
	//CaptureMovement((void*)&CptMove);
	mapped[REC_PLAY_CMD]=CMD_RESET_RECORD;
	mapped[REC_PLAY_CMD]=CMD_RECORD; // start recording
	c=' ';
	while(c!='k') // kill
	{
	    c=getchar();

	}
	mapped[REC_PLAY_CMD]=0; // stop recording

	Length=mapped[RECORD_BLOCK_SIZE];
	ReadDMA(0x3f000000,Length,FileName);


    }
    //resetTermios();
    return 0;
}


int fixedPointCV(float Val,int whole,int fract)
{
	return 0;
}
void setDefaults(int State)
{

	int i,ForceFelt,HiBoundry,LowBoundry,BoundryACC;
	//char c;
    FILE *CentersFile,*RemoteRobotAddress,*DiffFile;
	int HexValue;
	int IntFloat;
	float *fConvert=(float *)(&IntFloat);
	
	
	DiffFile = fopen("DiffCorrectionFactor.txt", "rs");
	if(DiffFile!=NULL)
	{
		fscanf (DiffFile, "%f", &DiffCorrectionFactor);
		//fgets(DiffFile, sizeof(RemoteRobotAdd)+1, RemoteRobotAddress);
		//fscanf(RemoteRobotAddress,"%[^\n]",RemoteRobotAdd);
		printf("%f \n",DiffCorrectionFactor);
		fclose(DiffFile);
	}
	DiffFile = fopen("AngleBoundry.txt", "rs");
	if(DiffFile!=NULL)
	{
		fscanf (DiffFile, "%i %i", &AngleHIBoundry,&AngleLOWBoundry);
		//fgets(DiffFile, sizeof(RemoteRobotAdd)+1, RemoteRobotAddress);
		//fscanf(RemoteRobotAddress,"%[^\n]",RemoteRobotAdd);
		printf("Angle High boundry%i    Angle Low Boundry %i \n",AngleHIBoundry,AngleLOWBoundry);
		fclose(DiffFile);
	}

	
	RemoteRobotAddress = fopen("RemoteRobotAddress.txt", "rs");
	if(RemoteRobotAddress!=NULL)
	{
		fgets(RemoteRobotAdd, sizeof(RemoteRobotAdd)+1, RemoteRobotAddress);
		//fscanf(RemoteRobotAddress,"%[^\n]",RemoteRobotAdd);
		printf("%s \n",RemoteRobotAdd);
		fclose(RemoteRobotAddress);
	}
	mapped[FORCE_BIAS_BASE]=0;
	mapped[FORCE_BIAS_END]=0;
	mapped[FORCE_BIAS_PIVOT]=0;
	mapped[FORCE_BIAS_ANGLE]=0;
	mapped[FORCE_BIAS_ROT]=0;

	mapped[ACCELERATION_MAXSPEED]=ACCELERATION_MAXSPEED_DEF;
	maxSpeed=(ACCELERATION_MAXSPEED_DEF) & 0b00000000000011111111111111111111;
	coupledAcceleration=((ACCELERATION_MAXSPEED_DEF) & 0b00000011111100000000000000000000) >> 20;

	if(State==1)
	{


		mapped[BASE_FORCE_DECAY]=000000;
		mapped[END_FORCE_DECAY]=000000;
		mapped[PIVOT_FORCE_DECAY]=000000;
		mapped[ANGLE_FORCE_DECAY]=000000;
		mapped[ROTATE_FORCE_DECAY]=000000;

		mapped[ACCELERATION_MAXSPEED]=ACCELERATION_MAXSPEED_DEF;
		maxSpeed=(ACCELERATION_MAXSPEED_DEF) & 0b00000000000011111111111111111111;
		coupledAcceleration=((ACCELERATION_MAXSPEED_DEF) & 0b00000011111100000000000000000000) >> 20;
	

		mapped[REC_PLAY_TIMEBASE]=5;
		
    CentersFile = fopen("AdcCenters.txt", "rs");

    //read file into array
	if(CentersFile!=NULL)
	{
		fscanf(CentersFile, "%x", &HexValue);
		printf("%x \n",HexValue);
		mapped[BASE_SIN_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[BASE_COS_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[END_SIN_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[END_COS_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[PIVOT_SIN_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[PIVOT_COS_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[ANGLE_SIN_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[ANGLE_COS_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[ROT_SIN_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[ROT_COS_CENTER]=HexValue;
		fclose(CentersFile);
	}
    

/*		mapped[BASE_SIN_CENTER]=0x7f80000;
		mapped[BASE_COS_CENTER]=0x7f80000;
		mapped[END_SIN_CENTER]=0x7f80000;
		mapped[END_COS_CENTER]=0x07f80000;
		mapped[PIVOT_SIN_CENTER]=0x4a00000;
		mapped[PIVOT_COS_CENTER]=0x6500000;
		mapped[ANGLE_SIN_CENTER]=0x7890000;
		mapped[ANGLE_COS_CENTER]=0x5080000;
		mapped[ROT_SIN_CENTER]=0x5200000;
		mapped[ROT_COS_CENTER]=0x6f00000;*/

//		mapped[BASE_SIN_CENTER]=0x7f80000;
//		mapped[BASE_COS_CENTER]=0x7f80000;
//		mapped[END_SIN_CENTER]=0x7f80000;
//		mapped[END_COS_CENTER]=0x07f80000;
//		mapped[PIVOT_SIN_CENTER]=0x6500000;
//		mapped[PIVOT_COS_CENTER]=0x4a00000;
//		mapped[ANGLE_SIN_CENTER]=0x7f80000;
//		mapped[ANGLE_COS_CENTER]=0x4900000;
//		mapped[ROT_SIN_CENTER]=0x6f00000;
//		mapped[ROT_COS_CENTER]=0x5200000;


		mapped[DIFF_FORCE_BETA ]=0x0102;

	// set up PID defaults
	
		mapped[PID_P]=DEFAULT_PID_SETTING_XYZ;
		mapped[PID_I]=0;
		mapped[PID_D]=0;
		mapped[PID_DELTATNOT]=0;
		mapped[PID_DELTAT]=0;
		mapped[PID_ADDRESS]=0;
		mapped[PID_ADDRESS]=1;
		mapped[PID_ADDRESS]=2;
		mapped[PID_ADDRESS]=3;
		mapped[PID_P]=DEFAULT_PID_SETTING_PY;
		mapped[PID_ADDRESS]=4;
		
		// set boundries  
		BaseBoundryHigh=90000;
		BaseBoundryLow=-90000;
		EndBoundryHigh=130000;
		EndBoundryLow=-130000;
		PivotBoundryHigh=70000;
		PivotBoundryLow=-70000;
		AngleBoundryHigh=DEFAULT_ANGLE_BOUNDRY_HI;
		AngleBoundryLow=DEFAULT_ANGLE_BOUNDRY_LOW;
		RotateBoundryHigh=3000;
		RotateBoundryLow=-3000;
		
		
		mapped[BOUNDRY_BASE]=0x2bf2d40e; //
		mapped[BOUNDRY_END]=0x3f7ac086;  //
		mapped[BOUNDRY_PIVOT]=0x222eddd2; //1
		HiBoundry=AngleHIBoundry/8;
		LowBoundry=AngleLOWBoundry/8;
		BoundryACC=(HiBoundry << 16) | (0x0000ffff & LowBoundry);
		mapped[BOUNDRY_ANGLE]=BoundryACC;// 0x01f4fe0c; //
		printf("Angle boundry pack %x ",BoundryACC);
		
		mapped[BOUNDRY_ROT]=((RotateBoundryHigh/8) << 16) | (0X0000FFFF & (RotateBoundryLow/8));

		
		
		
		
		/*EndAxisLoc=GetAxisCurrent(1);
		AngleCorrectionFactor=EndAxisLoc*EndBoundryScale;
		Hbd=AngleBoundryHigh+AngleCorrectionFactor;
		Lbd=AngleBoundryLow+AngleCorrectionFactor;
		HiBoundry=Hbd/8;
		LowBoundry=Lbd/8;
		BoundryACC=(HiBoundry << 16) | (0x0000ffff & LowBoundry);
		//printf("%x %d %d \n",BoundryACC,Hbd,Lbd);
		mapped[BOUNDRY_ANGLE]=BoundryACC;//0x01f4 fe0c; //*/


//		mapped[BOUNDRY_BASE]=120000; //0x3a98c568
//		mapped[BOUNDRY_END]=130000;  //0x3f7ac086
//		mapped[BOUNDRY_PIVOT]=75000; //0x249fdb61
//		mapped[BOUNDRY_ANGLE]=5000; //0x01f4fe0c
//		mapped[BOUNDRY_ROT]=7000;   //0x03e8fc18

		mapped[SPEED_FACTORA]=0;
		mapped[SPEED_FACTORB]=0;
		mapped[MAX_ERROR]=(2000 ^ 6000);
		

		mapped[DIFF_FORCE_TIMEBASE]=140000; // this is currently remapped to MaxSpeed XYZ Force
		
		mapped[DIFF_FORCE_MAX_SPEED]=90000;
//		IntFloat)
		*fConvert=DiffCorrectionFactor;
		
		printf(" float value %x \n",IntFloat);
		//if(RunMode==1)
			mapped[DIFF_FORCE_ANGLE_COMPENSATE]=IntFloat;
		//else
		//	mapped[DIFF_FORCE_ANGLE_COMPENSATE]=0;//IntFloat;
		
		
		




	
		mapped[FRICTION_BASE]=0x100; // fixed point 16 bit == .1
		mapped[FRICTION_END]=0x100; // fixed point 16 bit == .1
		mapped[FRICTION_PIVOT]=0x100; // fixed point 16 bit == .1
		mapped[FRICTION_ANGLE]=0x50; // fixed point 16 bit == .1
		mapped[FRICTION_ROT]=0x50; // fixed point 16 bit == .1
	
		
		
	}
	if(State==2)
	{
		CmdVal=3158688;
		mapped[FINE_ADJUST_BASE]=0;
		mapped[FINE_ADJUST_END]=0;
		mapped[FINE_ADJUST_PIVOT]=0;
		mapped[FINE_ADJUST_ANGLE]=0;
		mapped[FINE_ADJUST_ROT]=0;
	}
	if(State==3)
	{
		
		printf("\nMonitor Force Started");
		while(1)
		{
			printf("\nTable Delta ");
			for(i=0;i<5;i++)
			{
				printf("%5d ",getNormalizedInput(BASE_POSITION_DELTA+i));
			}
			printf(" PID_DELTA ");
			for(i=0;i<5;i++)
			{
				printf("%5d ",getNormalizedInput(BASE_POSITION_PID_DELTA+i));
			}
			//printf("\nForce_DELTA ");
			//for(i=0;i<5;i++)
			//{
			//	printf("%5d ",getNormalizedInput(BASE_POSITION_FORCE_DELTA+i));
			//}
		}
		
	}
	if(State==4)
	{
		//initTermios(0);
		mapped[PID_ADDRESS]=3;
		mapped[PID_P]=0;
		mapped[PID_I]=0;
		mapped[PID_D]=0;
		mapped[PID_DELTATNOT]=0;
		mapped[PID_DELTAT]=0;
		mapped[PID_ADDRESS]=4;
		//c=' ';
		mapped[FINE_ADJUST_BASE]=0;
		mapped[FINE_ADJUST_END]=0;
		mapped[FINE_ADJUST_PIVOT]=0;
		mapped[FINE_ADJUST_ANGLE]=0;
		mapped[FINE_ADJUST_ROT]=0;
		
		printf("\nForce Protect Started");
		while(1) 
//		while(c!='k') // kill
		{
			//c=getchar();
			for(i=0;i<5;i++)
			{
				ForceFelt=getNormalizedInput(BASE_POSITION_PID_DELTA+i)-getNormalizedInput(BASE_POSITION_DELTA+i);
				printf("%5d ",ForceFelt);
				if(abs(ForceFelt-PositionAdjust[i]) >= ForceLimit[i])
				{
					if((ForceFelt-PositionAdjust[i]) < 0)
					{
						//PositionAdjust[i]=2*(abs((ForceFelt-PositionAdjust[i]))-ForceLimit[i]);						
					}
					else
					{
						//PositionAdjust[i]=2*-(abs((ForceFelt-PositionAdjust[i]))-ForceLimit[i]);
					}
				}
			}
			for(i=0;i<3;i++)
			{
				//mapped[FINE_ADJUST_BASE+i]=PositionAdjust[i];
				printf("%5d ",PositionAdjust[i]);
			}
			printf("\n");
			
			usleep(10000);
		}	
	//resetTermios();		
	}
	if(State==5)
	{
		
		printf("\nMonitor Accelerateion Force Started");
		while(1)
		{
			printf("\nSpeed Delta ");
			for(i=0;i<5;i++)
			{
				printf("%5d ",mapped[SLOPE_BASE_POSITION+i]);
			}
		}
		
	}
	if(State==6)
	{
		
		printf("\nMonitor Force offset Started");
		while(1)
		{
			//printf("\nSpeed Delta ");
			for(i=0;i<5;i++)
			{
				printf("%6d  ",GetAxisCurrent(i));
			}
			printf("\n");
		}
		
	}
	/*if(State==7)
	{
		
		//printf("\nMonitor Force offset Started");
		while(1)
		{
			EndAxisLoc=GetAxisCurrent(1);
			AngleCorrectionFactor=EndAxisLoc*EndBoundryScale;
			Hbd=DEFAULT_ANGLE_BOUNDRY_HI+AngleCorrectionFactor;
			Lbd=DEFAULT_ANGLE_BOUNDRY_LOW+AngleCorrectionFactor;
			HiBoundry=Hbd/8;
			LowBoundry=Lbd/8;
			BoundryACC=(HiBoundry << 16) | (0x0000ffff & LowBoundry);
			printf("%x %d %d \n",BoundryACC,Hbd,Lbd);
			//mapped[BOUNDRY_ANGLE]=BoundryACC;//0x01f4 fe0c; //
		}
		
	}*/
}


int HashInputCMD(char *s)
{

	if(s[0]=='r')
		return READ_CMD;
	if(s[0]=='w')
		return WRITE_CMD;
	if(s[0]=='x')
		return EXIT_CMD;
	if(s[0]=='s')
		return SLOWMOVE_CMD;
	if(s[0]=='a')
		return MOVEALL_CMD;
	if(s[0]=='d')
		return DMAREAD_CMD;
	if(s[0]=='t')
		return DMAWRITE_CMD;
	if(s[0]=='c')
		return CAPTURE_AD_CMD;
	if(s[0]=='f')
		return FIND_HOME_CMD;
	if(s[0]=='n')
		return FIND_INDEX_CMD;
	if(s[0]=='p')
		return FIND_HOME_REP_CMD;
	if(s[0]=='l')
		return LOAD_TABLES;
	if(s[0]=='i')
		return CAPTURE_POINTS_CMD;
	if(s[0]=='z')
		return SLEEP_CMD;
	if(s[0]=='m')
		return RECORD_MOVEMENT;
	if(s[0]=='o')
		return REPLAY_MOVEMENT;
	if(s[0]=='R')
		return MOVEALL_RELATIVE;
	if(s[0]=='S')
		return SET_PARAM;
	if(s[0]=='g')
		return SEND_HEARTBEAT;
	if(s[0]=='F')
		return SET_FORCE_MOVE_POINT;
	if(s[0]=='g')
		return HEART_BEAT;
	if(s[0]=='h')
		return HEART_BEAT;
	if(s[0]=='I')
		return WRITE_BUFFER_CMD;
	if(s[0]=='O')
		return READ_BUFFER_CMD;
	return 0;
}

int isNear(int a,int b,int range)
{
	if(abs(a-b)<=range)
		return TRUE;
	else
		return FALSE;
}
int getNormalizedInput(int param) 
{
	int val;
	val = mapped[param];
	if((val & 0x40000)!=0)
	{
		return (val | 0xfff80000);
	}
	else
	{
		return val;
	}
}



int WaitMoveGoal(int a1,int a2,int a3,int a4,int a5,int timeout)
{
    int b1,b2,b3,b4,b5,c1,c2,c3,c4,c5;
    //long            ms; // Milliseconds
    time_t          oldTime,newTime;  // Seconds
    struct timespec spec;

    b1=getNormalizedInput(BASE_POSITION_AT);
    b2=getNormalizedInput(END_POSITION_AT);
    b3=getNormalizedInput(PIVOT_POSITION_AT);
    b4=getNormalizedInput(ANGLE_POSITION_AT);
    b5=getNormalizedInput(ROT_POSITION_AT);

    clock_gettime(CLOCK_REALTIME, &spec);
    oldTime  = spec.tv_sec;
    //printf("\n%d %d %d %d %d\n",b1,b2,b3,b4,b5);	
    //printf("\n%d %d %d %d %d\n",a1,a2,a3,a4,a5);	
    //    printf("\nStart wait Goal");
    while(!isNear(a1,b1,50) || !isNear(a2,b2,50) || !isNear(a3,b3,50) || !isNear(a4,b4,50) || !isNear(a5,b5,50))
    {
	b1=getNormalizedInput(BASE_POSITION_AT);
	b2=getNormalizedInput(END_POSITION_AT);
	b3=getNormalizedInput(PIVOT_POSITION_AT);
	b4=getNormalizedInput(ANGLE_POSITION_AT);
	b5=getNormalizedInput(ROT_POSITION_AT);
	c1=getNormalizedInput(BASE_POSITION_DELTA);
	c2=getNormalizedInput(END_POSITION_DELTA);
	c3=getNormalizedInput(PIVOT_POSITION_DELTA);
	c4=getNormalizedInput(ANGLE_POSITION_DELTA);
	c5=getNormalizedInput(ROT_POSITION_DELTA);
	clock_gettime(CLOCK_REALTIME, &spec);
	newTime  = spec.tv_sec;
	if((newTime-oldTime)>5)
	    return 0;

	//printf("\n%d %d %d %d %d Waiting for %d %d %d %d %d ",b1,b2,b3,b4,b5,a1,a2,a3,a4,a5);	
    }
    //printf("\nEnd wait Goal");
    printf("\nLast C1-5 value:%d %d %d %d %d ",c1,c2,c3,c4,c5);
    return 0;

}

int MoveRobot(int a1,int a2,int a3,int a4,int a5, int mode)
{


/*	int b1,b2,b3,b4,b5;
	b1=getNormalizedInput(PLAYBACK_BASE_POSITION);
	b2=getNormalizedInput(PLAYBACK_END_POSITION);
	b3=getNormalizedInput(PLAYBACK_PIVOT_POSITION);
	b4=getNormalizedInput(PLAYBACK_ANGLE_POSITION);
	b5=getNormalizedInput(PLAYBACK_ROT_POSITION);
	a1=a1-b1; // subtract out the playback position 
	a2=a2-b2;
	a3=a3-b3;
	a4=a4-b4;
	a5=a5-b5;
	printf("\nPlayback position %d %d %d %d %d",b1,b2,b3,b4,b5);
*/
	//printf("\nStart wait Goal");
	mapped[COMMAND_REG]=CMD_MOVEEN | CmdVal;
	mapped[BASE_POSITION]=a1;
	mapped[END_POSITON]=a2;
	mapped[PIVOT_POSITON]=a3;
	mapped[ANGLE_POSITON]=a4;
	mapped[ROT_POSITON]=a5;
	mapped[COMMAND_REG]=CMD_MOVEEN | CMD_MOVEGO | CmdVal;
	mapped[COMMAND_REG]=CmdVal;
	if(mode==BLOCKING_MOVE)
		WaitMoveGoal(a1,a2,a3,a4,a5,DEFAULT_MOVE_TIMEOUT);
	return 0;
}

int SetParam(char *a1,float fa2,int a3,int a4,int a5)
{
	int i,BDH,BDL,Axis;
	int EndAxisLoc,AngleCorrectionFactor;
	float EndBoundryScale=DiffCorrectionFactor/16;//0.0666;
	int a2=(int)fa2;
	int fxa2=(a2<<8)+(fa2-a2)*256;
	
	//printf("%s %s %d %d \n",Params[i],a1,a2,i);

	for(i=0;i<MAX_PARAMS;i++)
	{
		//printf("%s %s %d %d \n",Params[i],a1,a2,i);
		if(strcmp(a1,Params[i])==0)
		{
				switch(i)
				{
						case 0:
						//printf("Set Speed\n");
							//set Max Speed
							maxSpeed=a2 & 0b00000000000011111111111111111111;
							mapped[ACCELERATION_MAXSPEED]=maxSpeed + (coupledAcceleration << 20);
							return 0;
						break;
						case 1:
							//set Acceleration
							coupledAcceleration=a2 & 0b111111;
							mapped[ACCELERATION_MAXSPEED]=maxSpeed + (coupledAcceleration << 20);
							return 0;
						break;
						case 2:
							mapped[FORCE_BIAS_BASE]=a2;
							return 0;
						case 3:
							mapped[FORCE_BIAS_END]=a2;
							return 0;
						break;
						case 4:
							mapped[FORCE_BIAS_PIVOT]=a2;
							return 0;
						break;
						case 5:
							mapped[FORCE_BIAS_ANGLE]=a2;
							return 0;
						break;
						case 6:
							mapped[FORCE_BIAS_ROT]=a2;
							return 0;
						break;
						case 7:
							mapped[FRICTION_BASE]=fxa2;
							return 0;
						break;
						case 8:
							mapped[FRICTION_END]=fxa2;
							return 0;
						break;
						case 9:
							mapped[FRICTION_PIVOT]=fxa2;
							return 0;
						break;
						case 10:
							mapped[FRICTION_ANGLE]=fxa2;
							return 0;
						break;
						case 11:
							mapped[FRICTION_ROT]=fxa2;
							return 0;
						break;
						case 12:
							BaseBoundryHigh=a2;
							BDH=BaseBoundryHigh;
							BDL=BaseBoundryLow;
							Axis=BOUNDRY_BASE;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 13:
							BaseBoundryLow=a2;
							BDH=BaseBoundryHigh;
							BDL=BaseBoundryLow;
							Axis=BOUNDRY_BASE;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 14:
							EndBoundryHigh=a2;
							BDH=EndBoundryHigh;
							BDL=EndBoundryLow;
							Axis=BOUNDRY_END;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 15:
							EndBoundryLow=a2;
							BDH=EndBoundryHigh;
							BDL=EndBoundryLow;
							Axis=BOUNDRY_END;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 16:
							PivotBoundryHigh=a2;
							BDH=PivotBoundryHigh;
							BDL=PivotBoundryLow;
							Axis=BOUNDRY_PIVOT;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 17:
							PivotBoundryLow=a2;
							BDH=PivotBoundryHigh;
							BDL=PivotBoundryLow;
							Axis=BOUNDRY_PIVOT;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 18:
							AngleBoundryHigh=a2;
							EndAxisLoc=GetAxisCurrent(1);
							AngleCorrectionFactor=EndAxisLoc*EndBoundryScale;
							//AngleBoundryHigh+AngleCorrectionFactor;
							//AngleBoundryLow+AngleCorrectionFactor;
							BDH=AngleBoundryHigh+AngleCorrectionFactor;
							BDL=AngleBoundryLow+AngleCorrectionFactor;
							Axis=BOUNDRY_ANGLE;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 19:
							AngleBoundryLow=a2;
							EndAxisLoc=GetAxisCurrent(1);
							AngleCorrectionFactor=EndAxisLoc*EndBoundryScale;
							//AngleBoundryHigh+AngleCorrectionFactor;
							//AngleBoundryLow+AngleCorrectionFactor;
							BDH=AngleBoundryHigh+AngleCorrectionFactor;
							BDL=AngleBoundryLow+AngleCorrectionFactor;
							Axis=BOUNDRY_ANGLE;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 20:
							RotateBoundryHigh=a2;
							BDH=RotateBoundryHigh;
							BDL=RotateBoundryLow;
							Axis=BOUNDRY_ROT;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 21:
							RotateBoundryLow=a2;
							BDH=RotateBoundryHigh;
							BDL=RotateBoundryLow;
							Axis=BOUNDRY_ROT;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 22:
							SetGripperMotor(a2);
							printf("Gripper Motor Set\n");
							return 0;
						break;
						case 23:
							SetGripperRoll(a2);
							printf("Gripper Roll Set\n");
							return 0;
						break;
						default:
						break;
				}
		
		}
	}
	return 0;

}
int MoveRobotRelative(int a1,int a2,int a3,int a4,int a5, int mode)
{
	int b1,b2,b3,b4,b5;
	b1=getNormalizedInput(BASE_POSITION_AT);
	b2=getNormalizedInput(END_POSITION_AT);
	b3=getNormalizedInput(PIVOT_POSITION_AT);
	b4=getNormalizedInput(ANGLE_POSITION_AT);
	b5=getNormalizedInput(ROT_POSITION_AT);
	
	//printf("\nRelative move %d %d %d %d %d",b1,b2,b3,b4,b5);
	return MoveRobot(a1+b1,a2+b2,a3+b3,a4+b4,a5+b5,mode);
}


int ReadDMA(int p1,int p2,char *p3)
{
	FILE *fp;
	int i,j,blocks,writeSize;
	int dataarray[256];
	fp=fopen(p3, "wb");
	if(fp!=0)
	{
		mapped[DMA_CONTROL]=DMA_RESET_ALL;
		mapped[DMA_CONTROL]=0;
		blocks=p2/256;
		for(j=0;j<blocks;j++) // only do full blocks inside loop
		{	
			mapped[DMA_READ_ADDRESS]=p1+(j*1024);
			mapped[DMA_READ_PARAMS]=(2<<8) | 127;
			mapped[DMA_CONTROL]=DMA_READ_BLOCK;
			mapped[DMA_CONTROL]=0;
			for(i=0;i<256;i++)
			{
				mapped[DMA_CONTROL]=DMA_READ_DEQUEUE;
				dataarray[i]=mapped[DMA_READ_DATA];
				//printf("\n %d",dataarray[i]);
				mapped[DMA_CONTROL]=0;
			}
			writeSize=fwrite((const void *)dataarray,sizeof(int),256,fp);
			printf("\n write %d iteration %d",writeSize,j);
			mapped[DMA_CONTROL]=0;
		}
		fclose(fp);
	}
	else
	{
		printf("\n %s",p3);
	}
	return 0;
}
int WriteDMA(int Address,char *FileName)
{
	FILE *fp;
	int i,j,blocks,readSize,Length;
	int dataarray[256];
	fp=fopen(FileName, "rb");
	if(fp!=0)
	{

		fseek(fp, 0, SEEK_END);    /* file pointer at the end of file */
		Length = ftell(fp);   /* take a position of file pointer size variable */
		fseek(fp, 0, SEEK_SET);    /* file pointer at the beginning of file */
		mapped[DMA_CONTROL]=DMA_RESET_ALL;
		mapped[DMA_CONTROL]=0;
		blocks=Length/256;
		for(j=0;j<blocks;j++) // only do full blocks inside loop
		{
			if((readSize=fread((void *)dataarray,sizeof(int),256,fp))==256)
			{
				mapped[DMA_WRITE_ADDRESS]=Address+(j*1024);
				mapped[DMA_WRITE_PARAMS]=(2<<8) | 127;
				mapped[DMA_CONTROL]=0;
				for(i=0;i<256;i++)
				{
					mapped[DMA_WRITE_DATA]=dataarray[i];
					mapped[DMA_CONTROL]=DMA_WRITE_ENQUEUE;
					mapped[DMA_CONTROL]=0;
				}
				mapped[DMA_CONTROL]=DMA_WRITE_INITIATE;
				mapped[DMA_CONTROL]=0;
				printf("\n write %d iteration %d",readSize,j);
			}
		}
		fclose(fp);
		printf("\n Table Loaded: Length %d",Length);
	}
	else
	{
	    printf("errno = %d.\n", errno);
		//perror(errno);
		printf("\n failed to open %s",FileName);
	}
	return Length;
}

int CaptureADtoFile(int Axis,int Start,int Length,int Delay,char *FileName)
{
	FILE *fp;
	int i,j,k,blocks,writeSize,AvgSIN,AvgCOS,ADVal;
	int dataarray[512];
	switch(Axis)
	{
	    case 0  :
		MoveRobot(Start,0,0,0,0,BLOCKING_MOVE);
		break; 
	    case 1  :
		MoveRobot(0,Start,0,0,0,BLOCKING_MOVE);
		break; 
	    case 2  :
		MoveRobot(0,0,Start,0,0,BLOCKING_MOVE);
		break; 
	    case 3  :
		MoveRobot(0,0,0,Start,0,BLOCKING_MOVE);
		break; 
	    case 4  :
		MoveRobot(0,0,0,0,Start,BLOCKING_MOVE);
		break; 
	}
	fp=fopen(FileName, "w");
	if(fp!=0)
	{
		blocks=Length/256;
		for(k=0;k<blocks;k++) // only do full blocks inside loop
		{	
			for(i=0;i<256;i++)
			{
				mapped[Axis]=Start+i+(k*256);
				AvgSIN=0;
				AvgCOS=0;
				for(j=0;j<Delay;j++)
				{	
					ADVal=mapped[ADLookUp[Axis]];
					AvgSIN=AvgSIN+ADVal;
					AvgCOS=AvgCOS+mapped[ADLookUp[Axis]+1];
				}			
				dataarray[i*2]=AvgSIN/Delay;
				dataarray[(i*2)+1]=AvgCOS/Delay;
//				printf("\n SIN %d COS %d",dataarray[i*2],dataarray[(i*2)+1]);
			}
			writeSize=fwrite((const void *)dataarray,sizeof(int),512,fp);
			printf("\n write %d iteration %d",writeSize,k);
		}
		fclose(fp);
	}
	else
	{
		printf("\n %s",FileName);
	}
	return 0;
}
int FindHome(int Axis,int Start,int Length,int Delay,char *FileName)
{
	FILE *fp;
	int i,j,k,blocks,AvgSIN,AvgCOS,ADVal,fileLen,MinSIN,MinSINIdx=0;
	//int readSize;
	//int MinCOS,MinCOSIdx=0;
	int RefrenceArray[1000000];
	int CaptureArray[Length*2];
	switch(Axis)
	{
	    case 0  :
		MoveRobot(Start,0,0,0,0,BLOCKING_MOVE);
		break; 
	    case 1  :
		MoveRobot(0,Start,0,0,0,BLOCKING_MOVE);
		break; 
	    case 2  :
		MoveRobot(0,0,Start,0,0,BLOCKING_MOVE);
		break; 
	    case 3  :
		MoveRobot(0,0,0,Start,0,BLOCKING_MOVE);
		break; 
	    case 4  :
		MoveRobot(0,0,0,0,Start,BLOCKING_MOVE);
		break; 
	}
	fp=fopen(FileName, "rb");
	if(fp!=0)
	{
		fseek(fp, 0, SEEK_END);
		fileLen=ftell(fp);
		fseek(fp, 0, SEEK_SET);
		//ReadSize=fread((char *)RefrenceArray, fileLen, 1, fp);
		fread((char *)RefrenceArray, fileLen, 1, fp);
		fclose(fp);
	}
	else
	{
		printf("\n %s",FileName);
		return 1;
	}
	for(k=0;k<Length;k++) 
	{	
		mapped[Axis]=Start+k;
		AvgSIN=0;
		AvgCOS=0;
		for(j=0;j<Delay;j++)
		{	
			ADVal=mapped[ADLookUp[Axis]];
			AvgSIN=AvgSIN+ADVal;
			AvgCOS=AvgCOS+mapped[ADLookUp[Axis]+1];
		}			
		CaptureArray[k*2]=AvgSIN/Delay;
		CaptureArray[(k*2)+1]=AvgCOS/Delay;
//		printf("\n Query %d Refrence %d",CaptureArray[k*2],RefrenceArray[k*2]);
	}
	MoveRobot(0,0,0,0,0,BLOCKING_MOVE);
	blocks=fileLen/8;
	MinSIN=0x7fffffff;
	//MinCOS=0x7fffffff;
	for(i=0;i<blocks;i++)
	{
		AvgSIN=0;
		AvgCOS=0;
		for(k=0;k<Length;k++)
		{
			AvgSIN=AvgSIN+abs(CaptureArray[k*2]-RefrenceArray[(k+i)*2]);
			AvgSIN=AvgSIN+abs(CaptureArray[(k*2)+1]-RefrenceArray[(((k+i)*2)+1)]);
//			AvgCOS=AvgCOS+abs(CaptureArray[(k*2)+1]-RefrenceArray[(((k+i)*2)+1)]);
		}
		if(MinSIN>AvgSIN)
		{
			MinSIN=AvgSIN;
			MinSINIdx=i;
		}
/*		if(MinCOS>AvgCOS)
		{
			MinCOS=AvgCOS;
			MinCOSIdx=i;
		}*/
	}
	printf("\n SAD %d SINidx %d\n",MinSIN,MinSINIdx);
	switch(Axis)
	{
	    case 0  :
		MoveRobot(30000-MinSINIdx,0,0,0,0,BLOCKING_MOVE);
		break; 
	    case 1  :
		MoveRobot(0,30000-MinSINIdx,0,0,0,BLOCKING_MOVE);
		break; 
	    case 2  :
		MoveRobot(0,0,30000-MinSINIdx,0,0,BLOCKING_MOVE);
		break; 
	    case 3  :
		MoveRobot(0,0,0,30000-MinSINIdx,0,BLOCKING_MOVE);
		break; 
	    case 4  :
		MoveRobot(0,0,0,0,30000-MinSINIdx,BLOCKING_MOVE);
		break; 
	}
	mapped[COMMAND_REG]=256;  //reset home
	mapped[COMMAND_REG]=0;
	return 30000-MinSINIdx;
}

void writeBuffer(char *name, int size, int offset)
{
	FILE *fp;
	int readSize=0;
	char * bufferAddr = memBaseAddr + offset;
	//char * bufferAddr = testAddr + offset;

	if (name == NULL)
	{
		printf("file name is null \n");
		return;
	}
	if (size == 0)
	{
		printf("size is 0 \n");
		return;
	}
	if ((size + offset) > WRITE_SIZE)
	{
		printf("size is too large \n");
		return;
	}
	printf("write file %s into 0x%p size %d \n",name, bufferAddr, size);

	fp=fopen(name, "r");

	if(fp!=0)
	{
		fseek(fp, 0, SEEK_SET);    /* file pointer at the beginning of file */
		readSize=fread((void *)bufferAddr,1,size,fp);
		if (readSize != size)
		{
			printf("read file %s size %d fail \n",name, size);
		}
		fclose(fp);
	}
	else
	{
		printf("open file %s fail \n",name);
	}
}

void config_wr_over_regs(struct bmp_data *data)
{
    mapped[REG_INDEX0]=184;
    //mapped[REG_INDEX1]=data->data_len;
    mapped[REG_INDEX1]=2452;
    mapped[REG_INDEX2]=data->data_width;
    mapped[REG_INDEX3]=data->data_height;
    mapped[REG_INDEX6]=data->data_width*data->data_height;
    mapped[REG_INDEX7]=1;
    //uio_print("Block %d writing over,width=%d.height=%d\n",\
							wr_block_index,data->data_width,data->data_height);	


    if( wr_block_index == BLOCK_INDEX2 )
	mapped[REG_INDEX4]=IOS_PS_BLOCK2_WRITING_OVER;
    else{
	/*make sure that BLOCK_INDEX0 change to BLOCK_INDEX1*/
	wr_block_index = BLOCK_INDEX1;
	mapped[REG_INDEX4]=IOS_PS_BLOCK1_WRITING_OVER;
    }
    uio_print("Block %d write over\n",wr_block_index);
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
    uio_print("Block %d read over\n",rd_block_index);

    /*change the rd_block_index*/
    rd_block_index = (rd_block_index==BLOCK_INDEX3?BLOCK_INDEX4:BLOCK_INDEX3);
}



static int check_wr_over_regs(struct bmp_data *data)
{
    if( mapped[REG_INDEX1] != data->data_len ||
        mapped[REG_INDEX2] != data->data_width  ||
	mapped[REG_INDEX3] != data->data_height    ||
	mapped[REG_INDEX6] != data->data_width*data->data_height )
	//return 0;
    return 1;
}

static struct bmp_data *get_first_node(struct list_head *head)
{
    if( (head->next != NULL) && (!list_empty(&list_h)) ){
	return list_entry(head->next,struct bmp_data,list);
    }
    return NULL;
}

static void rm_first_node(struct list_head *head)
{
    if( head->next != NULL ){
	struct bmp_data *node = list_entry(head->next,struct bmp_data,list);
	free(node->data_buffer);
	free(node);
	list_del_init(head->next);
    }
}

void writeData(int offset)
{
	char * bufferAddr = memBaseAddr + offset;
	struct bmp_data *node = get_first_node(&list_h);
	if(node == NULL ){
	    uio_print("NO pic buffer\n");
	    return;
	}
	//uio_print("len=%d,width=%d,height=%d\n",node->data_len,node->data_width,node->data_height);

	char *buffer = node->data_buffer;
	unsigned int size = node->data_len;

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

	rm_first_node(&list_h);
#if 0
	/*Check the primary regs,if result is 0, we can't del the first node*/
	if (check_wr_over_regs(node)){
	    rm_first_node(&list_h);
	}else
	    uio_print("Can't rm the list node\n");
#endif
}

void readData(int offset,int size)
{
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
	uio_print("read data from 0x%p size %d \n",bufferAddr, size);

#if 1
	char *name="outPic";
	FILE *fp=fopen(name, "wa+");

	if(fp!=0)
	{
		fseek(fp, 0, SEEK_SET);    /* file pointer at the beginning of file */
		size_t writeSize=fwrite((const void *)bufferAddr,1,size,fp);
		if (writeSize != size)
		{
			printf("write file %s size %d fail \n",name, size);
		}
		fclose(fp);
	}
	else
	{
		printf("open file %s fail \n",name);
	}
#else
	char readbuffer[size*2];
	memcpy(readbuffer,bufferAddr,size);
	uio_print("Data:%s\n",readbuffer);
#endif
	config_rd_over_regs();
}



void readBuffer(char *name, int size, int offset)
{
	FILE *fp;
	int writeSize=0;
	char * bufferAddr = image_ReadAddr + offset;
	//char * bufferAddr = testAddr + offset;

	if (name == NULL)
	{
		printf("file name is null \n");
		return;
	}
	if (size == 0)
	{
		printf("size is 0 \n");
		return;
	}
	if ((size + offset) > (BUFFER_SIZE - WRITE_SIZE))
	{
		printf("size is too large \n");
		return;
	}
	printf("read file %s from 0x%p size %d \n",name, bufferAddr, size);

	fp=fopen(name, "wa");

	if(fp!=0)
	{
		fseek(fp, 0, SEEK_SET);    /* file pointer at the beginning of file */
		writeSize=fwrite((const void *)bufferAddr,1,size,fp);
		if (writeSize != size)
		{
			printf("write file %s size %d fail \n",name, size);
		}
		fclose(fp);
	}
	else
	{
		printf("open file %s fail \n",name);
	}
}

void showPosAt(void)
{
		int b1,b2,b3,b4,b5;
		b1=getNormalizedInput(BASE_POSITION_AT);
		b2=getNormalizedInput(END_POSITION_AT);
		b3=getNormalizedInput(PIVOT_POSITION_AT);
		b4=getNormalizedInput(ANGLE_POSITION_AT);
		b5=getNormalizedInput(ROT_POSITION_AT);
		printf("\nPos %d %d %d %d %d  ",b1,b2,b3,b4,b5);	
}
void ReplayMovement(char *FileName)
{
	int Length,rbc;
	showPosAt();
	Length=WriteDMA(0x3f000000,FileName);
	mapped[RECORD_LENGTH]=Length/4;
	mapped[REC_PLAY_TIMEBASE]=1;
	mapped[REC_PLAY_CMD]=CMD_RESET_RECORD;
	mapped[REC_PLAY_CMD]=CMD_RESET_PLAY;
	mapped[REC_PLAY_CMD]=0;
	mapped[REC_PLAY_CMD]=CMD_PLAYBACK;
	rbc=mapped[READ_BLOCK_COUNT];
	showPosAt();
	
	//sleep(1);
	while((mapped[READ_BLOCK_COUNT] & 0x003fffff) != 0 )
	{
		if(rbc != mapped[READ_BLOCK_COUNT])
		{
				
			printf("%d \n",mapped[READ_BLOCK_COUNT] & 0x003fffff);
			rbc=mapped[READ_BLOCK_COUNT];
			showPosAt();
		}
		//printf("%d \n",mapped[READ_BLOCK_COUNT]);
	}
	while((mapped[READ_BLOCK_COUNT] & 0x00400000) != 0 )
		printf("%d \n",mapped[READ_BLOCK_COUNT]);
	showPosAt();
	mapped[REC_PLAY_CMD]=CMD_RESET_RECORD;
	mapped[REC_PLAY_CMD]=CMD_RESET_PLAY;
	mapped[REC_PLAY_CMD]=0;

	/*fa0=fa0+mapped[PLAYBACK_BASE_POSITION];
	fa1=fa1+mapped[PLAYBACK_END_POSITION];
	fa2=fa2+mapped[PLAYBACK_PIVOT_POSITION];
	fa3=fa3+mapped[PLAYBACK_ANGLE_POSITION];
	fa4=fa4+mapped[PLAYBACK_ROT_POSITION];
	mapped[FINE_ADJUST_BASE]=fa0;
	mapped[FINE_ADJUST_END]=fa1;
	mapped[FINE_ADJUST_PIVOT]=fa2;
	mapped[FINE_ADJUST_ANGLE]=fa3;
	mapped[FINE_ADJUST_ROT]=fa4;*/
	mapped[REC_PLAY_CMD]=CMD_RESET_RECORD;
}
int getInput(void)
{
	char iString[255];
	if(fgets(iString,200,stdin)!=NULL)
	{
		return ParseInput(iString);
	}
	return 0;

}
int ParseInput(char *iString)
{
	//char iString[255];
	const char delimiters[] = " ,";
	char *token,*p1,*p2,*p3,*p4,*p5;
	int i,j,Add,Start,Length,Delay,Axis,tokenVal;
	//printf("\nStart wait Goal");
	if(iString !=NULL)
	{
		token = strtok (iString, delimiters);
		if (token !=NULL)
		{
			tokenVal=HashInputCMD(token);
			//printf("Token %s TokenVal %i",token,tokenVal);
		}
		else
			return 1;
		if (tokenVal != 0 ){
			switch(tokenVal)
			{
				case HEART_BEAT :/*{{{*/
				break; 

				case SET_FORCE_MOVE_POINT :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					p5=strtok (NULL, delimiters);
					ForceDestination[0]=atoi(p1);
					ForceDestination[1]=atoi(p2);
					ForceDestination[2]=atoi(p3);
					ForceDestination[3]=atoi(p4);
					ForceDestination[4]=atoi(p5);
				break; 

				case SEND_HEARTBEAT :
					printf("heartbeat dispatched\n");
				break; 
				case SET_PARAM :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
	//				p3=strtok (NULL, delimiters);
	//				p4=strtok (NULL, delimiters);
	//				p5=strtok (NULL, delimiters);
					SetParam(p1,atof(p2),0,0,0);
				break; 
				case MOVEALL_RELATIVE :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					p5=strtok (NULL, delimiters);
					MoveRobotRelative(atoi(p1),atoi(p2),atoi(p3),atoi(p4),atoi(p5),BLOCKING_MOVE);
				break; 
				case REPLAY_MOVEMENT  :
					p1=strtok (NULL, delimiters);
					ReplayMovement(p1);
				break; 
				case SLEEP_CMD  :
					p1=strtok (NULL, delimiters);
					usleep(atoi(p1));
				break; 
				case MOVE_CMD  :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
				break; 
				case LOAD_TABLES  :
					WriteDMA(0x3f800000,"/srv/samba/share/a0tbl.dta");
					WriteDMA(0x3f900000,"/srv/samba/share/a1tbl.dta");
					WriteDMA(0x3fa00000,"/srv/samba/share/a2tbl.dta");
				break; 
				case CAPTURE_POINTS_CMD :
					p1=strtok (NULL, delimiters);
					InitCapturePoints(p1);
				break;
				case RECORD_MOVEMENT :
					p1=strtok (NULL, delimiters);
					InitCaptureMovement(p1);
				break;
				case DMAREAD_CMD :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					Add=(int)strtol(p1,0,16);
					Length=(int)strtol(p2,0,16);
					ReadDMA(Add,Length,p3);
				break;
				case DMAWRITE_CMD :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					Add=(int)strtol(p1,0,16);
					WriteDMA(Add,p2);
				break;
				case CAPTURE_AD_CMD :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					p5=strtok (NULL, delimiters);
					Axis=atoi(p1);
					Start=atoi(p2);
					Length=atoi(p3);
					Delay=atoi(p4);
					CaptureADtoFile(Axis,Start,Length,Delay,p5);
				break;
				case FIND_HOME_REP_CMD :
					p1=strtok (NULL, delimiters);
					Axis=atoi(p1);
					Start=0;
					Length=5000;
					Delay=10000;
					switch(Axis)
					{
						case 0  :
							while(abs(FindHome(Axis,Start,Length,Delay,"/srv/samba/share/d0.dta"))>300);
						break; 
						case 1  :
							while(abs(FindHome(Axis,Start,Length,Delay,"/srv/samba/share/d1.dta"))>300);
						break; 
						case 2  :
							while(abs(FindHome(Axis,Start,Length,Delay,"/srv/samba/share/d2.dta"))>300);
						break; 
						case 3  :
							while(abs(FindHome(Axis,Start,Length,Delay,"/srv/samba/share/d3.dta"))>300);
						break; 
						case 4  :
							while(abs(FindHome(Axis,Start,Length,Delay,"/srv/samba/share/d4.dta"))>300);
						break; 
					}
				break; 
					case FIND_HOME_CMD :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					p5=strtok (NULL, delimiters);
					Axis=atoi(p1);
					Start=atoi(p2);
					Length=atoi(p3);
					Delay=atoi(p4);
					FindHome(Axis,Start,Length,Delay,p5);
				break; 
				case FIND_INDEX_CMD :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					Axis=atoi(p1);
					Start=atoi(p2);
					Length=atoi(p3);
					Delay=atoi(p4);
					if(FindIndex(Axis,Start,Length,Delay)==0)
					{
						printf("/nIndex reached");
						break;
					}
					else
					{
						printf("/nIndex not found");
					}
				break; 
				case MOVEALL_CMD :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					p5=strtok (NULL, delimiters);
					MoveRobot(atoi(p1),atoi(p2),atoi(p3),atoi(p4),atoi(p5),BLOCKING_MOVE);
				break; 
				case SLOWMOVE_CMD  :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					p5=strtok (NULL, delimiters);
					Add=atoi(p1);
					Length=atoi(p3);
					Start=atoi(p2);
					Delay=atoi(p4);
					mapped[Add]=Start;
					printf("\n %d %d \n",Add,Length);
					for(i=0;i<Length;i++)
					{
						for(j=0;j<Delay;j++)			
							mapped[Add]=Start+i;
					}
				break;
				case READ_CMD  :/*}}}*/
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					if((p1==NULL) | (p2==NULL))
					{
						printf("\n %d %d need addres and length",atoi(p1),atoi(p2));
						break;
					}
					Add=atoi(p1);
					Length=atoi(p2);
					printf("\n %d %d \n",Add,Length);
					for(i=0;i<Length;i++)
					{
						printf("\n %x ",mapped[Add+i]);
					}
				break; 
				case WRITE_CMD  :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					if((p1==NULL) | (p2==NULL))
					{
						printf("\n %d %d need addres and data",atoi(p1),atoi(p2));
					}
					printf("\n %d %d \n",atoi(p1),atoi(p2));
					i=atoi(p1);
					j=atoi(p2);
					if(i==COMMAND_REG)
						CmdVal=j;
					mapped[i]=j;
					if(i==ACCELERATION_MAXSPEED)
					{
						maxSpeed=j & 0b00000000000011111111111111111111;
						coupledAcceleration=(j & 0b00000011111100000000000000000000) >> 20;
						//startSpeed=0,
					}
				break;
				case WRITE_BUFFER_CMD  :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					writeBuffer(p1, atoi(p2), atoi(p3));
				break;
				case READ_BUFFER_CMD  :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					readBuffer(p1, atoi(p2), atoi(p3));
				break;
				case EXIT_CMD  :
					mapped[FINE_ADJUST_BASE]=0;
					mapped[FINE_ADJUST_END]=0;
					mapped[FINE_ADJUST_PIVOT]=0;
					mapped[FINE_ADJUST_ANGLE]=0;
					mapped[FINE_ADJUST_ROT]=0;
					return 1;
				break;

				default :
				printf("\nunrecognized command");
			}
			return 0;
		}
		else
			return 1;
	}
	return 1;
}

void *Ps_send_handle (void *arg)
{
    char *rd_p;
    int i=0;
    int index_num=0;
    int index_bytes=0;
    char pl_rd_over;

    uio_print("init \n");
    parse_each_file();
    while(1)
    {
	sleep(1);
	rd_p=(char *)mapped;
	if(rd_p[0] != 0x08 ){
	    uio_print("Fpga Data is error\n");
	    return;
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
	sleep(2);
	uio_print("index0=0x%x.index1=0x%x.index2=0x%x.index3=0x%x.index4=0x%x\n",\
			    					mapped[REG_INDEX0],\
								mapped[REG_INDEX1],mapped[REG_INDEX2],\
								mapped[REG_INDEX3],mapped[REG_INDEX4]);
	rd_p=(char *)mapped;
	if(rd_p[0] != 0x08 ){
		uio_print("Index Data is error\n");
		return;
	}
	index_num=rd_p[1];
	index_bytes=rd_p[2];
	uio_print("index_num=%d,index_bytes=%d\n",index_num,index_bytes);
	for(i=0;i<index_num;i++){
		//uio_print("rd_p[4+%d]=0x%x\n",i,rd_p[4+i]);
		if( rd_p[4+i]==REG_INDEX1 ){
			/*high 8 bit + low 16 bits data_len*/
			pl_wr_over = rd_p[4+4*index_bytes+i*4+2];
			pl_wr_lens = (unsigned short )mapped[1+index_bytes+i];
		        uio_print("pl_wr_over=0x%x\n",pl_wr_over);
		        uio_print("pl_wr_lens=0x%x\n",pl_wr_lens);
			break;
		}
	}

	if( (rd_block_index== BLOCK_INDEX3) && ((pl_wr_over&0xff) == IOS_PL_BLOCK3_WRITING_OVER) ){
	    readData(MEM_BLOCK_3_OFFSET,pl_wr_lens);

	}else if((rd_block_index == BLOCK_INDEX4) && ((pl_wr_over&0xff) == IOS_PL_BLOCK4_WRITING_OVER) ){
	    readData(MEM_BLOCK_4_OFFSET,pl_wr_lens);
	}
    }
}


static void parse_each_file()
{
    //char outfile_name[30];
    char cur_dir[200];
    DIR *dir;
    struct dirent *ptr;
    int index=0;

    memset(cur_dir,'\0',sizeof(cur_dir));
    getcwd(cur_dir,999);
    printf("the current dir is : %s\n",cur_dir);
    if ((dir=opendir(cur_dir)) == NULL)
    {
	perror("Open dir error...");
	exit(1);
    }
    while( (ptr=readdir(dir)) != NULL ){
	if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
	    continue;
	else if(ptr->d_type == 8)    ///file
	{
	    //strcpy(img_path[img_num],cur_dir);
	    //strcat(img_path[img_num++],ptr->d_name);
	    if(strstr(ptr->d_name,".jpg")!=NULL || (strstr(ptr->d_name,".jpeg"))!=NULL ){
		printf("file[%d] is : %s\n",index++,ptr->d_name);
		//strncpy(outfile_name,ptr->d_name,strlen(ptr->d_name)-strlen(".jpg"));
		//strcat(outfile_name,".bmp");
		struct bmp_data *data = malloc(sizeof(struct bmp_data));
		/*load pic attrs*/
		char *buff = loadJpg((char *)ptr->d_name,data);
		data->data_buffer=buff;
		list_add(&data->list, &list_h);
	    }
	}
    }
}

/*
 *argv1:state
 *argv2:
 *argv3:runmode
 * */
int main(int argc, char *argv[]) {

  int fd,mfd,err;

  int size;
  int DefaultMode;
  int imageSize = BUFFER_SIZE;

  if (argc != 4) {
    fprintf(stderr, "Usage: %s Needs init mode, Master/Slave and RunMode\n", argv[0]);
    exit(1);
  }

  DefaultMode = atoi(argv[1]);
  size = 4095;
  RunMode=atoi(argv[3]);
  if (size <= 0) {
    fprintf(stderr, "Bad size: %d\n", size);
    exit(1);
  }

  fd = open("/dev/uio1", O_RDWR);
  if (fd < 0) {
    perror("Failed to open uio1 devfile");
    return 1;
  }

#if 0
  fd = open("uio_dev.txt", O_RDWR|O_CREAT,0644);
  {
    off_t offset;
    offset = 4095;
    //step2, create a hole file
    //off_t lseek(int fildes, off_t offset, int whence);
    if (lseek(fd, offset, SEEK_SET) == (off_t) - 1)
    {
	fprintf(stderr, "lseek() failed:%s\n", strerror(errno));
	//FIXME:unlink the file
	close(fd);
	exit(-3);
    }

    size_t n;

    //size_t write(int fd, const void *buf, size_t count);
    if ((n = write(fd, "", 1)) < 0)
    {
	fprintf(stderr, "write() failed:%s\n", strerror(errno));
	exit(-4);
    }
  }
#endif

  map_addr = mmap( NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  if (map_addr == MAP_FAILED) {
    perror("Failed to mmap");
    return 1;
  }

  mapped = map_addr;


  mfd = open("/dev/mem", O_RDWR);
  map_addrCt = mmap(NULL, imageSize, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, 0x3f000000);
#if 0
  mfd = open("uio_mem.txt", O_RDWR|O_CREAT,0644);

  {
    off_t offset;
    offset = imageSize;
    //step2, create a hole file
    //off_t lseek(int fildes, off_t offset, int whence);
    if (lseek(mfd, offset, SEEK_SET) == (off_t) - 1)
    {
	fprintf(stderr, "lseek() failed:%s\n", strerror(errno));
	//FIXME:unlink the file
	close(mfd);
	exit(-3);
    }

    size_t n;

    //size_t write(int fmd, const void *buf, size_t count);
    if ((n = write(mfd, "", 1)) < 0)
    {
	fprintf(stderr, "write() failed:%s\n", strerror(errno));
	exit(-4);
    }
  }

  map_addrCt = mmap(NULL, imageSize, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, 0);
#endif

  if (map_addrCt == MAP_FAILED) {
      //printf("\nCant Map Callibration Tables \n");
      perror("Failed to mmap High memory");
      return 1;
  }
  memBaseAddr = (char *)map_addrCt;
  image_ReadAddr = memBaseAddr + WRITE_SIZE;

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


  //  Addr= = atoi(argv[3]);
  //  Dta= = atoi(argv[4]);
  //  mapped[Addr] = Dta;
  setDefaults(DefaultMode);
  //	if(DefaultMode ==2 )
  if(atoi(argv[2])==1)
  {

      err = pthread_create(&(tid[0]), NULL, &StartServerSocket,  (void*)&ThreadsExit);
      if (err != 0)
      {
	  printf("\ncan't create thread :[%s]", strerror(err));
	  return 1;
      }
      else
      {
	  printf("\n Begin Socket Server Thread\n");
      }
  }
  else if(atoi(argv[2])==2)
  {

      err = pthread_create(&(tid[1]), NULL, &StartClientSocket, NULL);
      if (err != 0)
      {
	  printf("\ncan't create thread :[%s]", strerror(err));
	  return 1;
      }
      else
      {
	  printf("\n Begin Socket Client Thread\n");
      }
  }
  else if(atoi(argv[2])==3)
  {

      err = pthread_create(&(tid[0]), NULL, &StartServerSocketDDE,  (void*)&ThreadsExit);
      if (err != 0)
      {
	  printf("\ncan't create thread :[%s]", strerror(err));
	  return 1;
      }
      else
      {
	  printf("\n Begin Socket Server Thread For DDE\n");
      }
  }
  if(RunMode==1)
  {
      err = pthread_create(&(tid[2]), NULL, &RealtimeMonitor, (void*)&ThreadsExit );
      if (err != 0)
      {
	  printf("\ncan't create thread :[%s]", strerror(err));
	  return 1;
      }
      else
      {
	  printf("\n Begin Realtime Monitor Thread\n");
      }
  }


  while(getInput()==0);
  ThreadsExit=0;
  sleep(1);


  printf("\nExiting \n");

  munmap(map_addr, size);
  munmap(map_addrCt, imageSize);

  close(fd);
  close(mfd);
  return 0;
}
