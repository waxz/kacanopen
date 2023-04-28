/*
This file is part of CanFestival, a library implementing CanOpen Stack. 

Copyright (C): Edouard TISSERANT and Francis DUPIN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/*
	zlg_usbcan driver.
*/

#include <stdio.h>
#include <unistd.h>
#include <threads.h>
#include <stdio.h>
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define KRESET "\x1B[0m"
#define MLOGW(__format,...) {printf(KRED "%s:%i @%s:" __format " \n" KRESET, __FILE__, __LINE__, __FUNCTION__ , __VA_ARGS__);}
#define MLOGI(__format,...) {printf(KGRN "%s:%i @%s:" __format " \n" KRESET, __FILE__, __LINE__, __FUNCTION__ , __VA_ARGS__);}
#define MLOGD(__format,...) {printf(KYEL "%s:%i @%s:" __format " \n" KRESET, __FILE__, __LINE__, __FUNCTION__ , __VA_ARGS__);}


#include <math.h>
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))



#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>


typedef enum {
    STR2INT_SUCCESS,
    STR2INT_OVERFLOW,
    STR2INT_UNDERFLOW,
    STR2INT_INCONVERTIBLE
} str2int_errno;

/* Convert string s to int out.
 *
 * @param[out] out The converted int. Cannot be NULL.
 *
 * @param[in] s Input string to be converted.
 *
 *     The format is the same as strtol,
 *     except that the following are inconvertible:
 *
 *     - empty string
 *     - leading whitespace
 *     - any trailing characters that are not part of the number
 *
 *     Cannot be NULL.
 *
 * @param[in] base Base to interpret string in. Same range as strtol (2 to 36).
 *
 * @return Indicates if the operation succeeded, or why it failed.
 */
str2int_errno str2int(int *out, char *s, int base) {
    char *end;
    if (s[0] == '\0' || isspace(s[0]))
        return STR2INT_INCONVERTIBLE;
    errno = 0;
    long l = strtol(s, &end, base);
    /* Both checks are needed because INT_MAX == LONG_MAX is possible. */
    if (l > INT_MAX || (errno == ERANGE && l == LONG_MAX))
        return STR2INT_OVERFLOW;
    if (l < INT_MIN || (errno == ERANGE && l == LONG_MIN))
        return STR2INT_UNDERFLOW;
    if (*end != '\0')
        return STR2INT_INCONVERTIBLE;
    *out = l;
    return STR2INT_SUCCESS;
}





#include "controlcan.h"

#define NEED_PRINT_MESSAGE
#include "can_driver.h"
#include "def.h"

#define MAX_NB_CAN_PIPES 16

typedef struct {
  char used;
  int pipe[2];
} CANPipe;



static thread_local CANPipe canpipes[MAX_NB_CAN_PIPES] = {{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},};
#define MAX_CHANNEL_NUM 2
#define MAX_BUFFER_SIZE 100
typedef struct{
    unsigned int nDeviceType;
    int nDeviceInd;
    int nRunning;
    int nMaxChannel;
    int nMaxWaitMs;
    int nChannel; // active channel used in send and recv
    int nChannelStatus[MAX_CHANNEL_NUM]; //0 : init value, 1 : ok, -1 : error
} CAN_Dev;

/*********functions which permit to communicate with the board****************/

UNS8 canReceive_driver(CAN_HANDLE fd0, Message *m)
{
	if(read(((CANPipe*)fd0)->pipe[0], m, sizeof(Message)) != (ssize_t)sizeof(Message))
	{
		return 1;
	}
	return 0;
}

// use fd0 as channel index
UNS8 canReceiveBatch_driver(CAN_HANDLE fd0, Message *m, int* len )
{

    MLOGW("hello %ld", (long)fd0);

    *len=0;

    CAN_Dev * can_dev = fd0;
    long nCANInd = can_dev->nChannel;

    MLOGW("channel index [%ld]", nCANInd);


    if(nCANInd <0 || nCANInd >= MAX_CHANNEL_NUM){
        MLOGW("channel [%ld] is wrong channel index", nCANInd);
        return 1;
    }
    if(can_dev->nChannelStatus[nCANInd] == 0){
        MLOGW("channel [%ld] not init ok ", nCANInd);
        return -1;
    }
    MLOGW("channel [%ld] init ok ", nCANInd);

    DWORD size;

    static thread_local VCI_CAN_OBJ recv_vco[MAX_BUFFER_SIZE];

    if( (nCANInd ==0) || (nCANInd ==1)){
        size = VCI_GetReceiveNum(can_dev->nDeviceType, can_dev->nDeviceInd, nCANInd);
        MLOGW("channel [%ld] VCI_GetReceiveNum [%d]", nCANInd,size);

        if(!size) return 0;
        size = MIN(size,MAX_BUFFER_SIZE);


        VCI_Receive(can_dev->nDeviceType, can_dev->nDeviceInd, nCANInd, recv_vco, size, can_dev->nMaxWaitMs);
        MLOGW("channel [%ld] recv [%d] msg", nCANInd,size);
        *len = size;

        for(long i = 0 ; i < size; i++){

            if(recv_vco[i].ExternFlag){
                MLOGW("VCI_Receive ExternFlag %d", recv_vco[i].ExternFlag);
                return -2;
            }else{
                m[i].cob_id = recv_vco[i].ID;

            }

            m[i].len = recv_vco[i].DataLen;
            m[i].rtr = recv_vco[i].RemoteFlag;


            for(int j = 0 ; j < 8;j++){
                m[i].data[j] =recv_vco[i].Data[j];
            }
        }


    }else{

        return -1;

    }

    return 0;
}

/***************************************************************************/
UNS8 canSendBatch_driver(CAN_HANDLE fd0, Message *m, int len )
{

    CAN_Dev * can_dev = fd0;
    long nCANInd = can_dev->nChannel;

    if(can_dev->nChannelStatus[nCANInd] == 0){
        MLOGW("channel [%d] not init ok ", nCANInd);
        return -1;
    }

    static thread_local VCI_CAN_OBJ send_vco[MAX_BUFFER_SIZE];


    DWORD size;

    size = MIN(MAX_BUFFER_SIZE,len);

    for(long i = 0 ; i < size; i++){

        if(m[i].ext){
            MLOGW("VCI_Receive ExternFlag %d",m[i].ext);
            return -2;
        }else{

        }

        send_vco[i].ExternFlag = m[i].ext;

        send_vco[i].ID = m[i].cob_id;// 填写第一帧的ID
        send_vco[i].SendType = 0;// 正常发送
        send_vco[i].RemoteFlag = m[i].rtr;// 数据帧
        send_vco[i].DataLen = m[i].len;// 数据长度1个字节

        for(int j = 0 ; j < 8;j++){
            send_vco[i].Data[j] = m[i].data[j];
        }
    }
    DWORD dwRel = VCI_Transmit(can_dev->nDeviceType, can_dev->nDeviceInd, nCANInd, send_vco, size);


}

UNS8 canSend_driver(CAN_HANDLE fd0, Message const *m)
{
  int i;
  
  printf("%lx->[ ", (CANPipe*)fd0 - &canpipes[0]); 
  for(i=0; i < MAX_NB_CAN_PIPES; i++)
  {
  	if(canpipes[i].used && &canpipes[i] != (CANPipe*)fd0)
  	{
		printf("%x ",i);	
  	}
  }
  printf(" ]");	
  print_message(m);
  
  // Send to all readers, except myself
  for(i=0; i < MAX_NB_CAN_PIPES; i++)
  {
  	if(canpipes[i].used && &canpipes[i] != (CANPipe*)fd0)
  	{
		write(canpipes[i].pipe[1], m, sizeof(Message));
  	}
  }
  return 0;
}

/***************************************************************************/
int TranslateBaudRate(char* optarg){
	if(!strcmp( optarg, "1M")   || !strcmp( optarg, "1000")) return (int)1000;
	if(!strcmp( optarg, "500K") || !strcmp( optarg, "500")) return (int)500;
	if(!strcmp( optarg, "250K") || !strcmp( optarg, "250")) return (int)250;
	if(!strcmp( optarg, "125K") || !strcmp( optarg, "125")) return (int)125;
	if(!strcmp( optarg, "100K") || !strcmp( optarg, "100")) return (int)100;
	if(!strcmp( optarg, "50K")  || !strcmp( optarg, "50")) return (int)50;
	if(!strcmp( optarg, "20K")  || !strcmp( optarg, "20")) return (int)20;
	if(!strcmp( optarg, "10K")  || !strcmp( optarg, "10")) return (int)10;
	if(!strcmp( optarg, "5K")   || !strcmp( optarg, "5")) return (int)5;
	if(!strcmp( optarg, "none")) return 0;
    if(!strcmp( optarg, "0")) return 0;

	return 0x0000;
}
//*******ZLG

int TranslateBaudRateZLG(unsigned baud_rate, unsigned char* Timing0, unsigned char* Timing1)
{
    switch (baud_rate) {    //Kbps
        case 5:
            *Timing0 = 0xBFU; *Timing1 = 0xFFU; break;
        case 10:
            *Timing0 = 0x31U; *Timing1 = 0x1CU; break;
        case 20:
            *Timing0 = 0x18U; *Timing1 = 0x1CU; break;
        case 40:
            *Timing0 = 0x87U; *Timing1 = 0xFFU; break;
        case 50:
            *Timing0 = 0x09U; *Timing1 = 0x1CU; break;
        case 80:
            *Timing0 = 0x83U; *Timing1 = 0xFFU; break;
        case 100:
            *Timing0 = 0x04U; *Timing1 = 0x1CU; break;
        case 125:
            *Timing0 = 0x03U; *Timing1 = 0x1CU; break;
        case 200:
            *Timing0 = 0x81U; *Timing1 = 0xFAU; break;
        case 250:
            *Timing0 = 0x01U; *Timing1 = 0x1CU; break;
        case 400:
            *Timing0 = 0x80U; *Timing1 = 0xFA; break;
        case 500:
            *Timing0 = 0x00U; *Timing1 = 0x1CU; break;
        case 666:
            *Timing0 = 0x80U; *Timing1 = 0xB6U; break;
        case 800:
            *Timing0 = 0x00U; *Timing1 = 0x16U; break;
        case 1000:
            *Timing0 = 0x00U; *Timing1 = 0x14U; break;
        default:
            return -1;
    }

    return 0;
}

UNS8 canChangeBaudRate_driver( CAN_HANDLE fd0, char* baud)
{
    MLOGW("changing to baud rate %s, is not supported", baud);
//    printf("%lx-> changing to baud rate %s[%d]\n", (CANPipe*)fd0 - &canpipes[0],baud,TranslateBaudRate(baud));
    return 0;
}

#define new(TYPE) malloc(sizeof(*TYPE))

/***************************************************************************/
///
/// \param board , define busname ="<device_type>:<device_index>" , eg. "USBCAN-I:0"; baudrate = "<baudrate_on_channel_1>:<baudrate_on_channel_2>", eg "500:500", if fist channel is disabled, set it to 0, eg "0:500"
/// \return
CAN_HANDLE canOpen_driver(s_BOARD *board)
{



    CAN_Dev* can_dev = malloc(sizeof( CAN_Dev));

    MLOGI("can_dev address : %ld",can_dev );

    can_dev->nDeviceInd = 0;
    can_dev->nChannelStatus[0] = 0;
    can_dev->nChannelStatus[1] = 0;
    can_dev->nRunning = 0;
    can_dev->nMaxWaitMs = 1;
    can_dev->nMaxChannel = MAX_CHANNEL_NUM;
    int nReserved = 0;

    // split busname
    {
        char* st = board->busname ;
        char *ch;
        ch = strtok(st, ":");
        int count = 0;
        while (ch != NULL) {
            printf("get %s\n", ch);

            switch (count) {
                case 0:
                    printf("device %s\n", ch);
                    if(strcmp(board->busname, "USBCAN-I")  == 0  )       can_dev->nDeviceType = 3;
                    else if( strcmp(board->busname, "USBCAN-II")  == 0 ) can_dev->nDeviceType = 4;
                    else{
                        MLOGW("board->busname:[%s] is not supported", board->busname);
                        free(can_dev);
                        return (void*)0;
                    }
                    break;
                case 1:
                    printf("id %s\n", ch);
                    int dev_id;
                    if(str2int(&dev_id, ch, 10) == STR2INT_SUCCESS){
                        printf("dev_id %d\n", dev_id);
                        can_dev->nDeviceInd = dev_id;
                    }else{
                        MLOGW("board->nDeviceInd format error : [%s]", ch);
                        free(can_dev);
                        return (void*)0;
                    }

                    break;
                default:
                    break;
            }
            count ++;
            ch = strtok(NULL, ":");
        }

    }
    MLOGI("create board with busname:[%s], device_type:[%d], device_index:[%d]",board->busname,can_dev->nDeviceType,can_dev->nDeviceInd );


    int baud_rate_list[MAX_CHANNEL_NUM] ;

    int channel_num = 0;
    // split baud_rate
    {
        char* st = board->baudrate ;
        char *ch;
        ch = strtok(st, ":");
        int count = 0;
        while (ch != NULL) {
            printf("get baud_rate %s on channel %d\n", ch, count);
            if(count >= MAX_CHANNEL_NUM){
                MLOGW("define channel  more than MAX_CHANNEL_NUM = %d", MAX_CHANNEL_NUM);
                free(can_dev);
                return (void*)0;
            }
            baud_rate_list[count] =  TranslateBaudRate(ch);
            count ++;
            channel_num++;
            ch = strtok(NULL, ":");
        }

    }


    if(channel_num==0){

        MLOGW("device channel_num is [%d]", channel_num);
        free(can_dev);
        return (void*)0;
    }


    DWORD dwRel  = VCI_OpenDevice(can_dev->nDeviceType, can_dev->nDeviceInd, nReserved);
    if (dwRel != STATUS_OK)
    {
        MLOGW("device [%s] open failed, error: %d", board->busname,dwRel );
        free(can_dev);
        return (void*)0;
    }

    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xffffffff;
    config.Filter = 1;
    config.Mode = 0;


    for(int nCANInd = 0 ; nCANInd < MAX_CHANNEL_NUM; nCANInd++){
        int baud_rate = baud_rate_list[nCANInd];
        if(baud_rate == 0){
            MLOGW("device type [%s], index [%d],  channel[%d], set wrong baud_rate [%d], disable channel[%d]\n", board->busname,  can_dev->nDeviceInd, nCANInd, baud_rate,nCANInd );
            continue;
        }
        TranslateBaudRateZLG(baud_rate, &(config.Timing0), &(config.Timing1));

        dwRel = VCI_InitCAN(can_dev->nDeviceType, can_dev->nDeviceInd, nCANInd, &config);
        if (dwRel == STATUS_ERR)
        {
            VCI_CloseDevice(can_dev->nDeviceType, can_dev->nDeviceInd);
            free(can_dev);
            return (void*)0;
        }
        dwRel = VCI_StartCAN(can_dev->nDeviceType, can_dev->nDeviceInd, nCANInd);
        if (dwRel == STATUS_ERR)
        {
            VCI_CloseDevice(can_dev->nDeviceType, can_dev->nDeviceInd);
            free(can_dev);
            return (void*)0;
        }
        can_dev->nChannelStatus[nCANInd] = 1;

        dwRel = VCI_ClearBuffer(can_dev->nDeviceType, can_dev->nDeviceInd, nCANInd);

    }
    can_dev->nRunning = 1;

    return can_dev;
}
CAN_HANDLE canOpen_driver_pipe(s_BOARD *board)
{

  (void) board;
  int i;  
  for(i=0; i < MAX_NB_CAN_PIPES; i++)
  {
  	if(!canpipes[i].used)
	  	break;
  }

  /* Create the pipe.  */
  if (i==MAX_NB_CAN_PIPES || pipe(canpipes[i].pipe))
    {
      fprintf (stderr, "Open failed.\n");
      return (CAN_HANDLE)NULL;
    }

   canpipes[i].used = 1;
   return (CAN_HANDLE) &canpipes[i];
}

/***************************************************************************/
int canClose_driver_pipe(CAN_HANDLE fd0)
{
  close(((CANPipe*)fd0)->pipe[0]);
  close(((CANPipe*)fd0)->pipe[1]);
  ((CANPipe*)fd0)->used = 0;
  return 0;
}
int canClose_driver(CAN_HANDLE fd0)
{
    if(fd0 != 0){

        CAN_Dev* can_dev = fd0;

        if(can_dev->nRunning == 1){
            VCI_CloseDevice(can_dev->nDeviceType, can_dev->nDeviceInd);
        }
        can_dev->nRunning = 0;
        free(can_dev);
        MLOGI("close device, device_type:[%d], device_index:[%d]",can_dev->nDeviceType,can_dev->nDeviceInd );

    }else{

    }
    return 0;
}

