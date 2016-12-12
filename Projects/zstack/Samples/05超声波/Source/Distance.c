/******************************************************************************
  Filename:       Distance.c
  Revised:        $Date: 2012-03-07 01:04:58 -0800 (Wed, 07 Mar 2012) $
  Revision:       $Revision: 29656 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
******************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Generic"
  application every 5 seconds.  The application will also
  receives "Hello World" packets.

  The "Hello World" messages are sent/received as MSG type message.

  This applications doesn't have a profile, so it handles everything
  directly - itself.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include <string.h>
//#include "Common.h"
#include "DebugTrace.h"
#include "Distance.h"
#include "MT.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "mt_uart.h"
#include "HC_SR04.h"

#define SEND_DATA_EVENT 0x01
float fDistance;
float  distance;
float s;
int dis;
uint H;
uint H1;
uint L;
uint L1;
unsigned char theMessageData[10]="EndDevice";
const cId_t Distance_ClusterList[Distance_MAX_CLUSTERS] =
{
  Distance_CLUSTERID
};



const SimpleDescriptionFormat_t Distance_SimpleDesc =
{
  Distance_ENDPOINT,              //  int Endpoint;
  Distance_PROFID,                //  uint16 AppProfId[2];
  Distance_DEVICEID,              //  uint16 AppDeviceId[2];
  Distance_DEVICE_VERSION,        //  int   AppDevVer:4;
  Distance_FLAGS,                 //  int   AppFlags:4;
  
  
  0,          //  byte  AppNumInClusters;
  (cId_t *)NULL,  //  byte *pAppInClusterList;
  Distance_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)Distance_ClusterList   //  byte *pAppInClusterList;
};

unsigned char TempDATA;
endPointDesc_t Distance_epDesc;
byte Distance_TaskID;
byte Distance_TransID;
devStates_t Distance_NwkState;
void Distance_MessageMSGCB(afIncomingMSGPacket_t *MSGpkt);
void Distance_SendTheMessage(unsigned char *theMessageData);



void Distance_Init( byte task_id )
{
  halUARTCfg_t uartConfig;//串口
    
  Distance_TaskID = task_id;
  Distance_NwkState=DEV_INIT;
  Distance_TransID = 0;

  
  Distance_epDesc.endPoint = Distance_ENDPOINT;
  Distance_epDesc.task_id = &Distance_TaskID;
  Distance_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&Distance_SimpleDesc;
  
  Distance_epDesc.latencyReq = noLatencyReqs;
  afRegister( &Distance_epDesc ); 

}

UINT16 Distance_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Distance_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
       
          case ZDO_STATE_CHANGE:
            Distance_NwkState = (devStates_t)(MSGpkt->hdr.status);
            if(Distance_NwkState==DEV_END_DEVICE)
            {
              P1_0=~P1_0;
              osal_set_event(Distance_TaskID,SEND_DATA_EVENT);
            }
            break;
            
          default:
            break;
      }
      osal_msg_deallocate( (uint8 *)MSGpkt );
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Distance_TaskID );
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  if(events&SEND_DATA_EVENT)
  {
    Init_GPIO();
    Init_Key1();
    TRIG =0;      //TRIG触发测距
    Delay_10us(); //给至少10us的高电平信号
    TRIG =1;      //TRIG触发终止
    Delayms(50);//延时周期
    osal_start_timerEx(Distance_TaskID,SEND_DATA_EVENT,3000);
    return(events^SEND_DATA_EVENT);
  }
  return 0;
}

void Distance_SendTheMessage(unsigned char *theMessageData)
{ 
  afAddrType_t my_DstAddr;
  my_DstAddr.addrMode=(afAddrMode_t)Addr16Bit;
  my_DstAddr.endPoint=Distance_ENDPOINT;
  my_DstAddr.addr.shortAddr=0x0000; 
  AF_DataRequest(&my_DstAddr
  ,&Distance_epDesc
  ,Distance_CLUSTERID
  ,osal_strlen("EndDevice")+1
  ,theMessageData
  ,&Distance_TransID
  ,AF_DISCV_ROUTE
  ,AF_DEFAULT_RADIUS);
}

/*********************************************************************
 */
HAL_ISR_FUNCTION( halKeyPort1Isr, P1INT_VECTOR )
{
  
  if (HAL_KEY_SW_7_PXIFG & HAL_KEY_SW_7_BIT)
  {
    Init_T1();
    while(ECHO);
    L=T1CNTL; 
    H=T1CNTH;
    s=256*H;
    distance=((s+L)*340)/30000.0;
    fDistance=distance/2;
    theMessageData[0]='0'+(int)fDistance/100;
    theMessageData[1]='0'+((int)fDistance-(int)fDistance/100*100)/10;
    theMessageData[2]='0'+(int)fDistance%10;
    Distance_SendTheMessage(theMessageData);
    T1CNTL=0x0000;
    T1CNTH=0x0000;
    P1IFG = 0;             //清中断标志 
    P1IF = 0;             //清中断标志 
  }
}
