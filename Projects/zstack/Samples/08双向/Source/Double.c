/******************************************************************************
Filename:       Double.c
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
#include "Double.h"
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

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif  

/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/
// This list should be filled with Application specific Cluster IDs.
#define SEND_DATA_EVENT 0x01
unsigned char flag='0';
const cId_t Double_ClusterList[Double_MAX_CLUSTERS] =
{
  Double_CLUSTERID
};



const SimpleDescriptionFormat_t Double_SimpleDesc =
{
  Double_ENDPOINT,              //  int Endpoint;
  Double_PROFID,                //  uint16 AppProfId[2];
  Double_DEVICEID,              //  uint16 AppDeviceId[2];
  Double_DEVICE_VERSION,        //  int   AppDevVer:4;
  Double_FLAGS,                 //  int   AppFlags:4;
  
  
  0,          //  byte  AppNumInClusters;
  (cId_t *)NULL,  //  byte *pAppInClusterList;
  Double_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)Double_ClusterList   //  byte *pAppInClusterList;
};

unsigned char TempDATA;
endPointDesc_t Double_epDesc;
byte Double_TaskID;
byte Double_TransID;
devStates_t Double_NwkState;
void Double_MessageMSGCB(afIncomingMSGPacket_t *MSGpkt);
void Double_SendTheMessage(void);



void Double_Init( byte task_id )
{
  halUARTCfg_t uartConfig;//串口
  
  Double_TaskID = task_id;
  Double_NwkState=DEV_INIT;
  Double_TransID = 0;
  
  
  Double_epDesc.endPoint = Double_ENDPOINT;
  Double_epDesc.task_id = &Double_TaskID;
  Double_epDesc.simpleDesc
    = (SimpleDescriptionFormat_t *)&Double_SimpleDesc;
  
  Double_epDesc.latencyReq = noLatencyReqs;
  afRegister( &Double_epDesc ); 
  
}
//KET1 外部中断方式
void InitKey2(void)
{
  //  P1IEN |= 0X08;
  //  PICTL |= 0X2; // 下降沿触发   
  //  IEN2 |= 0X10;   // 允许P1口中断; 
  //  P1IFG = 0x08;   // 初始化中断标志位
  P1DIR |= 0x13;
  //  EA = 1; 
}

UINT16 Double_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  
  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Double_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        
      case ZDO_STATE_CHANGE:
        Double_NwkState = (devStates_t)(MSGpkt->hdr.status);
        if(Double_NwkState==DEV_END_DEVICE)
        {
          P1_0 = ~P1_0;
          osal_set_event(Double_TaskID,SEND_DATA_EVENT);
        }
        break;
        
      case AF_INCOMING_MSG_CMD:
        Double_MessageMSGCB( MSGpkt );
        break;
        
        
      default:
        break;
      }
      osal_msg_deallocate( (uint8 *)MSGpkt );
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Double_TaskID );
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  if(events&SEND_DATA_EVENT)
  {
    flag='0';
    //InitKey2();
    //Double_SendTheMessage();
    //osal_start_timerEx(Double_TaskID,SEND_DATA_EVENT,3000);
    return(events^SEND_DATA_EVENT);
  }
  return 0;
}

void Double_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  unsigned char buffer[4]; 
  switch ( pkt->clusterId )
  {
  case Double_CLUSTERID:
    osal_memcpy(buffer, pkt->cmd.Data, 4);
    if(buffer[0] == '1')       
    {
      HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );;
      HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );;
      HalLedSet ( HAL_LED_3, HAL_LED_MODE_ON );;
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );;
    }
    if(buffer[0] == '2')       
    {
      HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );;
      HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );;
      HalLedSet ( HAL_LED_3, HAL_LED_MODE_OFF );;
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );;
    }
    break;
  }
}


void Double_SendTheMessage(void)
{ 
  unsigned char theMessageData[10]="1";
  afAddrType_t my_DstAddr;
  my_DstAddr.addrMode=(afAddrMode_t)Addr16Bit;
  my_DstAddr.endPoint=Double_ENDPOINT;
  my_DstAddr.addr.shortAddr=0x0000; 
  theMessageData[0]=flag;
  AF_DataRequest(&my_DstAddr
                 ,&Double_epDesc
                   ,Double_CLUSTERID
                     ,1
                       ,theMessageData
                         ,&Double_TransID
                           ,AF_DISCV_ROUTE
                             ,AF_DEFAULT_RADIUS);
}
