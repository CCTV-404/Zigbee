/******************************************************************************
  Filename:       Posture.c
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
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "Posture.h"
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

#include "MPU.h"
#define SEND_DATA_EVENT 0x01
float fPosture;
const cId_t Posture_ClusterList[Posture_MAX_CLUSTERS] =
{
  Posture_CLUSTERID
};



const SimpleDescriptionFormat_t Posture_SimpleDesc =
{
  Posture_ENDPOINT,              //  int Endpoint;
  Posture_PROFID,                //  uint16 AppProfId[2];
  Posture_DEVICEID,              //  uint16 AppDeviceId[2];
  Posture_DEVICE_VERSION,        //  int   AppDevVer:4;
  Posture_FLAGS,                 //  int   AppFlags:4;
  
  
  0,          //  byte  AppNumInClusters;
  (cId_t *)NULL,  //  byte *pAppInClusterList;
  Posture_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)Posture_ClusterList   //  byte *pAppInClusterList;
};

unsigned char TempDATA;
endPointDesc_t Posture_epDesc;
byte Posture_TaskID;
byte Posture_TransID;
devStates_t Posture_NwkState;
void Posture_MessageMSGCB(afIncomingMSGPacket_t *MSGpkt);
void Posture_SendTheMessage(void);



void Posture_Init( byte task_id )
{
  halUARTCfg_t uartConfig;//´®¿Ú
    
  Posture_TaskID = task_id;
  Posture_NwkState=DEV_INIT;
  Posture_TransID = 0;

  
  Posture_epDesc.endPoint = Posture_ENDPOINT;
  Posture_epDesc.task_id = &Posture_TaskID;
  Posture_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&Posture_SimpleDesc;
  
  Posture_epDesc.latencyReq = noLatencyReqs;
  afRegister( &Posture_epDesc ); 

}

UINT16 Posture_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Posture_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
       
          case ZDO_STATE_CHANGE:
            Posture_NwkState = (devStates_t)(MSGpkt->hdr.status);
            if(Posture_NwkState==DEV_END_DEVICE)
            {
              P1_0=~P1_0;
              osal_set_event(Posture_TaskID,SEND_DATA_EVENT);
            }
            break;
            
          default:
            break;
      }
      osal_msg_deallocate( (uint8 *)MSGpkt );
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Posture_TaskID );
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  if(events&SEND_DATA_EVENT)
  {
    _MPU_Init();
    NOP();
    read();
    Posture_SendTheMessage();
    osal_start_timerEx(Posture_TaskID,SEND_DATA_EVENT,3000);
    return(events^SEND_DATA_EVENT);
  }
  return 0;
}

void Posture_SendTheMessage(void)
{ 
  extern double ax,ay,az,jx,jy,jz; 
  unsigned char theMessageData[24]="EndDevice";

  afAddrType_t my_DstAddr;
  if(ax<0)
  {
    theMessageData[18]='0';
    ax=-ax;
  }
  else
  {
    theMessageData[18]='1';
  }
  if(ay<0)
  {
    theMessageData[19]='0';
    ay=-ay;
  }
  else
  {
    theMessageData[19]='1';
  }
  if(az<0)
  {
    theMessageData[20]='0';
    az=-az;
  }
  else
  {
    theMessageData[20]='1';
  }
  if(jx<0)
  {
    theMessageData[21]='0';
    jx=-jx;
  }
  else
  {
    theMessageData[21]='1';
  }
  if(jy<0)
  {
    theMessageData[22]='0';
    jy=-jy;
  }
  else
  {
    theMessageData[22]='1';
  }
  if(jz<0)
  {
    theMessageData[23]='0';
    jz=-jz;
  }
  else
  {
    theMessageData[23]='1';
  }
  theMessageData[0]='0'+(int)ax/10;
  theMessageData[1]='0'+(int)ax%10;
  theMessageData[2]='0'+(int)((ax-(int)(ax/10)*10-(int)ax%10)/0.1);
  theMessageData[3]='0'+(int)ay/10;
  theMessageData[4]='0'+(int)ay%10;
  theMessageData[5]='0'+(int)((ay-(int)(ay/10)*10-(int)ay%10)/0.1);
  theMessageData[6]='0'+(int)az/10;
  theMessageData[7]='0'+(int)az%10;
  theMessageData[8]='0'+(int)((az-(int)(az/10)*10-(int)az%10)/0.1);
  theMessageData[9]='0'+(int)jx/10;
  theMessageData[10]='0'+(int)jx%10;
  theMessageData[11]='0'+(int)((jx-(int)(jx/10)*10-(int)jx%10)/0.1);
  theMessageData[12]='0'+(int)jy/10;
  theMessageData[13]='0'+(int)jy%10;
  theMessageData[14]='0'+(int)((jy-(int)(jy/10)*10-(int)jy%10)/0.1);
  theMessageData[15]='0'+(int)jz/10;
  theMessageData[16]='0'+(int)jz%10;
  theMessageData[17]='0'+(int)((jz-(int)(jz/10)*10-(int)jz%10)/0.1);

  my_DstAddr.addrMode=(afAddrMode_t)Addr16Bit;
  my_DstAddr.endPoint=Posture_ENDPOINT;
  my_DstAddr.addr.shortAddr=0x0000; 


  AF_DataRequest(&my_DstAddr
  ,&Posture_epDesc
  ,Posture_CLUSTERID
  ,24
  ,theMessageData
  ,&Posture_TransID
  ,AF_DISCV_ROUTE
  ,AF_DEFAULT_RADIUS);
}


/*********************************************************************
 */
