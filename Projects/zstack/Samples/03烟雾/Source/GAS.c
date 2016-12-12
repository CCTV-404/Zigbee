/******************************************************************************
  Filename:       GAS.c
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
#include "GAS.h"
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
const cId_t GAS_ClusterList[GAS_MAX_CLUSTERS] =
{
  GAS_CLUSTERID
};



const SimpleDescriptionFormat_t GAS_SimpleDesc =
{
  GAS_ENDPOINT,              //  int Endpoint;
  GAS_PROFID,                //  uint16 AppProfId[2];
  GAS_DEVICEID,              //  uint16 AppDeviceId[2];
  GAS_DEVICE_VERSION,        //  int   AppDevVer:4;
  GAS_FLAGS,                 //  int   AppFlags:4;
  
  
  0,          //  byte  AppNumInClusters;
  (cId_t *)NULL,  //  byte *pAppInClusterList;
  GAS_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GAS_ClusterList   //  byte *pAppInClusterList;
};

unsigned char TempDATA;
endPointDesc_t GAS_epDesc;
byte GAS_TaskID;
byte GAS_TransID;
devStates_t GAS_NwkState;
void GAS_MessageMSGCB(afIncomingMSGPacket_t *MSGpkt);
void GAS_SendTheMessage(void);


void Scan()
{
  P1SEL &= ~0x08;
  P1DIR &= ~0x08;
}
void GAS_Init( byte task_id )
{
  halUARTCfg_t uartConfig;//´®¿Ú
    
  GAS_TaskID = task_id;
  GAS_NwkState=DEV_INIT;
  GAS_TransID = 0;

  
  GAS_epDesc.endPoint = GAS_ENDPOINT;
  GAS_epDesc.task_id = &GAS_TaskID;
  GAS_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GAS_SimpleDesc;
  
  GAS_epDesc.latencyReq = noLatencyReqs;
  afRegister( &GAS_epDesc ); 
  P1DIR &= ~(1<<3);

}

UINT16 GAS_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GAS_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
       
          case ZDO_STATE_CHANGE:
            GAS_NwkState = (devStates_t)(MSGpkt->hdr.status);
            if(GAS_NwkState==DEV_END_DEVICE)
            {
              P1_0=~P1_0;
              osal_set_event(GAS_TaskID,SEND_DATA_EVENT);
            }
            break;
            
          default:
            break;
      }
      osal_msg_deallocate( (uint8 *)MSGpkt );
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GAS_TaskID );
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  if(events&SEND_DATA_EVENT)
  {
    Scan();
    GAS_SendTheMessage();
    osal_start_timerEx(GAS_TaskID,SEND_DATA_EVENT,3000);
    return(events^SEND_DATA_EVENT);
  }
  return 0;
}

void GAS_SendTheMessage(void)
{ 
  unsigned char theMessageData[10]="EndDevice";

  afAddrType_t my_DstAddr;

  my_DstAddr.addrMode=(afAddrMode_t)Addr16Bit;
  my_DstAddr.endPoint=GAS_ENDPOINT;
  my_DstAddr.addr.shortAddr=0x0000; 


  P1DIR &= ~(1<<3);
  
  if(GASIN == 1)
  {
    theMessageData[0] = 'N';
  } else {
    theMessageData[0] = 'Y';
  }

  AF_DataRequest(&my_DstAddr
  ,&GAS_epDesc
  ,GAS_CLUSTERID
  ,1
  ,theMessageData
  ,&GAS_TransID
  ,AF_DISCV_ROUTE
  ,AF_DEFAULT_RADIUS);
}

