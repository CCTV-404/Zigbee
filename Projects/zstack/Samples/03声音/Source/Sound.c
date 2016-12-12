/******************************************************************************
  Filename:       Sound.c
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
#include "Sound.h"
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

/* SW_6 is at P0.1 */
#define HAL_KEY_SW_7_PORT   P1
#define HAL_KEY_SW_7_BIT    BV(3)
#define HAL_KEY_SW_7_SEL    P1SEL
#define HAL_KEY_SW_7_DIR    P1DIR

/* edge interrupt */
#define HAL_KEY_SW_7_EDGEBIT  BV(0)
#define HAL_KEY_SW_7_EDGE     HAL_KEY_RISING_EDGE


/* SW_6 interrupts */
#define HAL_KEY_SW_7_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_SW_7_IENBIT   BV(4) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_7_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_7_ICTLBIT  BV(3) /* P0IEN - P0.1 enable/disable bit */
#define HAL_KEY_SW_7_PXIFG    P1IFG /* Interrupt flag at source */

#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include <string.h>
//#include "Common.h"
#include "DebugTrace.h"
#include "Sound.h"
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


#define SEND_DATA_EVENT 0x01
unsigned char flag='0';
const cId_t Sound_ClusterList[Sound_MAX_CLUSTERS] =
{
  Sound_CLUSTERID
};



const SimpleDescriptionFormat_t Sound_SimpleDesc =
{
  Sound_ENDPOINT,              //  int Endpoint;
  Sound_PROFID,                //  uint16 AppProfId[2];
  Sound_DEVICEID,              //  uint16 AppDeviceId[2];
  Sound_DEVICE_VERSION,        //  int   AppDevVer:4;
  Sound_FLAGS,                 //  int   AppFlags:4;
  
  
  0,          //  byte  AppNumInClusters;
  (cId_t *)NULL,  //  byte *pAppInClusterList;
  Sound_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)Sound_ClusterList   //  byte *pAppInClusterList;
};

unsigned char TempDATA;
endPointDesc_t Sound_epDesc;
byte Sound_TaskID;
byte Sound_TransID;
devStates_t Sound_NwkState;
void Sound_MessageMSGCB(afIncomingMSGPacket_t *MSGpkt);
void Sound_SendTheMessage(void);



void Sound_Init( byte task_id )
{
  halUARTCfg_t uartConfig;//串口
    
  Sound_TaskID = task_id;
  Sound_NwkState=DEV_INIT;
  Sound_TransID = 0;
 
  Sound_epDesc.endPoint = Sound_ENDPOINT;
  Sound_epDesc.task_id = &Sound_TaskID;
  Sound_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&Sound_SimpleDesc;
  
  Sound_epDesc.latencyReq = noLatencyReqs;
  afRegister( &Sound_epDesc ); 

}
//KET1 外部中断方式
void InitKey2(void)
{
  P1IEN |= 0X08;
  PICTL |= 0X2; // 下降沿触发   
  IEN2 |= 0X10;   // 允许P1口中断; 
  P1IFG = 0x08;   // 初始化中断标志位
  P1DIR |= 0x13;
  EA = 1; 
}
UINT16 Sound_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Sound_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
       
          case ZDO_STATE_CHANGE:
            Sound_NwkState = (devStates_t)(MSGpkt->hdr.status);
            if(Sound_NwkState==DEV_END_DEVICE)
            {
              P1_0=~P1_0;
              osal_set_event(Sound_TaskID,SEND_DATA_EVENT);
            }
            break;
            
          default:
            break;
      }
      osal_msg_deallocate( (uint8 *)MSGpkt );
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Sound_TaskID );
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  if(events&SEND_DATA_EVENT)
  {
    flag='0';
    InitKey2();
    Sound_SendTheMessage();
    osal_start_timerEx(Sound_TaskID,SEND_DATA_EVENT,3000);
    return(events^SEND_DATA_EVENT);
  }
  return 0;
}

void Sound_SendTheMessage(void)
{ 
  unsigned char theMessageData[10]="EndDevice";

  afAddrType_t my_DstAddr;

  my_DstAddr.addrMode=(afAddrMode_t)Addr16Bit;
  my_DstAddr.endPoint=Sound_ENDPOINT;
  my_DstAddr.addr.shortAddr=0x0000; 


  theMessageData[0]=flag;

  AF_DataRequest(&my_DstAddr
  ,&Sound_epDesc
  ,Sound_CLUSTERID
  ,osal_strlen("EndDevice")+1
  ,theMessageData
  ,&Sound_TransID
  ,AF_DISCV_ROUTE
  ,AF_DEFAULT_RADIUS);
}




HAL_ISR_FUNCTION( halKeyPort1Isr, P1INT_VECTOR )
{
  
  if (HAL_KEY_SW_7_PXIFG & HAL_KEY_SW_7_BIT)
  {
    flag='1';
    Sound_SendTheMessage( );
    flag='0';
    P1IFG = 0;             //清中断标志 
    P1IF = 0;             //清中断标志 
  }
}
