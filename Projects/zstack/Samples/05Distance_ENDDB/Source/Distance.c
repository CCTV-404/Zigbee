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

#include "Distance.h"
#include "DebugTrace.h"

#include "HC_SR04.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

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
  Distance_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)Distance_ClusterList,  //  byte *pAppInClusterList;
  Distance_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)Distance_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in Distance_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t Distance_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte Distance_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // Distance_Init() is called.
devStates_t Distance_NwkState;


byte Distance_TransID;  // This is the unique message ID (counter)

afAddrType_t Distance_DstAddr;

byte DistanceMeasurement_TaskID;
devStates_t DistanceMeasurement_NwkState;
byte DistanceMeasurement_TransID;

float fDistance;
float  distance;
float s;
int dis;
uint H;
uint H1;
uint L;
uint L1;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Distance_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void Distance_HandleKeys( byte shift, byte keys );
static void Distance_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void Distance_SendTheMessage( void );

#if defined( IAR_ARMCM3_LM )
static void Distance_ProcessRtosMessage( void );
#endif

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Distance_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void Distance_Init( uint8 task_id )
{
  Distance_TaskID = task_id;
  Distance_NwkState = DEV_INIT;
  Distance_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  Distance_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  Distance_DstAddr.endPoint = 0;
  Distance_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  Distance_epDesc.endPoint = Distance_ENDPOINT;
  Distance_epDesc.task_id = &Distance_TaskID;
  Distance_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&Distance_SimpleDesc;
  Distance_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &Distance_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( Distance_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "Distance", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( Distance_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( Distance_TaskID, Match_Desc_rsp );

#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, Distance_RTOS_MSG_EVT );
#endif
}

/*********************************************************************
 * @fn      Distance_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 Distance_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Distance_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          Distance_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          Distance_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          Distance_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          Distance_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (Distance_NwkState == DEV_ZB_COORD)
              || (Distance_NwkState == DEV_ROUTER)
              || (Distance_NwkState == DEV_END_DEVICE) )
          {
            // Start sending "the" message in a regular interval.
            osal_start_timerEx( Distance_TaskID,
                                Distance_SEND_MSG_EVT,
                                Distance_SEND_MSG_TIMEOUT );
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Distance_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in Distance_Init()).
  if ( events & Distance_SEND_MSG_EVT )
  {
    // Send "the" message
    Distance_SendTheMessage();

    // Setup to send message again
    osal_start_timerEx( Distance_TaskID,
                        Distance_SEND_MSG_EVT,
                        Distance_SEND_MSG_TIMEOUT );

    // return unprocessed events
    return (events ^ Distance_SEND_MSG_EVT);
  }

  
#if defined( IAR_ARMCM3_LM )
  // Receive a message from the RTOS queue
  if ( events & Distance_RTOS_MSG_EVT )
  {
    // Process message from RTOS queue
    Distance_ProcessRtosMessage();

    // return unprocessed events
    return (events ^ Distance_RTOS_MSG_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      Distance_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void Distance_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Distance LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            Distance_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            Distance_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            Distance_DstAddr.endPoint = pRsp->epList[0];

            // Distance LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      Distance_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void Distance_HandleKeys( uint8 shift, uint8 keys )
{
  zAddrType_t dstAddr;

  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
      // Since SW1 isn't used for anything else in this application...
#if defined( SWITCH1_BIND )
      // we can use SW1 to simulate SW2 for devices that only have one switch,
      keys |= HAL_KEY_SW_2;
#elif defined( SWITCH1_MATCH )
      // or use SW1 to simulate SW4 for devices that only have one switch
      keys |= HAL_KEY_SW_4;
#endif
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request for the mandatory endpoint
      dstAddr.addrMode = Addr16Bit;
      dstAddr.addr.shortAddr = 0x0000; // Coordinator
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                            Distance_epDesc.endPoint,
                            Distance_PROFID,
                            Distance_MAX_CLUSTERS, (cId_t *)Distance_ClusterList,
                            Distance_MAX_CLUSTERS, (cId_t *)Distance_ClusterList,
                            FALSE );
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
      // Initiate a Match Description Request (Service Discovery)
      dstAddr.addrMode = AddrBroadcast;
      dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                        Distance_PROFID,
                        Distance_MAX_CLUSTERS, (cId_t *)Distance_ClusterList,
                        Distance_MAX_CLUSTERS, (cId_t *)Distance_ClusterList,
                        FALSE );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      Distance_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void Distance_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case Distance_CLUSTERID:
      // "the" message
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
  }
}

/*********************************************************************
 * @fn      Distance_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
static void Distance_SendTheMessage( void )
{
  unsigned char theMessageData[15] = "Distance:";
  
  _ltoa( (uint32)(fDistance), &theMessageData[9], 10 );
  
  for (unsigned char i=0; i<15-1; i++)
  {
    if (theMessageData[i] == 0x00 )
    {
      theMessageData[i] = ' ';
    }
  }

  if ( AF_DataRequest( &Distance_DstAddr, &Distance_epDesc,
                       Distance_CLUSTERID,
                       (byte)osal_strlen( theMessageData ) + 1,
                       (byte *)&theMessageData,
                       &Distance_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
}

#if defined( IAR_ARMCM3_LM )
/*********************************************************************
 * @fn      Distance_ProcessRtosMessage
 *
 * @brief   Receive message from RTOS queue, send response back.
 *
 * @param   none
 *
 * @return  none
 */
static void Distance_ProcessRtosMessage( void )
{
  osalQueue_t inMsg;

  if ( osal_queue_receive( OsalQueue, &inMsg, 0 ) == pdPASS )
  {
    uint8 cmndId = inMsg.cmnd;
    uint32 counter = osal_build_uint32( inMsg.cbuf, 4 );

    switch ( cmndId )
    {
      case CMD_INCR:
        counter += 1;  /* Increment the incoming counter */
                       /* Intentionally fall through next case */

      case CMD_ECHO:
      {
        userQueue_t outMsg;

        outMsg.resp = RSP_CODE | cmndId;  /* Response ID */
        osal_buffer_uint32( outMsg.rbuf, counter );    /* Increment counter */
        osal_queue_send( UserQueue1, &outMsg, 0 );  /* Send back to UserTask */
        break;
      }
      
      default:
        break;  /* Ignore unknown command */    
    }
  }
}

#endif

//***********************************************************************************

uint16 DistanceMeasurement_ProcessEvent( uint8 task_id, uint16 events )
{
  if ( events & DISTANCEMEASUREMENT_EVT )
  {
  osal_start_timerEx( DistanceMeasurement_TaskID,
                        DISTANCEMEASUREMENT_EVT,
                        DISTANCEMEASUREMENT_TIMEOUT );

    SysClkSet32M();
    Init_GPIO();
    Init_Key1();
    TRIG =0;      //TRIG触发测距
    Delay_10us(); //给至少10us的高电平信号
    TRIG =1;      //TRIG触发终止
    Delayms(50);//延时周期
    
    //HalLedSet ( HAL_LED_2, HAL_LED_MODE_TOGGLE );

    // return unprocessed events
    return (events ^ DISTANCEMEASUREMENT_EVT);
  }

  // Discard unknown events
  return 0;
}

void DistanceMeasurement_Init( uint8 task_id )
{
  DistanceMeasurement_TaskID = task_id;
  DistanceMeasurement_TransID = 0;
  
  osal_start_timerEx( DistanceMeasurement_TaskID,
                        DISTANCEMEASUREMENT_EVT,
                        DISTANCEMEASUREMENT_TIMEOUT );
  
  RegisterForKeys( DistanceMeasurement_TaskID );  
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
    T1CNTL=0x0000;
    T1CNTH=0x0000;
    P1IFG = 0;             //清中断标志 
    P1IF = 0;             //清中断标志 
  }
}
