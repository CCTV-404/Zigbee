/******************************************************************************
  Filename:       Gps.c
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

#include "Gps.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif
#include "MT.h"
#include "mt_uart.h"
/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif  
#include<ioCC2530.h>
#include <string.h>
#define uint unsigned int
#define uchar unsigned char
/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * DATA
 */
uchar Recdata[300]="CHENGDU JIAJIE TECH.INC";
uchar Recdata_GPGGA[70]="CHENGDU JIAJIE TECH.INC";
uint Recdata_GPGGA_start=0;
uchar Recdata_GPGSV[70]="CHENGDU JIAJIE TECH.INC";
uint Recdata_GPGSV_start=0;
uchar Recdata_GPGSA[70]="CHENGDU JIAJIE TECH.INC";
uint Recdata_GPGSA_start=0;
uchar Recdata_GPRMC[70]="CHENGDU JIAJIE TECH.INC";
uint Recdata_GPRMC_start=0;
uchar Recdata_GPVTG[70]="CHENGDU JIAJIE TECH.INC";
uint Recdata_GPVTG_start=0;
uchar UTC[6]="hhmmss";
uchar JingDu[10]="aaaaa,bbbb";
uchar WeiDu[9]="aaaa.bbbb";
uchar GaoDu[5]="mmm.m";
uchar WeiXing[1]="x";
uchar JiBie[1]="y";
uchar RXTXflag = 1;
uchar temp;
uint  datanumber = 0;
uint  stringlen;
uint check=0;
uint position=6;
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
const cId_t Gps_ClusterList[Gps_MAX_CLUSTERS] =
{
  Gps_CLUSTERID
};

const SimpleDescriptionFormat_t Gps_SimpleDesc =
{
  Gps_ENDPOINT,              //  int Endpoint;
  Gps_PROFID,                //  uint16 AppProfId[2];
  Gps_DEVICEID,              //  uint16 AppDeviceId[2];
  Gps_DEVICE_VERSION,        //  int   AppDevVer:4;
  Gps_FLAGS,                 //  int   AppFlags:4;
  Gps_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)Gps_ClusterList,  //  byte *pAppInClusterList;
  Gps_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)Gps_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in Gps_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t Gps_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte Gps_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // Gps_Init() is called.
devStates_t Gps_NwkState;


byte Gps_TransID;  // This is the unique message ID (counter)

afAddrType_t Gps_DstAddr;

byte GpsMeasurement_TaskID;
devStates_t GpsMeasurement_NwkState;
byte GpsMeasurement_TransID;

float fGps;
//uchar Recdata[300]="CHENGDU JIAJIE TECH.INC";
unsigned int flag=0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Gps_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void Gps_HandleKeys( byte shift, byte keys );
static void Gps_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void Gps_SendTheMessage( void );

void handle();
#if defined( IAR_ARMCM3_LM )
static void Gps_ProcessRtosMessage( void );
#endif

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Gps_Init
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
void Gps_Init( uint8 task_id )
{
  halUARTCfg_t uartConfig;
  Gps_TaskID = task_id;
  Gps_NwkState = DEV_INIT;
  Gps_TransID = 0;
  
  MT_UartInit();
  //initUARTSEND();
  MT_UartRegisterTaskID(task_id);

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  Gps_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  Gps_DstAddr.endPoint = 0;
  Gps_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  Gps_epDesc.endPoint = Gps_ENDPOINT;
  Gps_epDesc.task_id = &Gps_TaskID;
  Gps_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&Gps_SimpleDesc;
  Gps_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &Gps_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( Gps_TaskID );

  uartConfig.configured = TRUE;
  uartConfig.baudRate = HAL_UART_BR_9600;
  uartConfig.flowControl = FALSE;
  //uartConfig.callBackFunc         = rxCB;
  HalUARTOpen(0,&uartConfig);
  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "Gps", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( Gps_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( Gps_TaskID, Match_Desc_rsp );

#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, Gps_RTOS_MSG_EVT );
#endif
}

/*********************************************************************
 * @fn      Gps_ProcessEvent
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
uint16 Gps_ProcessEvent( uint8 task_id, uint16 events )
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
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Gps_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          Gps_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          Gps_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
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
          Gps_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          Gps_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (Gps_NwkState == DEV_ZB_COORD)
              || (Gps_NwkState == DEV_ROUTER)
              || (Gps_NwkState == DEV_END_DEVICE) )
          {
            // Start sending "the" message in a regular interval.
            osal_start_timerEx( Gps_TaskID,
                                Gps_SEND_MSG_EVT,
                                Gps_SEND_MSG_TIMEOUT );
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Gps_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in Gps_Init()).
  if ( events & Gps_SEND_MSG_EVT )
  {
    // Send "the" message
    Gps_SendTheMessage();

    // Setup to send message again
    osal_start_timerEx( Gps_TaskID,
                        Gps_SEND_MSG_EVT,
                        Gps_SEND_MSG_TIMEOUT );

    // return unprocessed events
    return (events ^ Gps_SEND_MSG_EVT);
  }

  
#if defined( IAR_ARMCM3_LM )
  // Receive a message from the RTOS queue
  if ( events & Gps_RTOS_MSG_EVT )
  {
    // Process message from RTOS queue
    Gps_ProcessRtosMessage();

    // return unprocessed events
    return (events ^ Gps_RTOS_MSG_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      Gps_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void Gps_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Gps LED
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
            Gps_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            Gps_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            Gps_DstAddr.endPoint = pRsp->epList[0];

            // Gps LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      Gps_HandleKeys
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
static void Gps_HandleKeys( uint8 shift, uint8 keys )
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
                            Gps_epDesc.endPoint,
                            Gps_PROFID,
                            Gps_MAX_CLUSTERS, (cId_t *)Gps_ClusterList,
                            Gps_MAX_CLUSTERS, (cId_t *)Gps_ClusterList,
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
                        Gps_PROFID,
                        Gps_MAX_CLUSTERS, (cId_t *)Gps_ClusterList,
                        Gps_MAX_CLUSTERS, (cId_t *)Gps_ClusterList,
                        FALSE );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      Gps_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void Gps_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case Gps_CLUSTERID:
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
 * @fn      Gps_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
static void Gps_SendTheMessage( void )
{
  unsigned char theMessageData[33] = "Gps:";
  for (unsigned char i=0;i<6;i++)
    theMessageData[i]=UTC[i];
  for (unsigned char i=0;i<9;i++)
    theMessageData[i+6]=WeiDu[i];
  for (unsigned char i=0;i<10;i++)
    theMessageData[i+15]=JingDu[i];
  for (unsigned char i=0;i<1;i++)
    theMessageData[i+25]=JiBie[i];
  for (unsigned char i=0;i<2;i++)
    theMessageData[i+26]=WeiXing[i];
  for (unsigned char i=0;i<5;i++)
    theMessageData[i+28]=GaoDu[i];
  
  /*_ltoa( (uint32)(UTC), &theMessageData[4], 10 );
  _ltoa( (uint32)(WeiDu), &theMessageData[10], 10 );
  _ltoa( (uint32)(JingDu), &theMessageData[19], 10 );
  _ltoa( (uint32)(JiBie), &theMessageData[29], 10 );
  _ltoa( (uint32)(WeiXing), &theMessageData[30], 10 );
  _ltoa( (uint32)(GaoDu), &theMessageData[32], 10 );*/
  
  for (unsigned char i=0; i<50-1; i++)
  {
    if (theMessageData[i] == 0x00 )
    {
      theMessageData[i] = ' ';
    }
  }

  if ( AF_DataRequest( &Gps_DstAddr, &Gps_epDesc,
                       Gps_CLUSTERID,
                       (byte)osal_strlen( theMessageData ),
                       (byte *)&theMessageData,
                       &Gps_TransID,
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
 * @fn      Gps_ProcessRtosMessage
 *
 * @brief   Receive message from RTOS queue, send response back.
 *
 * @param   none
 *
 * @return  none
 */
static void Gps_ProcessRtosMessage( void )
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

uint16 GpsMeasurement_ProcessEvent( uint8 task_id, uint16 events )
{
  if ( events & GPSMEASUREMENT_EVT )
  {
    osal_start_timerEx( GpsMeasurement_TaskID,
                        GPSMEASUREMENT_EVT,
                        GPSMEASUREMENT_TIMEOUT );
  
    HalUARTRead(0, Recdata, 300);
    handle();
    flag++;
    // return unprocessed events
    return (events ^ GPSMEASUREMENT_EVT);
  }

  // Discard unknown events
  return 0;
}

void GpsMeasurement_Init( uint8 task_id )
{
  GpsMeasurement_TaskID = task_id;
  GpsMeasurement_TransID = 0;
  
  osal_start_timerEx( GpsMeasurement_TaskID,
                        GPSMEASUREMENT_EVT,
                        GPSMEASUREMENT_TIMEOUT );
  
  RegisterForKeys( GpsMeasurement_TaskID );  
}

void handle()
{
  check=0;
  Recdata_GPGGA_start=0;
  Recdata_GPGSV_start=0;
  Recdata_GPGSA_start=0;
  Recdata_GPRMC_start=0;
  Recdata_GPVTG_start=0;
  while(check<295)
  {
      if(Recdata[check]=='$'&&Recdata[check+1]=='G'&&Recdata[check+2]=='P'
        &&Recdata[check+3]=='G'&&Recdata[check+4]=='G'&&Recdata[check+5]=='A')
      {
         position=check+6;
         while(Recdata_GPGGA_start<70&&Recdata[position++]!='$')
         {
           Recdata_GPGGA[Recdata_GPGGA_start++]=Recdata[position-1];
           Recdata_GPGSV_start=0;
           Recdata_GPGSA_start=0;
           Recdata_GPRMC_start=0;
           Recdata_GPVTG_start=0;
         }
         //uint GaoDu_count=0;
         
         
         for (uint i=0;i<6;i++)
           UTC[i]=Recdata_GPGGA[i+1];
         if (Recdata_GPGGA[12]==',')
         {
           for (uint i=0;i<9;i++)
             WeiDu[i]='*';
           for (uint i=0;i<10;i++)
             JingDu[i]='*';
           for (uint i=0;i<2;i++)
             WeiXing[i]='*';
           for (uint i=0;i<5;i++)
             GaoDu[i]='*';
         }
         else
         {
           for (uint i=0;i<9;i++)
             WeiDu[i]=Recdata_GPGGA[i+12];
           for (uint i=0;i<10;i++)
             JingDu[i]=Recdata_GPGGA[i+24];
         }
         uint GaoDu_count=0;
         while(Recdata_GPGGA[GaoDu_count]!='M')
         {
           if(GaoDu_count<70)
             GaoDu_count++;
         }
         if(Recdata_GPGGA[GaoDu_count]=='M')
         {
           if(Recdata_GPGGA[GaoDu_count-6]>'0'&&Recdata_GPGGA[GaoDu_count-6]<='9')
           {
             GaoDu[0]=Recdata_GPGGA[GaoDu_count-6];
             GaoDu[1]=Recdata_GPGGA[GaoDu_count-5];
             GaoDu[2]=Recdata_GPGGA[GaoDu_count-4];
             GaoDu[3]=Recdata_GPGGA[GaoDu_count-3];
             GaoDu[4]=Recdata_GPGGA[GaoDu_count-2];
           }
           else if(Recdata_GPGGA[GaoDu_count-5]>'0'&&Recdata_GPGGA[GaoDu_count-6]<='9')
           {
             GaoDu[0]='0';
             GaoDu[1]=Recdata_GPGGA[GaoDu_count-5];
             GaoDu[2]=Recdata_GPGGA[GaoDu_count-4];
             GaoDu[3]=Recdata_GPGGA[GaoDu_count-3];
             GaoDu[4]=Recdata_GPGGA[GaoDu_count-2];
           }
           else 
           {
             GaoDu[0]='0';
             GaoDu[1]='0';
             GaoDu[2]=Recdata_GPGGA[GaoDu_count-4];
             GaoDu[3]=Recdata_GPGGA[GaoDu_count-3];
             GaoDu[4]=Recdata_GPGGA[GaoDu_count-2];
           }
         }
      }
      else if(Recdata[check]=='$'&&Recdata[check+1]=='G'&&Recdata[check+2]=='P'
         &&Recdata[check+3]=='G'&&Recdata[check+4]=='S'&&Recdata[check+5]=='V')
      {
         position=check+6;
         while(Recdata[position++]!='$'&&Recdata_GPGSV_start<70)
         {
           Recdata_GPGSV[Recdata_GPGSV_start++]=Recdata[position-1];
           Recdata_GPGGA_start=0;
           Recdata_GPGSA_start=0;
           Recdata_GPRMC_start=0;
           Recdata_GPVTG_start=0;
         }
		 for (uint i=0;i<2;i++)
           WeiXing[i]=Recdata_GPGSV[i+5];
      }
      else if(Recdata[check]=='$'&&Recdata[check+1]=='G'&&Recdata[check+2]=='P'
          &&Recdata[check+3]=='G'&&Recdata[check+4]=='S'&&Recdata[check+5]=='A')
      {
          position=check+6;
          while(Recdata_GPGSA_start<70&&Recdata[position++]!='$')
          {
            Recdata_GPGSA[Recdata_GPGSA_start++]=Recdata[position-1];
            Recdata_GPGSV_start=0;
            Recdata_GPGGA_start=0;
            Recdata_GPRMC_start=0;
            Recdata_GPVTG_start=0;
          }
          JiBie[0]=Recdata_GPGSA[3];
      }
      else if(Recdata[check]=='$'&&Recdata[check+1]=='G'&&Recdata[check+2]=='P'
           &&Recdata[check+3]=='R'&&Recdata[check+4]=='M'&&Recdata[check+5]=='C')
      {
          position=check+6;
          while(Recdata_GPRMC_start<70&&Recdata[position++]!='$')
          {
            Recdata_GPRMC[Recdata_GPRMC_start++]=Recdata[position-1];
            Recdata_GPGSV_start=0;
            Recdata_GPGSA_start=0;
            Recdata_GPGGA_start=0;
            Recdata_GPVTG_start=0;
          }
          for (uint i=0;i<6;i++)
            UTC[i]=Recdata_GPRMC[i+1];
          if (Recdata_GPRMC[14]==',')
          {
            for (uint i=0;i<9;i++)
              WeiDu[i]='*';
            for (uint i=0;i<10;i++)
              JingDu[i]='*';
          }
          else
          {
            for (uint i=0;i<9;i++)
              WeiDu[i]=Recdata_GPRMC[i+14];
            for (uint i=0;i<10;i++)
              JingDu[i]=Recdata_GPRMC[i+26];
          }
      }
      else if(Recdata[check]=='$'&&Recdata[check+1]=='G'&&Recdata[check+2]=='P'
           &&Recdata[check+3]=='V'&&Recdata[check+4]=='T'&&Recdata[check+5]=='G')
      {
          position=check+6;
          while(Recdata_GPVTG_start<70&&Recdata[position++]!='$')
          {
            Recdata_GPVTG[Recdata_GPVTG_start++]=Recdata[position-1];
            Recdata_GPGSV_start=0;
            Recdata_GPGSA_start=0;
            Recdata_GPRMC_start=0;
            Recdata_GPGGA_start=0;
          }
      }
      check=check+1;
  } 	  
}


/*********************************************************************
 */
