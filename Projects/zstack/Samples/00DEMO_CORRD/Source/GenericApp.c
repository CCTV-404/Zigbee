/******************************************************************************
  Filename:       GenericApp.c
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

#include "GenericApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif
#include "MT_UART.h" 
#include "hal_uart.h"
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
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID_TEMHUM,
  GENERICAPP_CLUSTERID_LINGHT,
  GENERICAPP_CLUSTERID_SOUND,
  GENERICAPP_CLUSTERID_POSTURE,
  GENERICAPP_CLUSTERID_DISTANCE,
  GENERICAPP_CLUSTERID_GPS,
  GENERICAPP_CLUSTERID_GAS,
  GENERICAPP_CLUSTERID_DOUBLE
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,              //  int Endpoint;
  GENERICAPP_PROFID,                //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                 //  int   AppFlags:4;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.
devStates_t GenericApp_NwkState;


byte GenericApp_TransID;  // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;

#define ADC_REF_AVDD5 0x80
#define ADC_REF_125_V 0x00
#define ADC_14_BIT 0x30
#define ADC_AIN1_SENS 0x07
#define ADC_TEMP_SENS 0x0E
unsigned int   i;
unsigned int  AdcValue;
unsigned int  value;
unsigned int ADCValue;
unsigned char ADCV[]="";
void Delays(void) {
  unsigned int itemp;
  for(itemp=0;itemp<500;itemp++) {
    asm("nop");
  }
}
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void GenericApp_HandleKeys( byte shift, byte keys );
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void GenericApp_SendTheMessage( unsigned char *theMessageData );

#if defined( IAR_ARMCM3_LM )
static void GenericApp_ProcessRtosMessage( void );
#endif

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void initUARTSEND(void)
{

    CLKCONCMD &= ~0x40;                          //设置系统时钟源为32MHZ晶振
    while(CLKCONSTA & 0x40);                     //等待晶振稳定
    CLKCONCMD &= ~0x47;                          //设置系统主时钟频率为32MHZ
   
  
    //PERCFG  Peripheral-control register
    PERCFG = 0x00;				//位置1 P0口
    //POSEL Port0 function-select register
    P0SEL = 0x3c;				//P0_2,P0_3,P0_4,P0_5用作串口
    P2DIR &= ~0X80;                             //P0优先作为UART1

    U1CSR |= 0x80;				//UART方式
    U1GCR |= 8;				       
    U1BAUD |= 59;				//波特率设为9600
    UTX1IF = 0;                                 //UART1 TX中断标志初始置位0
}
static void rxCB(uint8 port,uint8 event)
{
unsigned  char LED[1];


  HalUARTRead(1, LED, 1);
  P1_1=~P1_1;
  //HalLedBlink(HAL_LED_2,0,50,500);  
  if(LED[0] == '1')
  {
    P1_0=0;
    GenericApp_SendTheMessage(LED);
  }   
  if(LED[0] == '2')
  {
    P1_0=1;
    GenericApp_SendTheMessage(LED);
  }   
  if(LED[0] == '3')
  {
    P1_0=0;
    GenericApp_SendTheMessage(LED);
  }  
  if(LED[0] == '4')
  {
    P1_0=1;
    GenericApp_SendTheMessage(LED);
  }  
  if(LED[0] == '5')
  {
    P1_0=0;
    GenericApp_SendTheMessage(LED);
  }  
  if(LED[0] == '6')
  {
    P1_0=1;
    GenericApp_SendTheMessage(LED);
  }  
}
/*********************************************************************
 * @fn      GenericApp_Init
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
void GenericApp_Init( uint8 task_id )
{
  halUARTCfg_t uartConfig;
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;
  MT_UartInit();
  initUARTSEND();
  MT_UartRegisterTaskID(task_id);
  HalUARTWrite(1,"Hello World\n",12);
  HalUARTWrite(1,"Hello,WeBee\n",12);

  HalUARTWrite(1,"\n",1);
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  GenericApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  GenericApp_DstAddr.endPoint = 0;
  GenericApp_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( GenericApp_TaskID );
  uartConfig.configured = TRUE;
  uartConfig.baudRate = HAL_UART_BR_9600;
  uartConfig.flowControl = FALSE;
  uartConfig.callBackFunc = rxCB;
  HalUARTOpen(1,&uartConfig);
  HalUARTWrite(1,"Hello World\n",12);
  HalUARTWrite(1,"Hello,WeBee\n",12);
  HalUARTWrite(1,"Hello World\n",12);
  HalUARTWrite(1,"\n",1);

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );

#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, GENERICAPP_RTOS_MSG_EVT );
#endif
}

/*********************************************************************
 * @fn      GenericApp_ProcessEvent
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
uint16 GenericApp_ProcessEvent( uint8 task_id, uint16 events )
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
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          GenericApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          GenericApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
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
          GenericApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (GenericApp_NwkState == DEV_ZB_COORD)
              || (GenericApp_NwkState == DEV_ROUTER)
              || (GenericApp_NwkState == DEV_END_DEVICE) )
          {
            // Start sending "the" message in a regular interval.
            osal_start_timerEx( GenericApp_TaskID,
                                GENERICAPP_SEND_MSG_EVT,
                                GENERICAPP_SEND_MSG_TIMEOUT );
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in GenericApp_Init()).
  if ( events & GENERICAPP_SEND_MSG_EVT )
  {
    // Send "the" message
    //GenericApp_SendTheMessage();

    // Setup to send message again
    ADCCON3 = (ADC_REF_AVDD5 | ADC_14_BIT | ADC_AIN1_SENS);
    //设置ADCCON1，转换模式
    ADCCON1 |= 0x30;
    //开始单次转换
    ADCCON1 |= 0x40;
    //等待AD转换完成
    while(!(ADCCON1 & 0x80));
    //保存ADC转换结果
    ADCValue = ADCL >> 2;
    ADCValue |= (((unsigned int)ADCH) << 6);
    Delays();
    if(ADCValue!=0)
    {
      ADCV[0]=ADCValue/1000+'0';
      ADCV[1]=(ADCValue-ADCValue/1000*1000)/100+'0';
      ADCV[2]=(ADCValue-ADCValue/100*100)/10+'0';
      ADCV[3]=ADCValue%10+'0';
      HalUARTWrite(1,"$u,14,00,",9);
      HalUARTWrite(1,ADCV,4);
      HalUARTWrite(1,",check,cr#",10);
      HalUARTWrite(1,"\n",1);
    }
    osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_SEND_MSG_EVT,
                        GENERICAPP_SEND_MSG_TIMEOUT );

    // return unprocessed events
    return (events ^ GENERICAPP_SEND_MSG_EVT);
  }

  
#if defined( IAR_ARMCM3_LM )
  // Receive a message from the RTOS queue
  if ( events & GENERICAPP_RTOS_MSG_EVT )
  {
    // Process message from RTOS queue
    GenericApp_ProcessRtosMessage();

    // return unprocessed events
    return (events ^ GENERICAPP_RTOS_MSG_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  { 
    case End_Device_Bind_rsp:        
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
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
            GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            GenericApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      GenericApp_HandleKeys
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
static void GenericApp_HandleKeys( uint8 shift, uint8 keys )
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
                            GenericApp_epDesc.endPoint,
                            GENERICAPP_PROFID,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
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
                        GENERICAPP_PROFID,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        FALSE );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  unsigned char buffer[15];
  unsigned char WenDuChars[2];
  unsigned char ShiDuChars[2];
  unsigned char GuangZhaoChars[4];
  unsigned char SoundFlag[1];
  unsigned char Distance[3];
  unsigned char Posture[24];
  unsigned char GPS[50];
  unsigned char GAS[1];
  switch ( pkt->clusterId )
  {
    /*********************************************************************
    *Temperature and Humidity data:
    */
    case GENERICAPP_CLUSTERID_TEMHUM:
      // "the" message
      osal_memcpy(buffer,pkt->cmd.Data,15);
      if(buffer[0]!=0)
      {
        WenDuChars[0]=buffer[5];
        WenDuChars[1]=buffer[6];
        HalUARTWrite(1,"$u,01,00,",9);
        HalUARTWrite(1,WenDuChars,2);
        HalUARTWrite(1,",check,cr#",10);
        HalUARTWrite(1,"\n",1);
      }
      else
      {
        HalUARTWrite(1,"It's Wrong",10);
      }
      if(buffer[1]!=0)
      { 
        ShiDuChars[0]=buffer[9];
        ShiDuChars[1]=buffer[10];
        HalUARTWrite(1,"$u,02,00,",9);
        HalUARTWrite(1,ShiDuChars,2);
        HalUARTWrite(1,",check,cr#",10);
        HalUARTWrite(1,"\n",1);
      }
      else
      {
        HalUARTWrite(1,"It's Wrong",10);
        HalUARTWrite(1,"\n",1);
      }
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
    /*********************************************************************
    *Light data:
    */
    case GENERICAPP_CLUSTERID_LINGHT:
      // "the" message
    osal_memcpy(buffer,pkt->cmd.Data,15);
    if(buffer[0]!=0)
    {
      GuangZhaoChars[0]=buffer[6];

      GuangZhaoChars[1]=buffer[7];
      GuangZhaoChars[2]=buffer[8];

      GuangZhaoChars[3]=buffer[9];
      HalUARTWrite(1,"$u,11,00,",9);
      if (GuangZhaoChars[1]==' ')
        HalUARTWrite(1,GuangZhaoChars,1);
      else if (GuangZhaoChars[2]==' ')
        HalUARTWrite(1,GuangZhaoChars,2);
      else if (GuangZhaoChars[3]==' ')
        HalUARTWrite(1,GuangZhaoChars,3);
      else 
        HalUARTWrite(1,GuangZhaoChars,4);
      //HalUARTWrite(1,pkt->cmd.Data,15);
      HalUARTWrite(1,",check,cr#",10);
      HalUARTWrite(1,"\n",1);
    }
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
    /*********************************************************************
    *Sound data:
    */
    case GENERICAPP_CLUSTERID_SOUND:
      // "the" message
      osal_memcpy(buffer,pkt->cmd.Data,15);    
      if(buffer[0]!=0)
      {
        SoundFlag[0]=buffer[6];
        HalUARTWrite(1,"$u,10,00,",9);
        HalUARTWrite(1,SoundFlag,1);
        HalUARTWrite(1,",check,cr#",10);
        HalUARTWrite(1,"\n",1);
      }
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
    /*********************************************************************
    *Posture data:
    */
    case GENERICAPP_CLUSTERID_POSTURE:
      // "the" message
      osal_memcpy(Posture,pkt->cmd.Data,24);
      if(Posture[0]!=0)
      {
        //Posture[0]=buffer[4];
        //Posture[1]=buffer[7];
        //Posture[2]=buffer[10];
        HalUARTWrite(1,"$u,12,00,",9);
        HalUARTWrite(1,Posture,24);
        HalUARTWrite(1,",check,cr#",10);
        HalUARTWrite(1,"\n",1);
      }
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
    /*********************************************************************
    *Distance and Humidity data:
    */
    case GENERICAPP_CLUSTERID_DISTANCE:
      // "the" message
      osal_memcpy(buffer,pkt->cmd.Data,15);
      if(buffer[0]!=0)
      {
        Distance[0]=buffer[9];
        Distance[1]=buffer[10];
        Distance[2]=buffer[11];
        HalUARTWrite(1,"$u,09,00,",9);
        //HalUARTWrite(1,pkt->cmd.Data,15);
        if (Distance[1]==' ')
          HalUARTWrite(1,Distance,1);
        else if (Distance[2]==' ')
          HalUARTWrite(1,Distance,2);
        else 
          HalUARTWrite(1,Distance,3);
        HalUARTWrite(1,",check,cr#",10);
        HalUARTWrite(1,"\n",1);
      }
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
      case GENERICAPP_CLUSTERID_GAS:
      // "the" message
      osal_memcpy(buffer,pkt->cmd.Data,15);
      if(buffer[0]!=0)
      {
        GAS[0]=buffer[0];
        HalUARTWrite(1,"$u,13,00,",9);
        HalUARTWrite(1,GAS,1);
        HalUARTWrite(1,",check,cr#",10);
        HalUARTWrite(1,"\n",1);
      }
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
      case GENERICAPP_CLUSTERID_GPS:
      // "the" message
      osal_memcpy(GPS,pkt->cmd.Data,50);
      HalUARTWrite(1,"$u,04,00,",9);
      HalUARTWrite(1,pkt->cmd.Data,50);
      HalUARTWrite(1,",check,cr#",10);
      HalUARTWrite(1,"\n",1);
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
      case GENERICAPP_CLUSTERID_DOUBLE:
      // "the" message
      osal_memcpy(GPS,pkt->cmd.Data,15);
      HalUARTWrite(1,"$u,08,00,",9);
      HalUARTWrite(1,"1",1);
      HalUARTWrite(1,",check,cr#",10);
      HalUARTWrite(1,"\n",1);
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
  }
}

/*********************************************************************
 * @fn      GenericApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_SendTheMessage(unsigned char *theMessageData)
{
  afAddrType_t my_DstAddr;
  my_DstAddr.addrMode=(afAddrMode_t)Addr16Bit;
  my_DstAddr.endPoint=GENERICAPP_ENDPOINT;
  my_DstAddr.addr.shortAddr=0xFFFF; 
  
  AF_DataRequest(&my_DstAddr
                 ,&GenericApp_epDesc
                   ,GENERICAPP_CLUSTERID_DOUBLE
                       ,1
                       ,theMessageData
                         ,&GenericApp_TransID
                           ,AF_DISCV_ROUTE
                             ,AF_DEFAULT_RADIUS);
}
#if defined( IAR_ARMCM3_LM )
/*********************************************************************
 * @fn      GenericApp_ProcessRtosMessage
 *
 * @brief   Receive message from RTOS queue, send response back.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessRtosMessage( void )
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

/*********************************************************************
 */
