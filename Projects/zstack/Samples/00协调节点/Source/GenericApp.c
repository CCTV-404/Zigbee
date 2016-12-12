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

#include<string.h>
#include "MT_UART.h"
//void MT_UartAppFlowControl ( bool status );
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
#define ADC_REF_AVDD5 0x80
#define ADC_REF_125_V 0x00
#define ADC_14_BIT 0x30
#define ADC_AIN1_SENS 0x07
#define ADC_TEMP_SENS 0x0E
 devStates_t GenericApp_NwkState;
char temp1='a';
char temp2='a';
char temp3 = 'a';
char temp4 = 'a';
char temp5 = 'a';
int flag1 = 0;
int flag2 = 0;
// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID_TEMHUM,
  GENERICAPP_CLUSTERID_LINGHT,
  GENERICAPP_CLUSTERID_SOUND,
  GENERICAPP_CLUSTERID_POSTURE,
  GENERICAPP_CLUSTERID_DISTANCE,
  GENERICAPP_CLUSTERID_GPS,
  GENERICAPP_CLUSTERID_GAS
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
  0,          //  byte  AppNumInClusters;
  (cId_t *)NULL   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;
byte GenericApp_TaskID;
byte GenericApp_TransID;
void GenericApp_MessageMSGCB(afIncomingMSGPacket_t *pkt);
void GenericApp_SendTheMessage(unsigned char *theMessageData);

//uint8 * GetShortAddr(void);
uint8 hextoword1(uint8 t);
uint8 hextoword2(uint8 t);
void GenericApp_SPI(void);
void GenericApp_UART1(afIncomingMSGPacket_t *pkt);
static void rxCB(uint8 port,uint8 event);

#if defined( IAR_ARMCM3_LM )
//static void GenericApp_ProcessRtosMessage( void );
#endif

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
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
void initUARTSEND(void)
{

   /* CLKCONCMD &= ~0x40;                          //设置系统时钟源为32MHZ晶振
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
*/

    //SLEEPCMD&= ~0X04;
   // CLKCONCMD = 0X10;                           //选择32KHz晶体振荡器,32MHz晶体振荡器,定时器tick输出8MHz
   // while(CLKCONSTA!=0X10);                    //晶振是否稳定
   // SLEEPCMD = 0X04;    

    CLKCONCMD &= ~0x40;                          //设置系统时钟源为32MHZ晶振
    while(CLKCONSTA & 0x40);                     //等待晶振稳定
    CLKCONCMD &= ~0x47; 
    
    PERCFG = 0x00;				//位置1 P0口
    P0SEL = 0x3c;				//P0用作串口
    //P2DIR &= ~0XC0;                             //P0优先作为串口0

    U0CSR |= 0x80;				//UART方式
    U0GCR |= 8;				        //baud_e
    U0BAUD |= 59;				//波特率设为9600
    UTX0IF = 1;                                 //UART1 TX中断标志初始置位0 制1位开中断

    U0CSR |= 0X40;				//允许接收
    IEN0 |= 0x84;				//开总中断，接收中断
}


/*static void rxCB(uint8 port,uint8 event)
{
unsigned  char LED[4];
unsigned char len;

  len = HalUARTRead(1, LED, 2);
  if(!len)
    return;
 
  if((LED[0] - '1') == 0x00)
  {
    P1_0=~P1_0;
    GenericApp_SendTheMessage(LED);
  }   
  if((LED[0] - '2') == 0x00)
  {
    P1_0=~P1_0;
    GenericApp_SendTheMessage(LED);
  }    
}*/
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
void GenericApp_Init( byte task_id )
{
  halUARTCfg_t uartConfig;
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;   
  //MT_UartInit();
  //initUARTSEND();
  
  //HalUARTWrite(1,"Hello World\n",12);

  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;
  
  
  afRegister( &GenericApp_epDesc );
  RegisterForKeys( GenericApp_TaskID );
  MT_UartRegisterTaskID(task_id);
  
  uartConfig.configured = TRUE;
  uartConfig.baudRate = HAL_UART_BR_9600;
  uartConfig.flowControl = FALSE;
  uartConfig.callBackFunc = rxCB;
  HalUARTOpen(0,&uartConfig);
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
UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case AF_INCOMING_MSG_CMD:
          GenericApp_MessageMSGCB(MSGpkt);
          break;
        
        case ZDO_STATE_CHANGE:
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (GenericApp_NwkState == DEV_ZB_COORD)
              || (GenericApp_NwkState == DEV_ROUTER)
              || (GenericApp_NwkState == DEV_END_DEVICE) )
          {
            GenericApp_SPI();
            // Start sending "the" message in a regular interval.
            osal_start_timerEx( GenericApp_TaskID,
                                GENERICAPP_SEND_MSG_EVT,
                                1000 );
          }
          break;
          
        /*case SPI_INCOMING_ZAPP_DATA:
          GenericApp_SPI();
          GenericApp_UART1(MSGpkt);
          break;*/
          default:
          break;
      }
      osal_msg_deallocate( (uint8 *)MSGpkt );
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
 /* if ( events & GENERICAPP_SEND_MSG_EVT )
  {
    // Send "the" message
    //GenericApp_SendTheMessage();
    P0DIR &= ~0x80;
    P0SEL |= 0x80;
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
                        1000 );

    // return unprocessed events
    return (events ^ GENERICAPP_SEND_MSG_EVT);
  }*/
  return 0;  
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
void GenericApp_MessageMSGCB(afIncomingMSGPacket_t *pkt)
{
  //unsigned char GPS[33];
  unsigned char buffer[24];
  unsigned char WenDuChars[2];
  unsigned char ShiDuChars[2]; 
  switch(pkt->clusterId)
  {
   /* case GENERICAPP_CLUSTERID_POSTURE:

    osal_memcpy(buffer,pkt->cmd.Data,24);
    if(buffer[0]!=0)
      {
        P1_1=~P1_1;
        P1_0=~P1_0;
        HalUARTWrite(1,"$u,12,00,",9);
        HalUARTWrite(1,buffer,24);
        HalUARTWrite(1,",check,cr#",10);
        HalUARTWrite(1,"\n",1);
      }
    break;*/
    case GENERICAPP_CLUSTERID_LINGHT:
    P1_0=~P1_0;
    P1_1=~P1_1;

    P1_4=~P1_4;
    
    osal_memcpy(buffer,pkt->cmd.Data,3);
    //char Txdata[25];
   // strcpy(Txdata,"HELLO");
    if(buffer[0]!=0)
      {
        temp1 = buffer[0];
        temp2 = buffer[1];
        temp3 = buffer[2];
        flag1++;
        //P1_1=~P1_1;
        //P1_0=~P1_0;
        HalUARTWrite(0,"Light",5);
        HalUARTWrite(0,buffer,3);
        HalUARTWrite(0,"\n",1);
       /*HalUARTWrite(0,"guang",5);
        HalUARTWrite(0,buffer,3);
       HalUARTWrite(0,"\n",1);*/
      }
    break;
   /* case GENERICAPP_CLUSTERID_SOUND:
    //P1_1=~P1_1;
    
    osal_memcpy(buffer,pkt->cmd.Data,1);
    if(buffer[0]=='0' || buffer[0]=='1')
      {
        P1_1=~P1_1;
        P1_0=~P1_0;
        HalUARTWrite(1,"$u,10,00,",9);
        HalUARTWrite(1,buffer,1);
        HalUARTWrite(1,",check,cr#",10);
        HalUARTWrite(1,"\n",1);
      }
    break;
    
    case GENERICAPP_CLUSTERID_DISTANCE:
    P1_1=~P1_1;
    
    osal_memcpy(buffer,pkt->cmd.Data,3);
    if(buffer[0]!=0)
      {
        P1_0=~P1_0;
        HalUARTWrite(1,"$u,09,00,",9);
        HalUARTWrite(1,buffer,3);
        HalUARTWrite(1,",check,cr#",10);
        HalUARTWrite(1,"\n",1);
      }
    break;
    */
   
    case GENERICAPP_CLUSTERID_TEMHUM:
    ////P1_0=~P1_0;
   // P1_1=~P1_1;

   // P1_4=~P1_4;
    osal_memcpy(buffer,pkt->cmd.Data,2);
    
    
    if(buffer[0]!=0)
      {
        flag2++;
        WenDuChars[0]=buffer[0]/10+'0';
        WenDuChars[1]=buffer[0]%10+'0';
        temp4 = WenDuChars[0];
        HalUARTWrite(0,"tem",3);
        HalUARTWrite(0,buffer,1);
        /*HalUARTWrite(1,"$u,01,00,",9);
        HalUARTWrite(1,WenDuChars,2);
        HalUARTWrite(1,",check,cr#",10);
        HalUARTWrite(1,"\n",1);*/
      }
      else
      {
        //HalUARTWrite(1,"It's Wrong",10);
      }
      if(buffer[1]!=0)
      { 
        ShiDuChars[0]=buffer[1]/10+'0';
        ShiDuChars[1]=buffer[1]%10+'0';
        temp5 =  ShiDuChars[0];
        HalUARTWrite(0,"hum",3);
         HalUARTWrite(0,buffer,1);
          HalUARTWrite(0,"\n",1);
       /* HalUARTWrite(1,"$u,02,00,",9);
        HalUARTWrite(1,ShiDuChars,2);
        HalUARTWrite(1,",check,cr#",10);
        HalUARTWrite(1,"\n",1);*/
      }
      else
      {
        //HalUARTWrite(1,"It's Wrong",10);
        HalUARTWrite(1,"\n",1);
      } 
    break;
   /* case GENERICAPP_CLUSTERID_GAS:
    //P1_1=~P1_1;
    
    osal_memcpy(buffer,pkt->cmd.Data,1);
    if(buffer[0]!=0)
      {
        P1_1=~P1_1;
        P1_0=~P1_0;
        HalUARTWrite(1,"$u,13,00,",9);
        HalUARTWrite(1,buffer,1);
        HalUARTWrite(1,",check,cr#",10);
        HalUARTWrite(1,"\n",1);
      }
    break;
    case GENERICAPP_CLUSTERID_DOUBLE:

    break;
    case GENERICAPP_CLUSTERID_GPS:
      osal_memcpy(GPS,pkt->cmd.Data,33);
      if(buffer[0]!=0)
      {
        HalUARTWrite(1,"$u,04,00,",9);
        HalUARTWrite(1,pkt->cmd.Data,33);
        HalUARTWrite(1,",check,cr#",10);
        HalUARTWrite(1,"\n",1);
      }
    break;*/
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
void GenericApp_SendTheMessage(unsigned char *theMessageData)
{
  afAddrType_t my_DstAddr;
  my_DstAddr.addrMode=(afAddrMode_t)Addr16Bit;
  my_DstAddr.endPoint=GENERICAPP_ENDPOINT;
  my_DstAddr.addr.shortAddr=0xFFFF; 
  
  AF_DataRequest(&my_DstAddr
                 ,&GenericApp_epDesc
                   ,GENERICAPP_CLUSTERID_DOUBLE
                       ,4
                       ,theMessageData
                         ,&GenericApp_TransID
                           ,AF_DISCV_ROUTE
                             ,AF_DEFAULT_RADIUS);
}

/*********************************************************************
 */
uint8 shortddr[8];
void GenericApp_SPI(void)
{       
  uint8 str_1[]="my short address is";
  //HalUARTWrite(0,&str_1[0],20);
  
  uint16 short_ddr;
  uint8 short_ddr_H;
  uint8 short_ddr_L;
  uint8 shortaddr[7];
  uint8* pointerl;
  
  short_ddr=NLME_GetShortAddr();
  short_ddr_H=(uint8)((short_ddr&0xff00)>>8);
  short_ddr_H=(uint8)short_ddr;
  
  shortddr[0]=48;
  shortddr[1]=120;
  shortddr[2]=hextoword1(short_ddr_H);
  shortddr[3]=hextoword2(short_ddr_H);
  shortddr[4]=hextoword1(short_ddr_L);
  shortddr[5]=hextoword2(short_ddr_L);
  shortddr[6]='\n';
  
  pointerl=&shortaddr[0];
  HalUARTWrite(0,"UART SUCCESS",12);
}
/*uint8 * GetShortAddr(void)
{
  uint8 * p;
  shor_dr=NLME_GetShortAddr();
  
  short_dr_H=(uint8)((short_dr&0xff00)>>8);
  short_dr_L=(uint8)short_dr;
  
  shortadr[0]=48;
  shortadr[1]=120;
  shortadr[2]=hextoword1(short_dr_H);
  shortadr[3]=hextoword2(short_dr_H);
  shortadr[4]=hextoword1(short_dr_L);
  shortadr[5]=hextoword2(short_dr_L);
  shortadr[6]='\n';
  p=&shortaddr[0];
  return(p);
}*/
uint8 hextoword1(uint8 t)
{
  uint8 abc;
  uint8 cba;
  uint8 xx1;
  abc = t;
  cba = 0xf0;
  abc=(abc&cba)>>4;
  if(abc<10)
  {
    xx1 = abc+48;
  }
  else
  {
    xx1=abc+55;
  }
  return xx1;
}
uint8 hextoword2(uint8 t)
{
  uint8 abc;
  uint8 cba;
  uint8 xx2;
  abc = t;
  cba = 0x0f;
  abc=(abc&cba)>>4;
  if(abc<10)
  {
    xx2 = abc+48;
  }
  else
  {
    xx2=abc+55;
  }
  return xx2;
}
void GenericApp_UART1(afIncomingMSGPacket_t *pkt)
{
  HalUARTWrite(0,(uint8 *)((osal_event_hdr_t *)pkt+1),pkt->hdr.status);
  MT_UartAppFlowControl(true);
}
static void rxCB(uint8 port,uint8 event)
{  
  //HalUARTWrite(0,"I get",5);
  unsigned  char Uartbuf[10];
  unsigned char len;
  len=HalUARTRead(0,Uartbuf,10);
  if(len)
  {
    HalUARTWrite(0,"I get",5);
    HalUARTWrite(0,Uartbuf,len);
    len=0;
  }
}