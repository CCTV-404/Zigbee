#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include <string.h>
//#include "Common.h"
#include "DebugTrace.h"
#include "TempHumApp.h"
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

const cId_t TempHumApp_ClusterList[TempHumApp_MAX_CLUSTERS] =
{
  TempHumApp_CLUSTERID
};



const SimpleDescriptionFormat_t TempHumApp_SimpleDesc =
{
  TempHumApp_ENDPOINT,              //  int Endpoint;
  TempHumApp_PROFID,                //  uint16 AppProfId[2];
  TempHumApp_DEVICEID,              //  uint16 AppDeviceId[2];
  TempHumApp_DEVICE_VERSION,        //  int   AppDevVer:4;
  TempHumApp_FLAGS,                 //  int   AppFlags:4;
  
  
  0,          //  byte  AppNumInClusters;
  (cId_t *)NULL,  //  byte *pAppInClusterList;
  TempHumApp_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)TempHumApp_ClusterList   //  byte *pAppInClusterList;
};

unsigned char TempDATA;
endPointDesc_t TempHumApp_epDesc;
byte TempHumApp_TaskID;
byte TempHumApp_TransID;
devStates_t TempHumApp_NwkState;
void TempHumApp_MessageMSGCB(afIncomingMSGPacket_t *MSGpkt);
void TempHumApp_SendTheMessage(void);

static void rxCB(uint8 port,uint8 event);
void TempHumApp_SerialSend(unsigned char* ser_str);

void TempHumApp_Init( byte task_id )
{
  //halUARTCfg_t uartConfig;//串口
    
  TempHumApp_TaskID = task_id;
  TempHumApp_NwkState=DEV_INIT;
  TempHumApp_TransID = 0;

  
  TempHumApp_epDesc.endPoint = TempHumApp_ENDPOINT;
  TempHumApp_epDesc.task_id = &TempHumApp_TaskID;
  TempHumApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&TempHumApp_SimpleDesc;
  
  TempHumApp_epDesc.latencyReq = noLatencyReqs;
  afRegister( &TempHumApp_epDesc ); 
   //uart注册
  MT_UartRegisterTaskID(task_id);
  //串口初始化
   MT_UartInit();
  halUARTCfg_t uartConfig;
  uartConfig.configured = TRUE;
  uartConfig.baudRate = HAL_UART_BR_9600;
  uartConfig.flowControl = FALSE;
  uartConfig.callBackFunc = rxCB;
  HalUARTOpen(0,&uartConfig);
}
static void rxCB(uint8 port,uint8 event)
{
  //HalUARTWrite(0,"I get",5);
  unsigned  char Uartbuf[10];
  unsigned char len;
  len=HalUARTRead(0,Uartbuf,10);
  //HalUARTWrite(0,""+len,1);
  if(len)
  {
    //HalUARTWrite(0,"I get",5);
    //HalUARTWrite(0,Uartbuf,len);
    TempHumApp_SerialSend(Uartbuf);
    len=0;
  }

}
UINT16 TempHumApp_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( TempHumApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
       
          case ZDO_STATE_CHANGE:
            TempHumApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
            if(TempHumApp_NwkState==DEV_END_DEVICE)
            {
              P1_0=~P1_0;
              osal_set_event(TempHumApp_TaskID,SEND_DATA_EVENT);
            }
            break;
            
          default:
            break;
      }
      osal_msg_deallocate( (uint8 *)MSGpkt );
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( TempHumApp_TaskID );
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  if(events&SEND_DATA_EVENT)
  {
    Read_DHT11();
    TempHumApp_SendTheMessage();
    osal_start_timerEx(TempHumApp_TaskID,SEND_DATA_EVENT,3000);
    return(events^SEND_DATA_EVENT);
  }
  return 0;
}

void TempHumApp_SendTheMessage(void)
{ 
unsigned char theMessageData[10]="EndDevice";

afAddrType_t my_DstAddr;

my_DstAddr.addrMode=(afAddrMode_t)Addr16Bit;
my_DstAddr.endPoint=TempHumApp_ENDPOINT;
my_DstAddr.addr.shortAddr=0x0000; 


theMessageData[0]=66;
theMessageData[1]=88;

AF_DataRequest(&my_DstAddr
,&TempHumApp_epDesc
,TempHumApp_CLUSTERID
,osal_strlen("EndDevice")+1
,theMessageData
,&TempHumApp_TransID
,AF_DISCV_ROUTE
,AF_DEFAULT_RADIUS);
}
/*HAL_ISR_FUNCTION( halKeyPort0Isr, P0INT_VECTOR )
{
  
  if (HAL_KEY_SW_7_PXIFG & HAL_KEY_SW_7_BIT)
  {
  }
  //HAL_KEY_SW_7_PXIFG = 0;
  //HAL_KEY_CPU_PORT_0_IF = 0;
}*/

void TempHumApp_SerialSend(unsigned char* ser_str)
{
  afAddrType_t my_DstAddr;
  unsigned char theMessageData[10]="EndDevice";
  
  my_DstAddr.addrMode=(afAddrMode_t)Addr16Bit;
  my_DstAddr.endPoint=TempHumApp_ENDPOINT;
  my_DstAddr.addr.shortAddr=0x0000; 

  memcpy(theMessageData,ser_str,10);
  

  AF_DataRequest(&my_DstAddr
    ,&TempHumApp_epDesc
    ,TempHumApp_CLUSTERID
    ,osal_strlen("EndDevice")+1
    ,theMessageData
    ,&TempHumApp_TransID
    ,AF_DISCV_ROUTE
    ,AF_DEFAULT_RADIUS);
}
