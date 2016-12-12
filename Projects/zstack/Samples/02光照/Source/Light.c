#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include <string.h>
//#include "Common.h"
#include "DebugTrace.h"
#include "Light.h"
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

#include "BH1750.h"
#define SEND_DATA_EVENT 0x01
float fLight = 123;
const cId_t Light_ClusterList[Light_MAX_CLUSTERS] =
{
  Light_CLUSTERID
};



const SimpleDescriptionFormat_t Light_SimpleDesc =
{
  Light_ENDPOINT,              //  int Endpoint;
  Light_PROFID,                //  uint16 AppProfId[2];
  Light_DEVICEID,              //  uint16 AppDeviceId[2];
  Light_DEVICE_VERSION,        //  int   AppDevVer:4;
  Light_FLAGS,                 //  int   AppFlags:4;
  
  
  0,          //  byte  AppNumInClusters;
  (cId_t *)NULL,  //  byte *pAppInClusterList;
  Light_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)Light_ClusterList   //  byte *pAppInClusterList;
};

unsigned char TempDATA;
endPointDesc_t Light_epDesc;
byte Light_TaskID;
byte Light_TransID;
devStates_t Light_NwkState;
void Light_MessageMSGCB(afIncomingMSGPacket_t *MSGpkt);
void Light_SendTheMessage(void);

static void rxCB(uint8 port,uint8 event);
void TempHumApp_SerialSend(unsigned char* ser_str);

void Light_Init( byte task_id )
{
  //halUARTCfg_t uartConfig;//串口
    
  Light_TaskID = task_id;
  Light_NwkState=DEV_INIT;
  Light_TransID = 0;

  
  Light_epDesc.endPoint = Light_ENDPOINT;
  Light_epDesc.task_id = &Light_TaskID;
  Light_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&Light_SimpleDesc;
  
  Light_epDesc.latencyReq = noLatencyReqs;
  afRegister( &Light_epDesc ); 
  
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
UINT16 Light_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Light_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
       
          case ZDO_STATE_CHANGE:
            Light_NwkState = (devStates_t)(MSGpkt->hdr.status);
            if(Light_NwkState==DEV_END_DEVICE)
            {
              P1_0=~P1_0;
              osal_set_event(Light_TaskID,SEND_DATA_EVENT);
            }
            break;
            
          default:
            break;
      }
      osal_msg_deallocate( (uint8 *)MSGpkt );
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Light_TaskID );
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  if(events&SEND_DATA_EVENT)
  {
    Init_BH1750();       //初始化BH1750
    Single_Write_BH1750(0x02);   // power on
    Single_Write_BH1750(0x10);   // H- resolution mode

    delay_nms(180);              //延时180ms
    Multiple_Read();       //连续读出数据，存储在BUF中

    dis_data=BUF[0];
    dis_data=(dis_data<<8)+BUF[1];//合成数据，即光照数据

    fLight = (float)dis_data/1.5;
    Light_SendTheMessage();
    osal_start_timerEx(Light_TaskID,SEND_DATA_EVENT,3000);
    return(events^SEND_DATA_EVENT);
  }
  return 0;
}

void Light_SendTheMessage(void)
{ 
  unsigned char theMessageData[10]="EndDevice";

  afAddrType_t my_DstAddr;

  my_DstAddr.addrMode=(afAddrMode_t)Addr16Bit;
  my_DstAddr.endPoint=Light_ENDPOINT;
  my_DstAddr.addr.shortAddr=0x0000; 

  fLight = 123;
  theMessageData[0]='0'+(int)fLight/100;
  theMessageData[1]='0'+((int)fLight-(int)fLight/100*100)/10;
  theMessageData[2]='0'+(int)fLight%10;

  AF_DataRequest(&my_DstAddr
  ,&Light_epDesc
  ,Light_CLUSTERID
  ,osal_strlen("EndDevice")+1
  ,theMessageData
  ,&Light_TransID
  ,AF_DISCV_ROUTE
  ,AF_DEFAULT_RADIUS);
}
void TempHumApp_SerialSend(unsigned char* ser_str)
{
  afAddrType_t my_DstAddr;
  unsigned char theMessageData[10]="EndDevice";
  
 my_DstAddr.addrMode=(afAddrMode_t)Addr16Bit;
  my_DstAddr.endPoint=Light_ENDPOINT;
  my_DstAddr.addr.shortAddr=0x0000; 

  memcpy(theMessageData,ser_str,10);
  

  AF_DataRequest(&my_DstAddr
  ,&Light_epDesc
  ,Light_CLUSTERID
  ,osal_strlen("EndDevice")+1
  ,theMessageData
  ,&Light_TransID
  ,AF_DISCV_ROUTE
  ,AF_DEFAULT_RADIUS);
}


