//DHT11温度湿度采集基础实验

#include <ioCC2530.h>
#include "OnBoard.h"
#define uint unsigned int
#define U8 unsigned char


#define DATA_PIN P1_3 

/*******函数声明*********/
void Delay_us(void);      //1us延时
void Delay_10us(void);    //10us延时
void Delay_ms(uint Time); //nms延时
void COMM(void);          //温湿写入
void Read_DHT11(void) ;   //温湿传感启动


//温湿度定义
U8 U8FLAG,U8temp;
U8 Hum_H,Hum_L;           //定义湿度存放变量
U8 Temp,Hum;              //定义温度存放变量
U8 U8T_data_H,U8T_data_L,U8RH_data_H,U8RH_data_L,U8checkdata;
U8 U8T_data_H_temp,U8T_data_L_temp,U8RH_data_H_temp,U8RH_data_L_temp,U8checkdata_temp;
U8 U8comdata;


/****************************
//延时函数
*****************************/
void Delay_us(void)
{
    MicroWait(1);   
}

void Delay_10us(void)
{
   MicroWait(10);
}

void Delay_ms(uint Time)
{
  unsigned char i;
  while(Time--)
  {
    for(i = 0;i<100;i++)
     Delay_10us();
  }
}


/***********************
   温湿度传感
***********************/
void COMM(void)	// 温湿写入
{     
    U8 i;         
    for(i = 0;i<8;i++)    
    {
     U8FLAG = 2; 
     DATA_PIN = 0;
     DATA_PIN = 1;
     while((!DATA_PIN)&&U8FLAG++);
     Delay_10us();
     Delay_10us();
     Delay_10us();
     U8temp = 0;
     if(DATA_PIN)U8temp = 1;
     U8FLAG = 2;
     while((DATA_PIN)&&U8FLAG++);   
     if(U8FLAG == 1)break;    
     U8comdata <<= 1;
     U8comdata |= U8temp; 
     }    
}

void Read_DHT11(void)   //温湿传感启动
{
    DATA_PIN = 0;
    Delay_ms(19);  //主机拉低18ms
    DATA_PIN = 1;     //总线由上拉电阻拉高 主机延时40us 
    P1DIR &=  ~0x08; //重新配置IO口方向
    Delay_10us();
    Delay_10us();						
    Delay_10us();
    Delay_10us();  
    //判断从机是否有低电平响应信号 如不响应则跳出，响应则向下运行 
     if(!DATA_PIN) 
     {
      U8FLAG = 2; //判断从机是否发出 80us 的低电平响应信号是否结束 
      while((!DATA_PIN)&&U8FLAG++);
      U8FLAG = 2;//判断从机是否发出 80us 的高电平，如发出则进入数据接收状态
      while((DATA_PIN)&&U8FLAG++); 
      COMM();//数据接收状态 
      U8RH_data_H_temp = U8comdata;
      COMM();
      U8RH_data_L_temp = U8comdata;
      COMM();
      U8T_data_H_temp = U8comdata;
      COMM();
      U8T_data_L_temp = U8comdata;
      COMM();
      U8checkdata_temp = U8comdata;
      DATA_PIN = 1; 
      //数据校验 
      U8temp = (U8T_data_H_temp+U8T_data_L_temp+U8RH_data_H_temp+U8RH_data_L_temp);
       if(U8temp == U8checkdata_temp)
      {
          U8RH_data_H = U8RH_data_H_temp;
          U8RH_data_L = U8RH_data_L_temp;
          U8T_data_H = U8T_data_H_temp;
          U8T_data_L = U8T_data_L_temp;
          U8checkdata = U8checkdata_temp;
       }
       Temp = U8T_data_H;
       Hum = U8RH_data_H;
    } 
    else 
    {  
      Temp = 0;
      Hum = 0;
    } 
    P1DIR |=  0x08; 
}