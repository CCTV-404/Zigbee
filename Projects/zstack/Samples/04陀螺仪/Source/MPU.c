/*******************************************************
文件名:main.c
版本号:v1.0
创建日期:2013-10-23
作者：楼燚航
主要函数描述：conversion 转换数据可供显示的ACII值
              BH1750_Start() BH1750_Stop()分别给出启动和停止信号
              BH1750_SendByte(BYTE dat)送一字节数据
              BH1750_RecvByte()接受一个字节数据
              Single_Write_BH1750(uchar REG_Address)写一个字节
              Multiple_Read()连续读出内部数据
              Init_BH1750()初始化BH1750
********************************************************/

#include "MPU.h"

BYTE    BUF[8];
uchar   ge, shi, bai, qian, wan;            //显示变量
int     dis_data;
//存放数据的变量
char HX;
char LX;
char HY;
char LY;
char HZ;
char LZ;
char TH;
char TL;
char GHX;
char GLX;
char GHY;
char GLY;
char GHZ;
char GLZ;
double ax,ay,az;                        //重力加速度在x、y、z方向的分量
double temp;
double jx,jy,jz;                        //加速度在x、y、z方向的分量
#define g 9.8















/**************************************
设置IIC数据口SDA为输入模式
**************************************/
void _SDA_IN(void)
{
  _SDA_DIR &= ~SDABIT;
}
/**************************************
设置IIC数据口SDA为输出模式
**************************************/
void _SDA_OUT(void)
{
  _SDA_DIR |= SDABIT;
}
/**************************************
设置IIC时钟口SCL为输入模式
**************************************/
void _SCL_IN(void)
{
  _SCL_DIR &=~ SCLBIT;
}
/**************************************
设置IIC时钟口SCL为输出模式
**************************************/
void _SCL_OUT(void)
{
  _SCL_DIR |= SCLBIT;
}
/**************************************
SDA写入1
**************************************/
void _SDA_1(void)
{
  _SDA_OUT();
  _SDA=1;
}
/**************************************
SDA写入0
**************************************/
void _SDA_0(void)
{
  _SDA_OUT();
  _SDA=0;
}
/**************************************
SCL写入1
**************************************/
void _SCL_1(void)
{
  _SCL_OUT();
  _SCL=1;
}
/**************************************
SCL写入0
**************************************/
void _SCL_0()
{
  _SCL_OUT();
  _SCL=0;
}
/**************************************
延时24微秒
不同的工作环境,需要调整此函数，注意时钟过快时需要修改
当改用1T的MCU时,请调整此延时函数
**************************************/
void NOP1() 
{
  asm("NOP");asm("NOP");asm("NOP");
   asm("NOP");asm("NOP");asm("NOP");
     asm("NOP");asm("NOP");asm("NOP");
       asm("NOP");asm("NOP");asm("NOP");
         asm("NOP");asm("NOP");asm("NOP");
           asm("NOP");asm("NOP");asm("NOP");
             asm("NOP");asm("NOP");asm("NOP");
               asm("NOP");asm("NOP");asm("NOP");
}
/**************************************
延时函数
**************************************/
void NOP(void)
{
  NOP1();NOP1();NOP1();NOP1();NOP1();NOP1();NOP1();NOP1();NOP1();
}
 
 
//IIC开始工作模式，设置方式为SCL为高电平时，SDA发生高到低跳变
void _Start(void)
 {
     _SDA_1();
     _SCL_1();
      NOP();
     _SDA_0();
      NOP();
     _SCL_0();
      NOP();
 }
 //IIC停止工作模式，设置方式为SCL为高电平时，SDA发生低到高跳变
void _Stop(void)
 {
     _SDA_0();
     _SCL_1();
      NOP();
     _SDA_1();
      NOP();
     _SCL_0();
      NOP();
 }
//初始化SDA，SCL口数据
 void _Port_Init(void)
 {
     _SCL_0();
     _SDA_1();
 }
//等待响应函数
 uint _Wait_ACK(void)
 {
     uint bc=0;
     uint16 i=1000;
     _SDA_IN();
     _SCL_1();
     NOP();
     do
     {
      if(_SDA==0)
       break;
     }while(i--);
     bc=_SDA;
     _SCL_0();
     _SDA_OUT();
     return bc;
 }
//向SDA数据口写入一个字节
 uint _Write(uint data)
 {
    uint i;
    _SDA_OUT();
    _SCL_0();
    for(i=0;i<8;i++)
    {
      if(data&0x80)
      {
        _SDA_1();
      }
      else
      {
        _SDA_0();
      }
      data<<=1;
      _SCL_1();
      NOP();
      _SCL_0();
      NOP();
    }
    _SDA_1();//释放总线
    if(_Wait_ACK()==0)//ACK
    {
      return 1;
    }
    else
    {
      return 0;
    }
 }
//向SDA数据口读出一个字节
 uint _Read()
 {
    uint data=0;
    uint i;
    _SDA_IN();
    _SCL_0();
    NOP();
    for(i=0;i<8;i++)
    {
      _SCL=1;
      NOP();
      data<<=1;
      if(_SDA)
      {
        data|=1;
      }
      _SCL=0;
      NOP();
    }
    _SDA_OUT();
    _SCL_0();
    NOP();
    return data;
 }
 //向地址addr写入一字节wd
uint _Write_Data(uint addr,uint wd)
 {
       _Start();
       if(_Write(MPU6050_SA_W))//Slave address+R/W    0xD0+0/1
       {
           if(_Write(addr))
           {
               if(_Write(wd))
               {
                 _Stop();
                 NOP();
                 return wd;
               }
               return 0xf3;
           }
           return 0xf2;
       }
       else
         return 0xf1;
 
}
//读取地址addr数据
 uint _Read_Data(uint addr)//数据读出
{
        uint rd;
       _Start();
       if(_Write(MPU6050_SA_W))
       {
         if(_Write(addr))
         {
           _Start();
           if(_Write(MPU6050_SA_R))
           {
             rd=_Read();
             if(_Wait_ACK())///NoACk
             _Stop();
             NOP();
             return rd;
           }
           return 0xf3;
         }
         return 0xf2;
       }
       return 0xf1;
 }
//MPU6050初始化
 void  _MPU_Init(void)
 {
    _Port_Init();
    _Write_Data(PWR_MGMT_1, 0x00);        //解除休眠状态
    _Write_Data(SMPLRT_DIV, 0x07);
    _Write_Data(CONFIG, 0x06);
    _Write_Data(GYRO_CONFIG, 0x18);
    _Write_Data(ACCEL_CONFIG, 0x01);
 }

void read()
{
    HX=_Read_Data(ACCEL_XOUT_H);
    LX=_Read_Data(ACCEL_XOUT_L);
    ax=((HX<<8)|LX)/32768.0*19.5;
    HY=_Read_Data(ACCEL_YOUT_H);
    LY=_Read_Data(ACCEL_YOUT_L);
    ay=((HY<<8)|LY)/32768.0*19.5;
    HZ=_Read_Data(ACCEL_ZOUT_H);
    LZ=_Read_Data(ACCEL_ZOUT_L);
    az=((HZ<<8)|LZ)/32768.0*19.5;
    TH=_Read_Data(TEMP_OUT_H);
    TL=_Read_Data(TEMP_OUT_L);
    temp=((TH<<8)|TL)/340.0+36.53;
    GHX=_Read_Data(GYRO_XOUT_H);
    GLX=_Read_Data(GYRO_XOUT_L);
    jx=((GHX<<8)|GLX)/32768.0*2000;
    GHY=_Read_Data(GYRO_YOUT_H);
    GLY=_Read_Data(GYRO_YOUT_L);
    jy=((GHY<<8)|GLY)/32768.0*2000;
    GHZ=_Read_Data(GYRO_ZOUT_H);
    GLZ=_Read_Data(GYRO_ZOUT_L);
    jz=((GHZ<<8)|GLZ)/32768.0*2000;
}

