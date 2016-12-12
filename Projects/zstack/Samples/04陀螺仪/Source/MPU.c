/*******************************************************
�ļ���:main.c
�汾��:v1.0
��������:2013-10-23
���ߣ�¥�D��
��Ҫ����������conversion ת�����ݿɹ���ʾ��ACIIֵ
              BH1750_Start() BH1750_Stop()�ֱ����������ֹͣ�ź�
              BH1750_SendByte(BYTE dat)��һ�ֽ�����
              BH1750_RecvByte()����һ���ֽ�����
              Single_Write_BH1750(uchar REG_Address)дһ���ֽ�
              Multiple_Read()���������ڲ�����
              Init_BH1750()��ʼ��BH1750
********************************************************/

#include "MPU.h"

BYTE    BUF[8];
uchar   ge, shi, bai, qian, wan;            //��ʾ����
int     dis_data;
//������ݵı���
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
double ax,ay,az;                        //�������ٶ���x��y��z����ķ���
double temp;
double jx,jy,jz;                        //���ٶ���x��y��z����ķ���
#define g 9.8















/**************************************
����IIC���ݿ�SDAΪ����ģʽ
**************************************/
void _SDA_IN(void)
{
  _SDA_DIR &= ~SDABIT;
}
/**************************************
����IIC���ݿ�SDAΪ���ģʽ
**************************************/
void _SDA_OUT(void)
{
  _SDA_DIR |= SDABIT;
}
/**************************************
����IICʱ�ӿ�SCLΪ����ģʽ
**************************************/
void _SCL_IN(void)
{
  _SCL_DIR &=~ SCLBIT;
}
/**************************************
����IICʱ�ӿ�SCLΪ���ģʽ
**************************************/
void _SCL_OUT(void)
{
  _SCL_DIR |= SCLBIT;
}
/**************************************
SDAд��1
**************************************/
void _SDA_1(void)
{
  _SDA_OUT();
  _SDA=1;
}
/**************************************
SDAд��0
**************************************/
void _SDA_0(void)
{
  _SDA_OUT();
  _SDA=0;
}
/**************************************
SCLд��1
**************************************/
void _SCL_1(void)
{
  _SCL_OUT();
  _SCL=1;
}
/**************************************
SCLд��0
**************************************/
void _SCL_0()
{
  _SCL_OUT();
  _SCL=0;
}
/**************************************
��ʱ24΢��
��ͬ�Ĺ�������,��Ҫ�����˺�����ע��ʱ�ӹ���ʱ��Ҫ�޸�
������1T��MCUʱ,���������ʱ����
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
��ʱ����
**************************************/
void NOP(void)
{
  NOP1();NOP1();NOP1();NOP1();NOP1();NOP1();NOP1();NOP1();NOP1();
}
 
 
//IIC��ʼ����ģʽ�����÷�ʽΪSCLΪ�ߵ�ƽʱ��SDA�����ߵ�������
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
 //IICֹͣ����ģʽ�����÷�ʽΪSCLΪ�ߵ�ƽʱ��SDA�����͵�������
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
//��ʼ��SDA��SCL������
 void _Port_Init(void)
 {
     _SCL_0();
     _SDA_1();
 }
//�ȴ���Ӧ����
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
//��SDA���ݿ�д��һ���ֽ�
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
    _SDA_1();//�ͷ�����
    if(_Wait_ACK()==0)//ACK
    {
      return 1;
    }
    else
    {
      return 0;
    }
 }
//��SDA���ݿڶ���һ���ֽ�
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
 //���ַaddrд��һ�ֽ�wd
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
//��ȡ��ַaddr����
 uint _Read_Data(uint addr)//���ݶ���
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
//MPU6050��ʼ��
 void  _MPU_Init(void)
 {
    _Port_Init();
    _Write_Data(PWR_MGMT_1, 0x00);        //�������״̬
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

