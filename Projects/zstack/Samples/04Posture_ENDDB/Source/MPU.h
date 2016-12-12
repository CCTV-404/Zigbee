#ifndef  __BH1750_H__
#define  __BH1750_H__

#include<ioCC2530.h>
#include<math.h>

#define BIT0              0x01
#define BIT1              0x02
#define BIT2              0x04
#define BIT3              0x08
#define BIT4              0x10
#define BIT5              0x20
#define BIT6              0x40
#define BIT7              0x80

#define   SCL     P1_7      //IICʱ�����Ŷ���
#define   SDA     P1_3      //IIC�������Ŷ���
#define   SCLBIT  BIT7
#define   SDABIT  BIT3

//�ܽŶ���
#define  _SDA  P1_3                      //IIC��������ΪP1_!
#define  _SDA_DIR  P1DIR                 //����P1�˿ڱ���

 
#define _SCL P1_7                        //IICʱ������ΪP1_2
#define _SCL_DIR P1DIR                   //����P1�˿ڱ���

#define   uchar  unsigned char
#define   uint   unsigned int
#define  uint16 unsigned short
//MPU6050�����ڲ�������ݵĵ�ַ
 #define        SMPLRT_DIV                    0x19        //�����ǲ����ʣ�����ֵ��0x07(125Hz)
 #define        CONFIG                        0x1A        //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
 #define        GYRO_CONFIG                   0x1B        //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
 #define        ACCEL_CONFIG                  0x1C        //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
 #define        ACCEL_XOUT_H                  0x3B
 #define        ACCEL_XOUT_L                  0x3C
 #define        ACCEL_YOUT_H                  0x3D
 #define        ACCEL_YOUT_L                  0x3E
 #define        ACCEL_ZOUT_H                  0x3F
 #define        ACCEL_ZOUT_L                  0x40
 #define        TEMP_OUT_H                    0x41
 #define        TEMP_OUT_L                    0x42
 #define        GYRO_XOUT_H                   0x43
 #define        GYRO_XOUT_L                   0x44        
 #define        GYRO_YOUT_H                   0x45
 #define        GYRO_YOUT_L                   0x46
 #define        GYRO_ZOUT_H                   0x47
 #define        GYRO_ZOUT_L                   0x48
 #define        PWR_MGMT_1                    0x6B        //��Դ��������ֵ��0x00(��������)
 #define        WHO_AM_I                      0x75        //IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
 #define MPU6050_SA_W   0xD0
 #define MPU6050_SA_R   0xD1


typedef   unsigned char BYTE;
typedef   unsigned short WORD;






void WriteDataLCM(uchar dataW);
void WriteCommandLCM(uchar CMD,uchar Attribc);
void DisplayOneChar(uchar X,uchar Y,uchar DData);



//��������
void _SDA_IN(void);                     //SDA����ģʽ
void _SDA_OUT(void);                    //SDA���ģʽ
void _SCL_IN(void);                     //SCL����ģʽ
void _SCL_OUT(void);                    //SCL����ģʽ
void _SDA_1(void);                      //SDAд1
void _SDA_0(void);                      //SDAд0
void _SCL_1(void);                      //SCLд1
void _SCL_0();                          //SCLд0
void NOP1();                            //��ʱ����
void NOP(void);                         //��ʱ����
void _Start(void);                      //��ʼ��������
void _Stop(void);                       //ֹͣ��������
void _Port_Init(void);                  //���ö˿�
uint _Wait_ACK(void);                  //�ȴ�ACK��Ӧ
uint _Write(uint data);               //SDA����ģʽ
uint _Read();                          //��ȡ����
uint _Write_Data(uint addr,uint wd); //
uint _Read_Data(uint addr);           //
void  _MPU_Init(void);                  //MPU��ʼ��
void read();

#define g 9.8

#endif