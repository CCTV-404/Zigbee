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

#define   SCL     P1_7      //IIC时钟引脚定义
#define   SDA     P1_3      //IIC数据引脚定义
#define   SCLBIT  BIT7
#define   SDABIT  BIT3

//管脚定义
#define  _SDA  P1_3                      //IIC数据引脚为P1_!
#define  _SDA_DIR  P1DIR                 //定义P1端口别名

 
#define _SCL P1_7                        //IIC时钟引脚为P1_2
#define _SCL_DIR P1DIR                   //定义P1端口别名

#define   uchar  unsigned char
#define   uint   unsigned int
#define  uint16 unsigned short
//MPU6050定义内部存放数据的地址
 #define        SMPLRT_DIV                    0x19        //陀螺仪采样率，典型值：0x07(125Hz)
 #define        CONFIG                        0x1A        //低通滤波频率，典型值：0x06(5Hz)
 #define        GYRO_CONFIG                   0x1B        //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
 #define        ACCEL_CONFIG                  0x1C        //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
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
 #define        PWR_MGMT_1                    0x6B        //电源管理，典型值：0x00(正常启用)
 #define        WHO_AM_I                      0x75        //IIC地址寄存器(默认数值0x68，只读)
 #define MPU6050_SA_W   0xD0
 #define MPU6050_SA_R   0xD1


typedef   unsigned char BYTE;
typedef   unsigned short WORD;






void WriteDataLCM(uchar dataW);
void WriteCommandLCM(uchar CMD,uchar Attribc);
void DisplayOneChar(uchar X,uchar Y,uchar DData);



//函数申明
void _SDA_IN(void);                     //SDA输入模式
void _SDA_OUT(void);                    //SDA输出模式
void _SCL_IN(void);                     //SCL输入模式
void _SCL_OUT(void);                    //SCL输入模式
void _SDA_1(void);                      //SDA写1
void _SDA_0(void);                      //SDA写0
void _SCL_1(void);                      //SCL写1
void _SCL_0();                          //SCL写0
void NOP1();                            //延时函数
void NOP(void);                         //延时函数
void _Start(void);                      //开始工作函数
void _Stop(void);                       //停止工作函数
void _Port_Init(void);                  //设置端口
uint _Wait_ACK(void);                  //等待ACK回应
uint _Write(uint data);               //SDA输入模式
uint _Read();                          //读取数据
uint _Write_Data(uint addr,uint wd); //
uint _Read_Data(uint addr);           //
void  _MPU_Init(void);                  //MPU初始化
void read();

#define g 9.8

#endif