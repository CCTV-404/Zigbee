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

#define   uchar  unsigned char
#define   uint   unsigned int
#define	  SlaveAddress   0x46 //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
                              //ALT  ADDRESS引脚接地时地址为0x46，接电源时地址为0xB8
typedef   unsigned char BYTE;
typedef   unsigned short WORD;

extern BYTE    BUF[8];                         //接收数据缓存区      	
extern int     dis_data;                       //关照变量

void delay_nms(unsigned int k);
void Init_BH1750(void);

void WriteDataLCM(uchar dataW);
void WriteCommandLCM(uchar CMD,uchar Attribc);
void DisplayOneChar(uchar X,uchar Y,uchar DData);
extern void conversion(uint temp_data);

extern void  Single_Write_BH1750(uchar REG_Address);               //单个写入数据
//uchar Single_Read_BH1750(uchar REG_Address);                //单个读取内部寄存器数据
extern void  Multiple_Read(void);                               //连续的读取内部寄存器数据

void Delay5us(void);
void Delay5ms(void);
void BH1750_Start(void);                    //起始信号
void BH1750_Stop(void);                     //停止信号
void BH1750_SendByte(BYTE dat);         //IIC单个字节写
BYTE BH1750_RecvByte(void);                 //IIC单个字节读

#endif