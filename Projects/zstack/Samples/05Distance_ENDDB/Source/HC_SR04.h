#ifndef  __HC_SR04_H__
#define  __HC_SR04_H__

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

#define uint unsigned int
#define uchar unsigned char

#define TRIG P1_7     //P0_5引脚连接HC_SR04的TRIG引脚
#define ECHO P1_3     //P0_6引脚连接HC_SR04的ECHO引脚

//函数声明
void SysClkSet32M(void);
void Delay_10us(void);
void Delayms(uint xms);
void Init_GPIO(void);
void Init_T1(void);
void Init_Key1(void);



void WriteDataLCM(uchar dataW);
void WriteCommandLCM(uchar CMD,uchar Attribc);
void DisplayOneChar(uchar X,uchar Y,uchar DData);


#endif