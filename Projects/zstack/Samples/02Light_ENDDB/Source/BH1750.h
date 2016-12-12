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

#define   uchar  unsigned char
#define   uint   unsigned int
#define	  SlaveAddress   0x46 //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�
                              //ALT  ADDRESS���Žӵ�ʱ��ַΪ0x46���ӵ�Դʱ��ַΪ0xB8
typedef   unsigned char BYTE;
typedef   unsigned short WORD;

extern BYTE    BUF[8];                         //�������ݻ�����      	
extern int     dis_data;                       //���ձ���

void delay_nms(unsigned int k);
void Init_BH1750(void);

void WriteDataLCM(uchar dataW);
void WriteCommandLCM(uchar CMD,uchar Attribc);
void DisplayOneChar(uchar X,uchar Y,uchar DData);
extern void conversion(uint temp_data);

extern void  Single_Write_BH1750(uchar REG_Address);               //����д������
//uchar Single_Read_BH1750(uchar REG_Address);                //������ȡ�ڲ��Ĵ�������
extern void  Multiple_Read(void);                               //�����Ķ�ȡ�ڲ��Ĵ�������

void Delay5us(void);
void Delay5ms(void);
void BH1750_Start(void);                    //��ʼ�ź�
void BH1750_Stop(void);                     //ֹͣ�ź�
void BH1750_SendByte(BYTE dat);         //IIC�����ֽ�д
BYTE BH1750_RecvByte(void);                 //IIC�����ֽڶ�

#endif