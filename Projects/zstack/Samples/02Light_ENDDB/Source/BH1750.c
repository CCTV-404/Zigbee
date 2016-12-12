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

#include "BH1750.h"

BYTE    BUF[8];
uchar   ge, shi, bai, qian, wan;            //��ʾ����
int     dis_data;


//*********************************************************
void conversion(uint temp_data)  //  ����ת���� ����ʮ���٣�ǧ����
{
    wan=temp_data/10000+0x30 ;
    temp_data=temp_data%10000;   //ȡ������
    qian=temp_data/1000+0x30 ;
    temp_data=temp_data%1000;    //ȡ������
    bai=temp_data/100+0x30   ;
    temp_data=temp_data%100;     //ȡ������
    shi=temp_data/10+0x30    ;
    temp_data=temp_data%10;      //ȡ������
    ge=temp_data+0x30; 	
}

/**************************************
��ʱ5΢��
��ͬ�Ĺ�������,��Ҫ�����˺�����ע��ʱ�ӹ���ʱ��Ҫ�޸�
������1T��MCUʱ,���������ʱ����
**************************************/
void Delay5us()
{
  asm("nop");//asm("nop");//asm("nop");asm("nop");
  //asm("nop");asm("nop");asm("nop");asm("nop");
  //asm("nop");asm("nop");asm("nop");asm("nop");
  //asm("nop");asm("nop");asm("nop");asm("nop");
  //asm("nop");asm("nop");asm("nop");asm("nop");
  //asm("nop");asm("nop");asm("nop");asm("nop");
  //asm("nop");asm("nop");asm("nop");asm("nop");
 // asm("nop");asm("nop");asm("nop");asm("nop");
}

//������ʱ**************************
void delay_nms(unsigned int k)	
{						
unsigned int i,j;	
for(i=0;i<k;i++)
{
  for(j=0;j<200;j++)
  {
    Delay5us();
  }	
}
}

/**************************************
��ʱ5����(STC90C52RC@12M)
��ͬ�Ĺ�������,��Ҫ�����˺���
������1T��MCUʱ,���������ʱ����
**************************************/
void Delay5ms()
{
  int i=0;
  for(i=0;i<1000;i++)
    Delay5us();
}

/**************************************
��ʼ�ź�
**************************************/
//_Pragma("optimize=none")
void BH1750_Start()
{
    P1DIR |= (SCLBIT + SDABIT);
    SDA = 1;                    //����������
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    SDA = 0;                    //�����½���
    Delay5us();                 //��ʱ
    SCL = 0;                    //����ʱ����
}

/**************************************
ֹͣ�ź�
**************************************/
//_Pragma("optimize=none")
void BH1750_Stop()
{
    P1DIR |= (SCLBIT + SDABIT);
    SDA = 0;                    //����������
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    SDA = 1;                    //����������
    Delay5us();                 //��ʱ
}

/**************************************
��IIC���߷���һ���ֽ�����
**************************************/
_Pragma("optimize=none")
void BH1750_SendByte(BYTE dat)
{
    P1DIR |= (SCLBIT + SDABIT);
    BYTE i;

    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;              //�Ƴ����ݵ����λ
        SDA = CY;               //�����ݿ�
        SCL = 1;                //����ʱ����
        Delay5us();             //��ʱ
        SCL = 0;                //����ʱ����
        Delay5us();             //��ʱ
    }
    SCL = 1;                    //����ʱ����
    P1DIR &= ~SDABIT;                  //�ı�SDA����
    Delay5us();                 //��ʱ
    CY = SDA;                   //��Ӧ���ź�
    SCL = 0;                    //����ʱ����
    Delay5us();                 //��ʱ
    P1DIR |= (SCLBIT + SDABIT);
}

/**************************************
��IIC���߽���һ���ֽ�����
**************************************/
//_Pragma("optimize=none")
BYTE BH1750_RecvByte()
{
    P1DIR |= SCLBIT;
    P1DIR &= ~SDABIT;
    BYTE i;
    BYTE dat = 0;

    SDA = 1;                    //ʹ���ڲ�����,׼����ȡ����,
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        SCL = 1;                //����ʱ����
        Delay5us();             //��ʱ
        dat |= SDA;             //������
        SCL = 0;                //����ʱ����
        Delay5us();             //��ʱ
    }
    P1DIR |= (SCLBIT + SDABIT);

    return dat;

}

//*********************************
//_Pragma("optimize=none")
void Single_Write_BH1750(uchar REG_Address)   
{
    BH1750_Start();                  //��ʼ�ź�
    BH1750_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�
    BH1750_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ��
  //  BH1750_SendByte(REG_data);       //�ڲ��Ĵ������ݣ�
    BH1750_Stop();                   //����ֹͣ�ź�
}

//********���ֽڶ�ȡ***************
//_Pragma("optimize=none")
/*uchar Single_Read_BH1750(uchar REG_Address)
{  uchar REG_data;
    BH1750_Start();                          //��ʼ�ź�
    BH1750_SendByte(SlaveAddress);           //�����豸��ַ+д�ź�
    BH1750_SendByte(REG_Address);                   //���ʹ洢��Ԫ��ַ����0��ʼ	
    BH1750_Start();                          //��ʼ�ź�
    BH1750_SendByte(SlaveAddress+1);         //�����豸��ַ+���ź�
    REG_data=BH1750_RecvByte();              //�����Ĵ�������
	BH1750_SendACK(1);
	BH1750_Stop();                           //ֹͣ�ź�
    return REG_data;
}*/

//*********************************************************
//
//��������BH1750�ڲ�����
//
//*********************************************************
//_Pragma("optimize=none")
void Multiple_Read()
{   uchar i;	
    BH1750_Start();                          //��ʼ�ź�
    BH1750_SendByte(SlaveAddress+1);         //�����豸��ַ+���ź�
	
   for (i=0; i<3; i++)                      //������ȡ2����ַ���ݣ��洢��BUF
    {
        BUF[i] = BH1750_RecvByte();          //BUF[0]�洢0x32��ַ�е�����
        if (i == 3)
        {
           P1DIR |= (SCLBIT + SDABIT);
           SDA = 1;                  //дӦ���ź�
           SCL = 1;                    //����ʱ����
           Delay5us();                 //��ʱ
           SCL = 0;                    //����ʱ����
           Delay5us();                 //��ʱ
        }
        else
        {		
           P1DIR |= (SCLBIT + SDABIT);
           SDA = 0;                  //дӦ���ź�
           SCL = 1;                    //����ʱ����
           Delay5us();                 //��ʱ
           SCL = 0;                    //����ʱ����
           Delay5us();                 //��ʱ
       }
   }

    BH1750_Stop();                          //ֹͣ�ź�
    Delay5ms();
}


//_Pragma("optimize=none")
void Init_BH1750()
{
   Single_Write_BH1750(0x08);
}

