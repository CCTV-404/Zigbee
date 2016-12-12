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

#include "BH1750.h"

BYTE    BUF[8];
uchar   ge, shi, bai, qian, wan;            //显示变量
int     dis_data;


//*********************************************************
void conversion(uint temp_data)  //  数据转换出 个，十，百，千，万
{
    wan=temp_data/10000+0x30 ;
    temp_data=temp_data%10000;   //取余运算
    qian=temp_data/1000+0x30 ;
    temp_data=temp_data%1000;    //取余运算
    bai=temp_data/100+0x30   ;
    temp_data=temp_data%100;     //取余运算
    shi=temp_data/10+0x30    ;
    temp_data=temp_data%10;      //取余运算
    ge=temp_data+0x30; 	
}

/**************************************
延时5微秒
不同的工作环境,需要调整此函数，注意时钟过快时需要修改
当改用1T的MCU时,请调整此延时函数
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

//毫秒延时**************************
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
延时5毫秒(STC90C52RC@12M)
不同的工作环境,需要调整此函数
当改用1T的MCU时,请调整此延时函数
**************************************/
void Delay5ms()
{
  int i=0;
  for(i=0;i<1000;i++)
    Delay5us();
}

/**************************************
起始信号
**************************************/
//_Pragma("optimize=none")
void BH1750_Start()
{
    P1DIR |= (SCLBIT + SDABIT);
    SDA = 1;                    //拉高数据线
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SDA = 0;                    //产生下降沿
    Delay5us();                 //延时
    SCL = 0;                    //拉低时钟线
}

/**************************************
停止信号
**************************************/
//_Pragma("optimize=none")
void BH1750_Stop()
{
    P1DIR |= (SCLBIT + SDABIT);
    SDA = 0;                    //拉低数据线
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SDA = 1;                    //产生上升沿
    Delay5us();                 //延时
}

/**************************************
向IIC总线发送一个字节数据
**************************************/
_Pragma("optimize=none")
void BH1750_SendByte(BYTE dat)
{
    P1DIR |= (SCLBIT + SDABIT);
    BYTE i;

    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;              //移出数据的最高位
        SDA = CY;               //送数据口
        SCL = 1;                //拉高时钟线
        Delay5us();             //延时
        SCL = 0;                //拉低时钟线
        Delay5us();             //延时
    }
    SCL = 1;                    //拉高时钟线
    P1DIR &= ~SDABIT;                  //改变SDA方向
    Delay5us();                 //延时
    CY = SDA;                   //读应答信号
    SCL = 0;                    //拉低时钟线
    Delay5us();                 //延时
    P1DIR |= (SCLBIT + SDABIT);
}

/**************************************
从IIC总线接收一个字节数据
**************************************/
//_Pragma("optimize=none")
BYTE BH1750_RecvByte()
{
    P1DIR |= SCLBIT;
    P1DIR &= ~SDABIT;
    BYTE i;
    BYTE dat = 0;

    SDA = 1;                    //使能内部上拉,准备读取数据,
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        SCL = 1;                //拉高时钟线
        Delay5us();             //延时
        dat |= SDA;             //读数据
        SCL = 0;                //拉低时钟线
        Delay5us();             //延时
    }
    P1DIR |= (SCLBIT + SDABIT);

    return dat;

}

//*********************************
//_Pragma("optimize=none")
void Single_Write_BH1750(uchar REG_Address)   
{
    BH1750_Start();                  //起始信号
    BH1750_SendByte(SlaveAddress);   //发送设备地址+写信号
    BH1750_SendByte(REG_Address);    //内部寄存器地址，
  //  BH1750_SendByte(REG_data);       //内部寄存器数据，
    BH1750_Stop();                   //发送停止信号
}

//********单字节读取***************
//_Pragma("optimize=none")
/*uchar Single_Read_BH1750(uchar REG_Address)
{  uchar REG_data;
    BH1750_Start();                          //起始信号
    BH1750_SendByte(SlaveAddress);           //发送设备地址+写信号
    BH1750_SendByte(REG_Address);                   //发送存储单元地址，从0开始	
    BH1750_Start();                          //起始信号
    BH1750_SendByte(SlaveAddress+1);         //发送设备地址+读信号
    REG_data=BH1750_RecvByte();              //读出寄存器数据
	BH1750_SendACK(1);
	BH1750_Stop();                           //停止信号
    return REG_data;
}*/

//*********************************************************
//
//连续读出BH1750内部数据
//
//*********************************************************
//_Pragma("optimize=none")
void Multiple_Read()
{   uchar i;	
    BH1750_Start();                          //起始信号
    BH1750_SendByte(SlaveAddress+1);         //发送设备地址+读信号
	
   for (i=0; i<3; i++)                      //连续读取2个地址数据，存储中BUF
    {
        BUF[i] = BH1750_RecvByte();          //BUF[0]存储0x32地址中的数据
        if (i == 3)
        {
           P1DIR |= (SCLBIT + SDABIT);
           SDA = 1;                  //写应答信号
           SCL = 1;                    //拉高时钟线
           Delay5us();                 //延时
           SCL = 0;                    //拉低时钟线
           Delay5us();                 //延时
        }
        else
        {		
           P1DIR |= (SCLBIT + SDABIT);
           SDA = 0;                  //写应答信号
           SCL = 1;                    //拉高时钟线
           Delay5us();                 //延时
           SCL = 0;                    //拉低时钟线
           Delay5us();                 //延时
       }
   }

    BH1750_Stop();                          //停止信号
    Delay5ms();
}


//_Pragma("optimize=none")
void Init_BH1750()
{
   Single_Write_BH1750(0x08);
}

