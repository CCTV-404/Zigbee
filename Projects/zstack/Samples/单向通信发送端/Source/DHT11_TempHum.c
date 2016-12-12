//DHT11�¶�ʪ�Ȳɼ�����ʵ��

#include <ioCC2530.h>
#include "OnBoard.h"
#define uint unsigned int
#define U8 unsigned char


#define DATA_PIN P1_3 

/*******��������*********/
void Delay_us(void);      //1us��ʱ
void Delay_10us(void);    //10us��ʱ
void Delay_ms(uint Time); //nms��ʱ
void COMM(void);          //��ʪд��
void Read_DHT11(void) ;   //��ʪ��������


//��ʪ�ȶ���
U8 U8FLAG,U8temp;
U8 Hum_H,Hum_L;           //����ʪ�ȴ�ű���
U8 Temp,Hum;              //�����¶ȴ�ű���
U8 U8T_data_H,U8T_data_L,U8RH_data_H,U8RH_data_L,U8checkdata;
U8 U8T_data_H_temp,U8T_data_L_temp,U8RH_data_H_temp,U8RH_data_L_temp,U8checkdata_temp;
U8 U8comdata;


/****************************
//��ʱ����
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
   ��ʪ�ȴ���
***********************/
void COMM(void)	// ��ʪд��
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

void Read_DHT11(void)   //��ʪ��������
{
    DATA_PIN = 0;
    Delay_ms(19);  //��������18ms
    DATA_PIN = 1;     //������������������ ������ʱ40us 
    P1DIR &=  ~0x08; //��������IO�ڷ���
    Delay_10us();
    Delay_10us();						
    Delay_10us();
    Delay_10us();  
    //�жϴӻ��Ƿ��е͵�ƽ��Ӧ�ź� �粻��Ӧ����������Ӧ���������� 
     if(!DATA_PIN) 
     {
      U8FLAG = 2; //�жϴӻ��Ƿ񷢳� 80us �ĵ͵�ƽ��Ӧ�ź��Ƿ���� 
      while((!DATA_PIN)&&U8FLAG++);
      U8FLAG = 2;//�жϴӻ��Ƿ񷢳� 80us �ĸߵ�ƽ���緢����������ݽ���״̬
      while((DATA_PIN)&&U8FLAG++); 
      COMM();//���ݽ���״̬ 
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
      //����У�� 
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