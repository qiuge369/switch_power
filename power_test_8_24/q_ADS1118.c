/*
 * q_ADS1118.c
 *
 *  Created on: 2020��8��10��
 *      Author: HP
 */
#include "msp430.h"
#include"intrinsics.h"

#include "Q_ADS1118.H"

/************************************************************
*��������: Write_SIP(unsigned int temp)
*��������: ��ADS118д�����üĴ���ֵ�����Ҷ���AD��ֵ
*��ڲ���: temp (д�����üĴ���)
*������ֵ: Read_Data
* ˵  �� ������ʱ��ͼ��ǰ16λ������ADת����ֵ ��16λ���ص����üĴ���
* ��ADS1118_CS_L��ADS1118_CS_Hע�͵�����16bitģʽ�������Ĵ������ò�������ӦADֵ��������ؼĴ�����ֵ��
**************************************************************/
u16 Write_SIP(unsigned int temp)
{
    //2.1->CS 2.2->SCLK 2.3->MOSI 2.4->MISO   ����׼SPI��Ƭѡ�ӿڣ�
    //5.2->CS 5.4->SCLK 7.4->MOSI 1.0->MISO  ����׼SPI��Ƭѡ�ӿڣ�
    char i;
    u16 Read_Data;
    P7DIR |= BIT4;                   //     DIN -->��Ӧ��MOSI��
    P1DIR &=~BIT0;                    //����P3.1Ϊ�������� Dout-->��Ӧ��MISO��;

    ADS1118_SCLK_L;
    ADS1118_CS_L;
    for(i=0;i<16;i++)           //ֻ����ʱ��������ʱ���ݱ����棬Ϊ������ͨ���½��ض�ȡ����
    {
      if((temp&0x8000)==0x8000)    //���λΪ1��
      {
          ADS1118_MOSI_H;
      }
      else
      {
          ADS1118_MOSI_L;
      }
      temp<<=1;

      ADS1118_SCLK_H;

      Read_Data<<=1;
      if(ADS1118_MISO_PINSTATE)//�ж�Ϊ0���0
      {
//          Read_Data++;
          Read_Data|=0x0001 ;
      }
      __delay_cycles(10000);//15000
      ADS1118_SCLK_L;

    }
    ADS1118_MOSI_L;

    ADS1118_CS_H;  //����Config Register������
    _NOP();

    return Read_Data;

}

/************************************************************
*��������: ADS1118_GPIO_Init()
*��������: ����ADS1118��IO�ڳ�ʼ��
*��ڲ���:
*������ֵ:
**************************************************************/

void ADS1118_GPIO_Init(void)
{//2.1->CS 2.2->SCLK 2.3->MOSI 2.4->MISO   ����׼SPI��Ƭѡ�ӿڣ�
 //5.2->CS 5.4->SCLK 7.4->MOSI 1.0->MISO  ����׼SPI��Ƭѡ�ӿڣ�
    P7DIR |= BIT4;//MOSI
    P5DIR |= BIT2;//CS

    P1DIR &= ~BIT0;//MISO
    P1REN |= BIT0;
    P1OUT &=~ BIT0;

    P5DIR |= BIT4;//clk
}
/************************************************************
*��������: change_voltage()
*��������: ����ADS1118��IO�ڳ�ʼ��
*��ڲ���:
*������ֵ:
**************************************************************/
float change_voltage(u16 AD_value,double FS)//��������AD��ֵת��Ϊ��ѹֵ
{
    //FS��ָ��ѹ�ȼ�
    float AD_Voltage;
    if(AD_value>=0x8000)
      {
        AD_value=0xFFFF-AD_value;//��0xFFFF�ĳ�0x10000
        AD_Voltage=(-1.0)*((AD_value*FS/0x8000));
      }
    else
        AD_Voltage=(1.0)*((AD_value*FS/32768));
    return AD_Voltage;
}
