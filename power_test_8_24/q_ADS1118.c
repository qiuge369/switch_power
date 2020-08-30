/*
 * q_ADS1118.c
 *
 *  Created on: 2020年8月10日
 *      Author: HP
 */
#include "msp430.h"
#include"intrinsics.h"

#include "Q_ADS1118.H"

/************************************************************
*函数名称: Write_SIP(unsigned int temp)
*功能描述: 向ADS118写入配置寄存器值，并且读回AD数值
*入口参数: temp (写入配置寄存器)
*返回数值: Read_Data
* 说  明 ：根据时序图，前16位读的是AD转换数值 后16位读回的配置寄存器
* 把ADS1118_CS_L和ADS1118_CS_H注释掉就是16bit模式，传进寄存器配置并读出对应AD值，不会读回寄存器的值。
**************************************************************/
u16 Write_SIP(unsigned int temp)
{
    //2.1->CS 2.2->SCLK 2.3->MOSI 2.4->MISO   （标准SPI无片选接口）
    //5.2->CS 5.4->SCLK 7.4->MOSI 1.0->MISO  （标准SPI无片选接口）
    char i;
    u16 Read_Data;
    P7DIR |= BIT4;                   //     DIN -->对应（MOSI）
    P1DIR &=~BIT0;                    //设置P3.1为数据输入 Dout-->对应（MISO）;

    ADS1118_SCLK_L;
    ADS1118_CS_L;
    for(i=0;i<16;i++)           //只有在时钟上升沿时数据被锁存，为控制器通过下降沿读取数据
    {
      if((temp&0x8000)==0x8000)    //最高位为1？
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
      if(ADS1118_MISO_PINSTATE)//判断为0或非0
      {
//          Read_Data++;
          Read_Data|=0x0001 ;
      }
      __delay_cycles(10000);//15000
      ADS1118_SCLK_L;

    }
    ADS1118_MOSI_L;

    ADS1118_CS_H;  //拉高Config Register读错误
    _NOP();

    return Read_Data;

}

/************************************************************
*函数名称: ADS1118_GPIO_Init()
*功能描述: 连接ADS1118的IO口初始化
*入口参数:
*返回数值:
**************************************************************/

void ADS1118_GPIO_Init(void)
{//2.1->CS 2.2->SCLK 2.3->MOSI 2.4->MISO   （标准SPI无片选接口）
 //5.2->CS 5.4->SCLK 7.4->MOSI 1.0->MISO  （标准SPI无片选接口）
    P7DIR |= BIT4;//MOSI
    P5DIR |= BIT2;//CS

    P1DIR &= ~BIT0;//MISO
    P1REN |= BIT0;
    P1OUT &=~ BIT0;

    P5DIR |= BIT4;//clk
}
/************************************************************
*函数名称: change_voltage()
*功能描述: 连接ADS1118的IO口初始化
*入口参数:
*返回数值:
**************************************************************/
float change_voltage(u16 AD_value,double FS)//将读出的AD数值转换为电压值
{
    //FS是指电压等级
    float AD_Voltage;
    if(AD_value>=0x8000)
      {
        AD_value=0xFFFF-AD_value;//把0xFFFF改成0x10000
        AD_Voltage=(-1.0)*((AD_value*FS/0x8000));
      }
    else
        AD_Voltage=(1.0)*((AD_value*FS/32768));
    return AD_Voltage;
}
