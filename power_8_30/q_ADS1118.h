/*
 * q_ADS1118.h
 *
 *  Created on: 2020年8月10日
 *      Author: HP
 */
#ifndef __Q_ADS1118_H_
#define __Q_ADS1118_H_

typedef unsigned short u16;

/*管脚电平定义*/
//5.2->CS 5.4->SCLK 7.4->MOSI（DIN） 1.0->MISO（DOUT）  （标准SPI无片选接口）
#define ADS1118_CS_H                P5OUT|=BIT2
#define ADS1118_CS_L                P5OUT&=~BIT2
#define ADS1118_SCLK_H              P5OUT|=BIT4
#define ADS1118_SCLK_L              P5OUT&=~BIT4
#define ADS1118_MISO_PINSTATE       (P1IN&BIT0)//1.0管脚的读取（要加括号，不然代码取决于运算优先级）
#define ADS1118_MOSI_H              P7OUT|=BIT4
#define ADS1118_MOSI_L              P7OUT&=~BIT4
#define usleep(x)                   __delay_cycles(x*100)

u16 Write_SIP(unsigned int temp);
void ADS1118_GPIO_Init(void);
float change_voltage(u16 AD_value,double FS);

#endif




