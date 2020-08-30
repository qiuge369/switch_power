/*
 * key_button.c
 *
 *  Created on: 2020年01月5日
 *      Author: SkingWei
 *      8月25把 _delay_cycles(300000);  //0.3s SCLK=1M改为了
 *      _delay_cycles(600000);  //0.3s SCLK=1M
 */
#include "msp430.h"
#include "key_button.h"
unsigned char KeyVal;
const unsigned char KeyOut[4] = {0xef, 0xdf, 0xbf, 0x7f }; // scan value of row
void init_key(void)
{
    /*(BIT0+BIT1+BIT2+BIT3)=0xf*/
    P4REN |= 0x0f;           /*Configure pull-up resistor*/
    P4DIR &= ~(0x0f);        /*Configure as input*/
    P4DIR |= 0xf0;           /*Configure as output*/
}
unsigned short int key()
{
    unsigned char ReadData[4];
    int i;
    for (i = 0; i < 4; i++)
    {
        P4OUT = KeyOut[i]|0x0f; //ignore low four bits
        ReadData[i] = (P4IN | 0xf0) ^ 0xff;// get 4 column value -> Determine which column is pressed , i -> i row
        switch (ReadData[i])
       {
          /*the fourth column*/
          case 0x08:
              KeyVal = 4*(i+1);
              _delay_cycles(3600000);  //0.3s SCLK=1M
              return KeyVal;

          /*the third column*/
          case 0x04:
              KeyVal = 4*(i+1)-1;
              _delay_cycles(3600000);  //0.3s SCLK=1M
              return KeyVal;

          /*the second column*/
          case 0x02:
              KeyVal = 4*(i+1)-2;
              _delay_cycles(3600000);  //0.3s SCLK=1M
              return KeyVal;

          /*the first column*/
          case 0x01:
              KeyVal = 4*(i+1)-3;
              _delay_cycles(3600000);  //0.3s SCLK=1M
              return KeyVal;

          default:
              KeyVal = 0;            //default key value
              break;
          }
     }
    return KeyVal;

}



