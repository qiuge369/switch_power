/* *
 * PWM���������ѹ��Ȼ��ADS118������ѹֵ���پ���PID�������׼ȷ�ĵ�ѹֵ��
 * __delay_cycles(20000000);//��ʱ5S��
 * * * */
#include <msp430f6638.h>
#include "oled.h"
#include "bmp.h"
#include "key_button.h"
#include "setclock.h"
#include "pid_delta.h"
#include "q_ADS1118.h"


//��������
void initPWM(void);
void initPara();
float getVoltage();
void pidAdjust(float in_voltage);
void changePWM(int duty_value);
void DispFloatat(unsigned char x,unsigned char y,float dat,unsigned char len1,unsigned char len2 );
void my_key();
void suprotect(float vol);

//��������
double duty=0;//ռ�ձ�
PID_DELTA pid;        //����pid�ṹ�����
double dealtV=0;  //pid�����
unsigned int AD_bit; //�����ȡADת����ֵ
float True_voltage=0;
int key_value;
double num=0;//����������ֵ

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    SetClock_MCLK12MHZ_SMCLK24MHZ_ACLK32_768K();//12MHz
//    UCSCTL5|=DIVS_2;//ʹ��USCͳһʱ��ϵͳ����Ԥ��Ƶ����SMCLK����4��Ƶ������������1M

    initPWM();
    initPara();//��ʼֵ
    OLED_Init();/*init OLED*/
    OLED_Clear(); /*clear OLED screen*/
    init_key();
    OLED_ShowString(0,0, "Voltage:");
    OLED_ShowString(0,2, "current:");
    OLED_ShowString(0,4, "set:");
    while(1)
    {
        True_voltage=getVoltage();

        if((True_voltage-pid.setPoint>=0.08)||(pid.setPoint-True_voltage>=0.080))
            pidAdjust(True_voltage);

        my_key();
        DispFloatat(72,4,pid.setPoint,2,3);//��ʾ
        DispFloatat(16,6,pid.Proportion,2,2);//��ʾ
        DispFloatat(72,6,pid.Integral,2,3);//��ʾ
    }
}

/******************************ADֵ��ȡ����**********************************/
int j=0,j_c=0;
float sum=0,sum_c=0;
float Voltage,Voltage_out=50;
float Voltage2;
float current;
float getVoltage()//��
{
    //��������ʱ��Ϊʲô�Ƿ���
    unsigned int Value,Value2;

    float Voltage2;
    float current;
    Value2 = Write_SIP(0xf38b);           //AD��ֵ     Conversion Register
    Voltage2=change_voltage(Value2,4.096);
    current=Voltage2/0.6052;
    DispFloatat(80,2,current,1,3);//��ʾ����ֵ
    suprotect(Voltage2);
    usleep(20);
    if(j>=100){
        Voltage_out=sum/100;
        DispFloatat(72,0,Voltage_out,2,3);//��ʾ��ѹֵ
        j=0;
        sum=0;
    }
    else
    {
        Value = Write_SIP(0xe38b);           //AD��ֵ     Conversion Register
        Voltage=change_voltage(Value,4.096);
        Voltage=Voltage*11.98;//-(1.519*current-0.1115)
        sum+=Voltage;
        j++;
    }
    return Voltage_out;

//        Voltage2=change_voltage(Value2,4.096);
//        current=Voltage2/0.6052;
//        DispFloatat(80,2,current,1,3);//��ʾ����ֵ
//        suprotect(Voltage2);
//        usleep(20);
//        Value = Write_SIP(0xe38b);           //AD��ֵ     Conversion Register
//        Voltage=change_voltage(Value,4.096);
//        Voltage=Voltage*11.98;//-(1.519*current-0.1115)
//        DispFloatat(72,0,Voltage,2,3);//��ʾ��ѹֵ
//        return Voltage;
}
/*****************************��������*********************************/
int c_i=0;
void suprotect(float vol)
{
    if(vol>1.625)
        {
            c_i++;
            if(c_i>10)
            {
                P8OUT |= BIT4;        //�ø�
                __delay_cycles(120000000);//��ʱ5S��
                P8OUT &= ~BIT4;        //�ø�
            }
        }
    else
        c_i=0;


}
/*****************************PID���ƺ�ѹ*********************************/
void pidAdjust(float in_voltage)
{
  dealtV = PidDeltaCal(&pid,in_voltage);  //�����������
  if((duty + dealtV) > 300)//65%
  {
      duty = 300;
    changePWM(duty);                      //��Ч����
  }
  else if((duty + dealtV) < 0)
  {
      duty = 0;
    changePWM(duty);                      //��Ч����
  }else{
      duty = duty + dealtV;                 //����ռ�ձ�
    changePWM(duty);                      //��Ч����
  }
}

/****************************�ı�PWMռ�ձ�*********************************/
void changePWM(int duty_value)//��
{
    TA0CCR1 = duty_value;
    TA0CCR2 = duty_value+1;//��֤��·PWM����������֮��ͬ��
}
/****************************PWM��ʼ�����*********************************/
void initPWM(void)//��
{
  P1DIR |= BIT2;
  P1SEL |= BIT2;        //ѡ��TA.1����

  P1DIR |= BIT3;
  P1SEL |= BIT3;        //ѡ��TA.1����

  TA0CTL |=TASSEL_2 + MC_3 + TACLR;//����A0������,ʱ��ԴSMCLK������ģʽ��ͬʱ���������//*���ü�����
  //TASSEL_2ѡ����SMCLK��MC_1����ģʽ�����������TACLR
  TA0CCTL0 = /*OUTMOD_7+*/  CCIE;//����ȽϼĴ���0��������ģʽΪ2��ͬʱʹ�ܶ�ʱ���жϣ�CCR0��Դ�жϣ���CCIE����ȽϼĴ�����ʹ������
  TA0CCR0 = 480;//����ȽϼĴ���,���ö�ʱ���ж�Ƶ��25K
  TA0CCTL1 |= OUTMOD_2; // TD0CCR1, Reset/Set
  TA0CCR1 = 240;             //ռ�ձ�CCR1/CCR0

  TA0CCTL2 |= OUTMOD_6; // TD0CCR2, Reset/Set
  TA0CCR2 = 240;             //ռ�ձ�CCR2/CCR0
}

/****************************���ó�ʼֵ*********************************/
void initPara()
{
  duty = 200;    //����ֵ����ȷ��
  pid.setPoint = 36;   ////�趨ֵ����ȷ��
  adjust_pid(&pid, 1.000, 0.100, 0);//����PIDϵ��
  adjust_pid_limit(&pid, -10, 10);//�趨PID������������Ʒ�Χ
  ADS1118_GPIO_Init();  //���ùܽţ�ģ��SPI������Vcc��GND��Ҫ6���ߣ���ȥ������Ҫ4���ߣ�����Ҫ�ܽ����ã�

  P8DIR |= BIT4;    //���������ܽ�
}

/****************************��������ʾ����********************************/
//dat:����    len1:������λ��    len2:С����λ��
const long numtab[]={
  1,10,100,1000,10000,100000,1000000,10000000,100000000,1000000000,10000000000};
char a;
void DispFloatat(unsigned char x,unsigned char y,float dat,unsigned char len1,unsigned char len2 )
{
    int dat1,dat2;
    dat1=(int)dat;
    dat2=(int)((dat-dat1)*numtab[len2]);
    OLED_ShowNum(x,y,dat1,len1,16);
    OLED_ShowString(x+8*len1,y, ".");
    if(dat2/numtab[len2-1]==0)
        {
            if(len2>2)
            {
                if(dat2/numtab[len2-2]==0){
                    OLED_ShowString(x+8*len1+8,y,"0");
                    OLED_ShowString(x+8*len1+16,y,"0");
                    OLED_ShowNum(x+8*len1+24,y,dat2,len2-2,16);
                }else{
                    OLED_ShowString(x+8*len1+8,y,"0");
                    OLED_ShowNum(x+8*len1+16,y,dat2,len2-1,16);
                }

            }  else{
                    OLED_ShowString(x+8*len1+8,y,"0");
                    OLED_ShowNum(x+8*len1+16,y,dat2,len2-1,16);
                }
        }
    else
        OLED_ShowNum(x+8*len1+8,y,dat2,len2,16);

}
/****************************��������********************************/
//������ֻ��Ҫ��ʾ��λ�԰ɣ�
int i=0;
void my_key()
{

    key_value= key();   /*scan Array_button, get the key value*/
            if(key_value!=0)
            {
                    if(i>1)//�ж���0����1
                        {
                           i=0;
                           if(num<=36.0&&num>=30.0)
                               pid.setPoint=num;//�趨������ѹֵ
                           OLED_ShowString(0,6, "    ");
                           num=0;
                        }
                    switch(key_value)
                    {
                        case(1):
                               OLED_ShowNum(8*i,6,1,1,16);  /*show the key value*/
                              switch(i)
                               {
                                   case 0:num+=10;break;
                                   case 1:num+=1;break;
                                   default:break;
                               }
                              i++;
                              key_value=0;
                              break;
                      case(2):
                          OLED_ShowNum(8*i,6,2,1,16);  /*show the key value*/
                          switch(i)
                           {
                               case 0:num+=20;break;
                               case 1:num+=2;break;
                               default:break;
                           }
                          i++;
                          key_value=0;
                          break;
                      case(3):
                            OLED_ShowNum(8*i,6,3,1,16);  /*show the key value*/
                            switch(i)
                             {
                                 case 0:num+=30;break;
                                 case 1:num+=3;break;
                                 default:break;
                             }
                            i++;
                            key_value=0;
                            break;
                      case(4)://A
                              if(pid.Proportion>0.0000001)
                                  pid.Proportion-=0.01;
//                            if(pid.setPoint<36.0)
//                                pid.setPoint+=1;//�����趨������ѹֵ
                              key_value=0;
                            break;
                      case(5):
                        OLED_ShowNum(8*i,6,4,1,16);  /*show the key value*/
                        switch(i)
                         {
                             case 0:num+=40;break;
                             case 1:num+=4;break;
                             default:break;
                         }
                        i++;
                        key_value=0;
                        break;
                      case(6):
                          OLED_ShowNum(8*i,6,5,1,16);  /*show the key value*/
                          switch(i)
                           {
                               case 0:num+=50;break;
                               case 1:num+=5;break;
                               default:break;
                           }
                          i++;
                          key_value=0;
                          break;
                      case(7):
                            OLED_ShowNum(8*i,6,6,1,16);  /*show the key value*/
                            switch(i)
                             {
                                 case 0:num+=60;break;
                                 case 1:num+=6;break;
                                 default:break;
                             }
                            i++;
                            key_value=0;
                            break;
                        case(8)://B
//                              if(pid.setPoint>30.0)
//                                  pid.setPoint-=1;//�趨������ѹֵ
                              if( pid.Integral>0.0000001)
                                  pid.Integral-=0.001;//��I
                              key_value=0;
                              break;
                      case(9):
                          OLED_ShowNum(8*i,6,7,1,16);  /*show the key value*/
                          switch(i)
                           {
                               case 0:num+=70;break;
                               case 1:num+=7;break;
                               default:break;
                           }
                          i++;
                          key_value=0;
                          break;
                      case(10):
                            OLED_ShowNum(8*i,6,8,1,16);  /*show the key value*/
                            switch(i)
                             {
                                 case 0:num+=80;break;
                                 case 1:num+=8;break;
                                 default:break;
                             }
                            i++;
                            key_value=0;
                            break;
                      case(11):
                            OLED_ShowNum(8*i,6,9,1,16);  /*show the key value*/
                            switch(i)
                              {
                                   case 0:num+=90;break;
                                   case 1:num+=9;break;
                                   default:break;
                              }
                            i++;
                            key_value=0;
                            break;
                      case(12)://C
                              pid.Proportion+=0.01;//��P
                              key_value=0;
                              break;
                      case(13):

                          break;
                      case(14)://0
                            OLED_ShowNum(8*i,6,0,1,16);  /*show the key value*/
                            switch(i)
                             {
                                 case 0:num+=0;break;
                                 case 1:num+=0;break;
                                 default:break;
                             }
                            i++;
                            key_value=0;
                            break;
                      case(15)://#ȷ��
//                            pid.setPoint=num;//�趨������ѹֵ
                            key_value=0;
                            break;
                      case(16)://D
                            pid.Integral+=0.001;//��I
                            key_value=0;
                            break;
                      default:break;
                    }
            }
}
