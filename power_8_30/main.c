/* *
 * PWM控制输出电压，然后ADS118采样电压值，再经过PID调节输出准确的电压值。
 * __delay_cycles(20000000);//延时5S？
 * * * */
#include <msp430f6638.h>
#include "oled.h"
#include "bmp.h"
#include "key_button.h"
#include "setclock.h"
#include "pid_delta.h"
#include "q_ADS1118.h"


//函数声明
void initPWM(void);
void initPara();
float getVoltage();
void pidAdjust(float in_voltage);
void changePWM(int duty_value);
void DispFloatat(unsigned char x,unsigned char y,float dat,unsigned char len1,unsigned char len2 );
void my_key();
void suprotect(float vol);

//变量声明
double duty=0;//占空比
PID_DELTA pid;        //声明pid结构体变量
double dealtV=0;  //pid误差量
unsigned int AD_bit; //定义读取AD转换数值
float True_voltage=0;
int key_value;
double num=0;//按键所得数值

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    SetClock_MCLK12MHZ_SMCLK24MHZ_ACLK32_768K();//12MHz
//    UCSCTL5|=DIVS_2;//使用USC统一时钟系统进行预分频，将SMCLK进行4分频——————1M

    initPWM();
    initPara();//初始值
    OLED_Init();/*init OLED*/
    OLED_Clear(); /*clear OLED screen*/
    init_key();
    OLED_ShowString(0,0, "Voltage:");
    OLED_ShowString(0,2, "current:");
    OLED_ShowString(0,4, "set:");
    while(1)
    {
        True_voltage=getVoltage();

        if((True_voltage-pid.setPoint>=0.011)||(pid.setPoint-True_voltage>=0.011))
            pidAdjust(True_voltage);

        my_key();
        DispFloatat(72,4,pid.setPoint,2,3);//显示
        DispFloatat(16,6,pid.Proportion,2,2);//显示
        DispFloatat(72,6,pid.Integral,2,3);//显示
    }
}

/******************************AD值读取函数**********************************/
int j=0;
float sum=0;
float getVoltage()//可
{
    //测两个的时候为什么是反的
        unsigned int Value,Value2;
        float Voltage,Voltage_out=40;
        float Voltage2;
        float current;
        Value2 = Write_SIP(0xf38b);           //AD数值     Conversion Register
        Voltage2=change_voltage(Value2,4.096);
        current=Voltage2/0.6052;
        DispFloatat(80,2,current,1,3);//显示电流值
        suprotect(Voltage2);
        usleep(20);
        if(j>5){
            Voltage_out=sum/5;
            DispFloatat(72,0,Voltage_out,2,3);//显示电压值
            j=0;
            sum=0;
        }
        else
        {
            Value = Write_SIP(0xe38b);           //AD数值     Conversion Register
            Voltage=change_voltage(Value,4.096);
            Voltage=Voltage*11.98;//-(1.519*current-0.1115)
            sum+=Voltage;
            j++;
        }
        return Voltage_out;

//        Value = Write_SIP(0xe38b);           //AD数值     Conversion Register
//        Voltage=change_voltage(Value,4.096);
//        Voltage=Voltage*11.98;//-(1.519*current-0.1115)
//        DispFloatat(72,0,Voltage,2,3);//显示电压值
//        return Voltage;
}
/*****************************过流保护*********************************/
int c_i=0;
void suprotect(float vol)
{
    if(vol>1.625)
        {
            c_i++;
            if(c_i>10)
            {
                P8OUT |= BIT4;        //置高
                __delay_cycles(120000000);//延时5S？
                P8OUT &= ~BIT4;        //置高
            }
        }
    else
        c_i=0;


}
/*****************************PID控制恒压*********************************/
void pidAdjust(float in_voltage)
{
  dealtV = PidDeltaCal(&pid,in_voltage);  //返回误差增量
  if((duty + dealtV) > 312)//65%
  {
      duty = 312;
    changePWM(duty);                      //生效控制
  }
  else if((duty + dealtV) < 0)
  {
      duty = 0;
    changePWM(duty);                      //生效控制
  }else{
      duty = duty + dealtV;                 //修正占空比
    changePWM(duty);                      //生效控制
  }
}

/****************************改变PWM占空比*********************************/
void changePWM(int duty_value)//可
{
    TA0CCR1 = duty_value;
    TA0CCR2 = duty_value+1;//保证两路PWM波除了死区之外同步
}
/****************************PWM初始化输出*********************************/
void initPWM(void)//可
{
  P1DIR |= BIT2;
  P1SEL |= BIT2;        //选择TA.1功能

  P1DIR |= BIT3;
  P1SEL |= BIT3;        //选择TA.1功能

  TA0CTL |=TASSEL_2 + MC_3 + TACLR;//配置A0计数器,时钟源SMCLK，上升模式，同时清除计数器//*配置计数器
  //TASSEL_2选择了SMCLK，MC_1计数模式，，最后清零TACLR
  TA0CCTL0 = /*OUTMOD_7+*/  CCIE;//捕获比较寄存器0输出，输出模式为2，同时使能定时器中断（CCR0单源中断），CCIE捕获比较寄存器的使能配置
  TA0CCR0 = 480;//捕获比较寄存器,设置定时器中断频率25K
  TA0CCTL1 |= OUTMOD_2; // TD0CCR1, Reset/Set
  TA0CCR1 = 240;             //占空比CCR1/CCR0

  TA0CCTL2 |= OUTMOD_6; // TD0CCR2, Reset/Set
  TA0CCR2 = 240;             //占空比CCR2/CCR0
}

/****************************设置初始值*********************************/
void initPara()
{
  duty = 200;    //测试值？不确定
  pid.setPoint = 36;   ////设定值，不确定
  adjust_pid(&pid, 0.69, 0.029, 0);//调整PID系数
  adjust_pid_limit(&pid, -10, 10);//设定PID误差增量的限制范围
  ADS1118_GPIO_Init();  //配置管脚（模拟SPI，加上Vcc、GND需要6根线，除去这俩需要4根线，故需要管脚配置）

  P8DIR |= BIT4;    //过流保护管脚
}

/****************************浮点数显示函数********************************/
//dat:数据    len1:整数的位数    len2:小数的位数
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
/****************************按键函数********************************/
//按键是只需要显示两位对吧？
int i=0;
void my_key()
{

    key_value= key();   /*scan Array_button, get the key value*/
            if(key_value!=0)
            {
                    if(i>1)//判断是0还是1
                        {
                           i=0;
                           if(num<=36.0&&num>=30.0)
                               pid.setPoint=num;//设定期望电压值
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
//                                pid.setPoint+=1;//步进设定期望电压值
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
//                                  pid.setPoint-=1;//设定期望电压值
                              if( pid.Integral>0.0000001)
                                  pid.Integral-=0.001;//调I
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
                              pid.Proportion+=0.01;//调P
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
                      case(15)://#确定
//                            pid.setPoint=num;//设定期望电压值
                            key_value=0;
                            break;
                      case(16)://D
                            pid.Integral+=0.001;//调I
                            key_value=0;
                            break;
                      default:break;
                    }
            }
}
