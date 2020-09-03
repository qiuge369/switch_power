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
void usrt_key();
void initPWM(void);
void initPara();
float getVoltage();
void pidAdjust(float in_voltage);
void changePWM(int duty_value);
void DispFloatat(unsigned char x,unsigned char y,float dat,unsigned char len1,unsigned char len2 );
void my_key();
void suprotect(float vol);

void init_uart_115200(void);

//变量声明
double duty=0;//占空比
PID_DELTA pid;        //声明pid结构体变量
double dealtV=0;  //pid误差量
float True_voltage=0;
int key_value;
double num=0;//按键所得数值

int j=0,j_c=0;
float sum=0,sum_c=0;
float Voltage,Voltage_out=50;
float Voltage2;
float current;

char str[4]={0};
int char_num=0;
char disp[4]={0};
char * sendstr;
int q_num = -1;
int recive=0;

int open=0;
int main(void)
 {
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    SetClock_MCLK12MHZ_SMCLK24MHZ_ACLK32_768K();//12MHz
    init_uart_115200();
    initPWM();
    initPara();//初始值
    OLED_Init();/*init OLED*/
    OLED_Clear(); /*clear OLED screen*/
    init_key();
    OLED_ShowString(0,0, "Voltage:");
    OLED_ShowString(0,2, "current:");
    OLED_ShowString(0,4, "Set:");

    _enable_interrupts();

    while(1)
    {
        True_voltage=getVoltage();

        if((True_voltage-pid.setPoint>=0.030)||(pid.setPoint-True_voltage>=0.030))
        {
            pidAdjust(True_voltage);
        }
        my_key();
        usrt_key();
        DispFloatat(72,4,pid.setPoint,2,3);//显示

    }
}

void init_uart_115200(void){
    UCA1CTL1 |= UCSWRST;
    UCA1CTL1 |= UCSSEL__SMCLK;     //选择ACLK，频率为32768Hz

    UCA1BR0 = 208;                           // 12MHz 115200 12M/115200=34.72222222222222
    UCA1BR1 = 0x0;
    UCA1MCTL = UCBRS_6+UCBRF_0;               // Modulation UCBRSx = 6

    //管脚复用
    P8SEL |= BIT2 | BIT3;
    P8DIR |= BIT2;              //P8.2输出
    P8DIR &= ~BIT3;             //P8.3输入

    UCA1CTL1 &= ~UCSWRST;

    UCA1IE |= UCRXIE | UCTXIE;                         //重新打开中断
    __enable_interrupt();
}

/* ======== USCI A0/B0 TX Interrupt Handler Generation ======== */
#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void){
    switch(__even_in_range(UCA1IV,4)){//user guide：36.4.14 UCAxIV Register
        case 0:break;
        case 2://接受数据
            recive=UCA1RXBUF;
            UCA1IFG |= UCTXIFG;

            break;                   //vector 2 : RXIFG
        case 4://发送数据
            //UCA1TXBUF = 0xAA;
            break;                    //vector 4 : TXIFG
        default: break;
    }

    __enable_interrupt();
    UCA1IE |= UCRXIE | UCTXIE;     //重新打开发送中断和接收中断
}

/******************************AD值读取函数**********************************/

float getVoltage()//可
{
    //测两个的时候为什么是反的
    unsigned int Value,Value2;
    Value2 = Write_SIP(0xf38b);           //AD数值     Conversion Register
    Voltage2=change_voltage(Value2,4.096);
    current=Voltage2/0.6052;

    suprotect(Voltage2);
    DispFloatat(80,2,current,1,3);//显示电流值

    Value = Write_SIP(0xe38b);           //AD数值     Conversion Register
    Voltage=change_voltage(Value,4.096);
    Voltage=Voltage*11.98-(0.01404*current*current*current*current-0.0781*current*current*current+0.1551*current*current-0.04532*current-0.09368);
    DispFloatat(72,0,Voltage,2,3);//显示电压值
    return Voltage;
}
/*****************************过流保护*********************************/
int c_i=0;
void suprotect(float vol)
{
    if(open==0)
    {
        if(vol>1.505)
            {
                c_i++;
                if(c_i>10)
                {
                    P8OUT |= BIT4;        //置高
                    __delay_cycles(120000000);//延时5S？
                    P8OUT &= ~BIT4;        //置低
                }
            }
        else
            c_i=0;

    }
}
/*****************************PID控制恒压*********************************/
void pidAdjust(float in_voltage)
{
  dealtV = PidDeltaCal(&pid,in_voltage);  //返回误差增量
  if((duty + dealtV) > 290)//65%
  {
      duty = 290;
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
//  TA0CCTL0 = /*OUTMOD_7+*/  CCIE;//捕获比较寄存器0输出，输出模式为2，同时使能定时器中断（CCR0单源中断），CCIE捕获比较寄存器的使能配置
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
  adjust_pid(&pid, 0, 0.0900, 0);//调整PID系数
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
int i=0;
void my_key()
{
    key_value= key();   /*scan Array_button, get the key value*/
            if(key_value!=0)
            {
                    if(i>1)//判断是0还是1
                        {
                           i=0;
                           if(num<=40.0&&num>=25.0)
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
                            if(pid.setPoint<40.0)
                                pid.setPoint+=1;//步进设定期望电压值
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
                              if(pid.setPoint>25.0)
                                  pid.setPoint-=1;//设定期望电压值
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
//                              pid.Proportion+=0.01;//调P
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
//                            pid.Integral+=0.001;//调I
                            key_value=0;
                            break;
                      default:break;
                    }
            }
}

//uart输入电压
void usrt_key()
{
            switch(recive)
            {
                case(107)://开
                        open=1;
                        P8OUT &= ~BIT4;        //置低
                       recive=0;
                     break;
               case(103)://关
                       open=0;
                        P8OUT |= BIT4;        //置高
                       recive=0;
                     break;
                case(54):
                        pid.setPoint=36;
                        recive=0;
                      break;
                case(53):
                        pid.setPoint=35;
                        recive=0;
                      break;
                case(52):
                        pid.setPoint=34;
                        recive=0;
                      break;
                case(51):
                        pid.setPoint=33;
                        recive=0;
                      break;
                case(50):
                      pid.setPoint=32;
                      recive=0;
                      break;
                case(49):
                      pid.setPoint=31;
                      recive=0;
                      break;
                case(48):
                      recive=0;
                      pid.setPoint=30;
                      break;
                case(47):
                      recive=0;
                      pid.setPoint=29;
                      break;
                case(46):
                      recive=0;
                      pid.setPoint=28;
                      break;
                case(45):
                      recive=0;
                      pid.setPoint=27;
                      break;
                case(44):
                      pid.setPoint=26;
                      recive=0;
                      break;
                case(43):
                      pid.setPoint=25;
                      recive=0;
                      break;
                case(55):
                      recive=0;
                      pid.setPoint=37;
                      break;
                case(56):
                      recive=0;
                      pid.setPoint=38;
                      break;
                case(57):
                      recive=0;
                      pid.setPoint=39;
                      break;
                case(58):
                      pid.setPoint=40;
                      recive=0;
                    break;
                default:break;
            }
}
