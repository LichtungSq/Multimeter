#include "msp430g2553.h"
#include "intrinsics.h"

//////////////////////////////
//          变量定义               //
//////////////////////////////

// 按键计数防抖
unsigned int key_cnt = 0;
// 计时器1
unsigned int clk_cnt = 0;
// 测量数字值(放大后的)
unsigned int measure = 0;
// 采样平均值
unsigned long result,newresult;
// 模式 1为直流 ，2为交流， 3为电容
int mode = 1;
// 档位
// 档位1 放大1倍(3V)
// 档位2 放大10倍(300mV)
// 档位3 放大100倍(30mV)
unsigned int range = 1;
// 频率值
unsigned int frequency = 0;
double frequency_c = 0;
// 用于定时器中断计数
unsigned int clock_isr = 0;
// 用于p1.1口上升沿计数
unsigned int gpio_isr1 = 0;
unsigned long gpio_isr2 = 0;
// 正负标志位
unsigned int sign_flag = 0; //1正，2负
// ADC采样数
int single = 0;
// 直流测量值加和
unsigned long sum = 0;
// 采样周期结束标志
char change_flag = 0;
// 交流平方和
float sqr_sum = 0;
float test;
int a=0,b=0;

int count=0;

#define uchar unsigned char
#define uint  unsigned int

#define THRESH_DOWN 280
#define THRESH_UP   3020
#define REF         3300
#define MAG_2       10
#define MAG_3       100
#define ln2         0.6931

/*********************************************************
*   接口定义：CS<--->P2.0; /WR<--->P2.1; DATA<--->P2.2    *
**********************************************************/

#define LCD_NON  20
#define LCD_POS  21
#define LCD_NEG  22
#define LCD_d    23
#define LCD_A    24
#define LCD_C    25
#define LCD_F    26
#define LCD_P    27

uchar table[32]=
{
    0xBE,0x06,0x7C,0x5E,0xC6,0xDA,0xFA,0x0E,0xFE,0xDE,  // 0 ~ 9
    0xBF,0x07,0x7D,0x5F,0xC7,0xDB,0xFB,0x0F,0xFF,0xDF,  // 0.~ 9.
    0x00,0x46,0x40,0x76,0xEE,0xB8,0xE8,0xEC             // non,'+','-','d','A','C','F','P'
};



#define BIAS     0X52   /*定义1 3 偏压4 背极*/
#define XTAL32   0X28   /*使用外部晶振*/
#define RC256    0X30   /*使用内部256KRC 振荡器*/
#define SYSEN    0X02   /*打开振荡发生器*/
#define LCDON    0X06   /*打开LCD*/
#define SYSDIS   0X00   /*关闭振荡发生器*/
#define LCDOFF   0X04   /*显示关闭*/
#define TONE4    0X80   /*设置BZ 输出频率为4K*/
#define TONEON   0X12   /*打开BZ 音频输出*/
#define TONEOFF  0X10   /*关闭BZ 音频输出*/
#define CLRWDT   0X1c   /*清零WDT*/
#define F1       0X40   /*WDT设置为4 秒溢出*/
#define IRQEN    0X10   /*IRQ 输出禁止*/
#define IRQDIS   0X00   /*IRQ 输出允许*/
#define WDTEN    0X0e   /*打开WDT*/
#define WDTDIS   0X0a   /*关闭WDT*/
#define TIMERDIS 0X08   /*关闭时基输出*/


void Start_spi(void);
void Delay(void);
void Delay1s(void);
void SENDCOMA(uchar com);
void SENDCOMB(uchar adr);
void SENDCOMC(uchar com);
void disp(void);
void DISP_ALL(void);
void SendByte(uchar dat);
void SendBit(uchar dat,uchar bitcnt);
unsigned int lcd[13];
int k=0;
const unsigned int lcd_init[13]={20,20,20,20,20,20,20,20,20,20,20,20,20};


void Port_Init(void)
{
    P2DIR |= 0x07;                              // p2.0,2.1,2.2 输出, LCD控制位

    P1SEL &= ~(BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);  // 将P1.0,P1.6,P1.7设置为通用I/O端口
    P1DIR |= BIT0+BIT3+BIT4+BIT5+BIT6+BIT7;     // 将P1.0,P1.6,P1.7设置输出
    P1OUT &= ~(BIT0+BIT3+BIT4+BIT5+BIT6+BIT7);  // 初始置零

    P1DIR &= ~(BIT3+BIT4);                      // P1.3 设置为输入
    P1REN |= BIT3;                              // P1.3 接上拉电阻
    P1OUT |= BIT3;

    P1SEL &= ~(BIT1+BIT2);
    P1DIR &= ~(BIT1+BIT2);
    P1OUT |= BIT1;
    P1IE  |= BIT1;
    P1IES |=BIT1;

    P1OUT |= BIT2;
    P1IE  |= BIT2;
    P1IES |=BIT2;
    P1IFG=0x00;


    ADC10CTL0 = SREF_2 + ADC10SHT_1 + ADC10ON;// + REFOUT;   //ADC10ON, interrupt disabled, external Reference
                                                 //Voltage SREF_0 + REFON + ADC10SHT_1 + ADC10ON; //
    ADC10CTL1 = INCH_5;                          // input A1 / P1.5
    ADC10AE0 |= BIT4 + BIT5;                     // P1.5 ADC option select P1.4 external ref
}

void TimerA0_Init(void)
{
    // Configure TimerA0
    TA0CTL = TASSEL_2 + MC_1 ;      // Source: SMCLK=1MHz, UP mode,
    TA0CCR0 = 1049;                 //1MHz时钟,计满1000次为1毫秒,999Hz校准成1049
    CCTL0 = CCIE;                   // CCR0 interrupt enabled
}

#pragma vector=PORT1_VECTOR         //p1.1中断服务程序
__interrupt void PORT1(void)
{
    if((P1IFG & BIT1)==BIT1)
    {
        gpio_isr1++;                     //上升沿每触发一次中断计数加1，通过统计单位时间内上升沿个数计算频率
        P1IFG &=~BIT1;
       // P1IFG &=~BIT2;
   }
   // P1IE  &= ~BIT1 ;
    if((P1IFG & BIT2)==BIT2)
    {
       gpio_isr2++;
       P1IFG &=~BIT2;
    }

}

// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR//1ms
__interrupt void Timer_A0 (void)
{

    clk_cnt++;
    clock_isr++;                    // 用于测量频率时计数
    if ((clock_isr == 3000) && (mode != 3) )          // 每3s记录一次1.1口中断计数值，即上升沿个数，采用3s可以使频率显示相对稳定并在变化时可快速反应
    {
        frequency = gpio_isr1 / 3;
        clock_isr = 0;
        gpio_isr1 = 0;
        frequency=(int)(0.9448*frequency+0.6349)+1;
        if((frequency==11)||(frequency==21)||(frequency==31)||(frequency==41)||(frequency==51)||(frequency==61)||(frequency==71)||(frequency==81)||(frequency==91))   frequency-=1;
    }

    if ((P1IN & BIT3) == 0 )        // P1.3按钮按下
    {
        key_cnt++;
        if (key_cnt == 50)
        {
            mode++;
            clk_cnt = 0;
            single = 0;
            sum = 0;
            change_flag = 0;
            if (mode == 4) mode = 1;
            result = 0;
            //key_cnt = 0;
        }
    }
    else key_cnt = 0;



    if(!change_flag)
    {
        switch (mode)
        {//mode1 直流
        case 1: sign_flag = ((P1IN & BIT1)>>1) + 1;         // P1.1取电压正负
                if(clk_cnt==100)
                {
                    clk_cnt = 0;
                    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
                    while (ADC10CTL1 & ADC10BUSY);          // Keep waiting till conversion complete
                    measure = (int)ADC10MEM;                // 读取ADC结果，存入变量measure

                    if(single < 30)
                    {
                        single++;//adc采样数
                        sum += measure;
                    }
                    else
                    {
                        change_flag = 1;      //采样结束标志
                    }
                }
                break;
        //交流
        case 2: sign_flag = 0;
                if(clk_cnt==3)   //采样时间 3ms
                {
                    clk_cnt = 0;
                    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
                    while (ADC10CTL1 & ADC10BUSY);          // Keep waiting till conversion complete
                    measure = (int)ADC10MEM;                // 读取ADC结果，存入变量measure

                    if(single < 1000)
                    {
                        single++;
                        sum = sum + ((long)measure * measure);
                    }
                    else
                    {
                        change_flag = 1;
                    }
                }
                break;
        case 3: if(clk_cnt==5000)
                {
                    frequency_c = gpio_isr2 / 5.0;// * 1.11702 - 462.518;
                    //if(frequency_c > 11600) frequency_c = frequency_c * 1.0631 + 32.535;
                    //else if(frequency_c > 5000) frequency_c = frequency_c * 1.058 - 414.765;
                    //if(frequency_c < 20) freqency_c +

                    clock_isr = 0;
                    gpio_isr2 = 0;
                    sign_flag = 0;
                    change_flag = 1;
                    clk_cnt = 0;
                }
                break;
        default: break;
        }
    }
    else clk_cnt = 0;

}

unsigned long Correct(unsigned long a)
{
    static unsigned long temp_0 = 0;
    static unsigned long temp_1 = 0;
    if(mode == 3)
    {  /* if(100000<a<1000000) a=a+20000;// a=0.9562*a+9353.4;
        if(10000<a<100000)  a=a-5000;
        if(a<10000)   a=a/10;*/
        //if(a<10000)     a=a/10;
        /*if (a < 30) a = a - 18;
        else if((a >= 30) && (a < 300)) a = 1.03 * a - 22.402;
        else if((a > 1500) && (a < 6000)) a = 0.879 * a + 139;
        else if((a > 10000) && (a < 50000)) a = 0.897 * a + 734;
        else if(a > 800000)
        {
            a = (temp_0 + temp_1 + a) / 3;
            temp_0 = temp_1;
            temp_1 = a;
        }*/
    }
    if(mode==2)
    {
        if(range==3)
        {    a=1.038*a-131.638;

           // a = 1.0231 * a -85.49;
            //if(a<1412) a=((a+42.449)*1.1063783)*1.0118-22.46;
            //else if(a>1412) a=((a+47.342)*1.1026696)*1.0118-22.46;
        }
        if(range==2)
        {
            a =0.9935* a +4.91;
            //if(a<348) a=((a-34.8098)*1.3704262)*1.02-3.478;
            //else if(348<a<501) a=((a+31.5413)*1.1310682)*1.02-3.478;
            //else if(a>501) a=((a+43.4967)*1.10625588)*1.02-3.478;
        }
        if(range==1)
        {
           // if(a<1000)
            a = 0.9912*a +13.7;
           // if(a>1000)   a=0.9941*a+11.7;
            //if(a<555) a=((a+33.42631)*1.120687)*1.02541-22.063;
            //else if(a>555) a=((a+37.79015)*1.11243367)*1.02541-22.063;
        }
    }

    if(mode==1)
    {
        if(range==3)
        {
            //if (sign_flag == 1) a =  0.9617 * a+507.79 ;
            if((sign_flag==2) && (a<1000))a =0.9786*a+29.371; //=0.9931*a+15 ;
            if((sign_flag==2) && (a>1000))a =0.9786*a+14.59 ;
        //else a=((((a-86.905)*1.003180)*1.06499+15.67781)*1.00037-1.04626)*0.9995+0.01563;

        }
        if(range==2)
        {
              if (sign_flag == 1)  a=0.9989*a+9.281;//{if(a<340)a =0.9868* a +52.648;  else  a=1.0041*a+36.292;}
              if(sign_flag==2) a = 0.978*a+16.567;//0.9815 * a +18.5;
        //else a=((((a-4.1245)*1.00168)*1.0639+15.22172)*1.00407+0.51289)*0.99852+0.55061;
        }
        if(range==1)
        {
            if (sign_flag == 1) a = 1.0006 * a + 15.1;
            //if((sign_flag==2) && (a<700)) a=(0.9856*a +0.0192)*1.002+13.9;
           // if((sign_flag==2) && (700<a<1600)) a = (0.9856*a +0.0192)*0.9975+18.8;
            if(sign_flag==2)
                {if(a<700)  a=0.9851*a+19.869;  //a=0.9876*a+19.919;
                  else  a=0.9975*a-1.9;}
                    //(0.9856 * a +0.0192)*1.0011+0.0161;
        //else (a=(((a+3.99533)*1.0029)*1.06262+12.7927)*1.00346+0.01407)*1.00343-0.98503;
        }

    }
    return a;
}
void Init_Devices(void)
{
    WDTCTL = WDTPW + WDTHOLD;     // Stop watchdog timer
    if (CALBC1_8MHZ ==0xFF || CALDCO_8MHZ == 0xFF)
    {
        while(1);            // If calibration constants erased, trap CPU!!
    }
    Port_Init();             //初始化I/O口
    TimerA0_Init();          //初始化定时器A0
    //_BIS_SR(GIE);            //开全局中断
    _EINT();
    //all peripherals are now initialized
}
/**********************************************************************
延时函数
***********************************************************************/
void Delay(void)
{
    uchar i;
    for(i=0;i<10;i++); /*用于调整CLK 脉冲宽度*/
}

void Delay1s(void){
    int i,j;
    for(i=0;i<1000;i++)
        for(j=0;j<1000;j++);
}

/*********************************************************************
发送命令函数A类
发送HT1621 命令时要先发送ID 值,及命令字用于设置HT1621.
*********************************************************************/
void SENDCOMA(uchar com)
{
    Start_spi();
    SendBit(0X80,4); /*发送设置命令ID=100 0*/
    SendByte(com); /*发送命令字*/
}

/*********************************************************************
发送命令函数B 类
发送HT1621 命令时要先发送ID 值,后发送要写入数据起始地址,
用于对RAM 写操作(调用此函数后即可发送数据) adr 是高5 位有效
*********************************************************************/
void SENDCOMB(uchar adr)
{
    Start_spi();
    SendBit(0XA0,4); /*发送写显示RAM 命令ID=101 0*/
    SendBit(adr,5); /*指定写入地址*/
}

/*********************************************************************
发送命令函数(C 类)
发送HT1621 命令时要先发送ID 值.然后发送C 类的命令字
*********************************************************************/
void SENDCOMC(uchar com)
{
    Start_spi();
    SendBit(0X90,4); /*发送命令ID=100 1*/
    SendByte(com); /*发送命令字*/
}

/******************************************************
全显示
******************************************************/
void DISP_ALL(void)
{
    SENDCOMA(BIAS); /*设置偏压,背极数*/
    SENDCOMA(RC256); /*设为内晶振256K*/
    SENDCOMA(SYSEN); /*启动振荡器*/

    SENDCOMA(LCDON); /*显示使能*/
    SENDCOMA(TIMERDIS); /*禁止时基输出*/
    disp(); /*输出全显数据*/
}

/**********************************************************************
向显示缓冲区填充,以最简便的形式显示数据
**********************************************************************/
void disp(void)
{
    uchar i,dat;
    SENDCOMB(0x00); /*把数据指针指回0 接着写入数据*/
    for(i=0;i<13;i++) /*写入12 字节数据*/
    {
        dat=lcd[i];
        SendByte(table[dat]); /*写入数据*/
    }
    SendByte(0x00);
}

/************************************************************************
名称: 发送数据位
************************************************************************/
void SendBit(uchar dat,uchar bitcnt)
{
    uchar i;
    for(i=0;i<bitcnt;i++)
    {
        if(( dat & 0X80 ) == 0)
            P2OUT&=~BIT2;//SDA_PORT = 0;
        else
            P2OUT|=BIT2;//SDA_PORT = 1; /*发送数据由高位到低位传送*/
        Delay();
        P2OUT|=BIT1;//CLK_PORT = 1; /*置时钟线为高通知被控器开始接收数位*/
        Delay();
        P2OUT&=~BIT1;//CLK_PORT = 0; /*钳住总线准备下一个数据位*/
        dat=dat<<1; /*0发送数据左移一位*/
    }
}

/**********************************************************************
起动ht1621
**********************************************************************/
void Start_spi(void)
{
    P2OUT|=BIT0;//CS_PORT=1;
    P2OUT|=BIT1;//CLK_PORT=1;
    P2OUT|=BIT2;//SDA_PORT=1;
    Delay();
    P2OUT&=~BIT0;//CS_PORT=0;
    P2OUT&=~BIT0;//CS_PORT=0;
    P2OUT&=~BIT1;//CLK_PORT=0;
    P2OUT&=~BIT1;//CLK_PORT=0;
}

/************************************************************************
发送字节函数,向ht1621 发送字节数据数据单元dat
************************************************************************/
void SendByte(uchar dat)
{
    SendBit(dat,8); /*发送字节*/
}

/************************************************************************
刷新显示
************************************************************************/
void FlushLcd(void)
{
    int i;
    for(i = 12; i > 0; i--)
    {
       lcd[i] = lcd_init[i];
    }


    if (sign_flag == 0) lcd[6] = LCD_NON;
    else if (sign_flag == 1) lcd[6] = LCD_POS;
    else if (sign_flag == 2) lcd[6] = LCD_NEG;
    lcd[8] = result / 1000;
    lcd[9] = result % 1000 / 100;
    lcd[10] = result % 1000 % 100 / 10;
    lcd[11] = result % 1000 % 100 % 10;

    //if()
    switch(range)
    {
    case 1: lcd[8] += 10; break;
    case 2: lcd[10] +=10; break;
    case 3: lcd[9] += 10; break;
    }

    switch (mode)
    {
    case 1: lcd[0] = LCD_d;
            lcd[1] = LCD_C;
            break;
    case 2: lcd[0] = LCD_A;
            lcd[1] = LCD_C;
            lcd[2] = LCD_NON;
            lcd[3] = LCD_F;

            lcd[4] = frequency / 100;
            lcd[5] = frequency % 100 / 10;
            lcd[6] = frequency % 100 % 10;
            break;
    case 3: lcd[0] = LCD_C;
            //lcd[3] = LCD_F;
            lcd[3] = result / 1000000;
            lcd[4] = result % 1000000 / 100000;
            lcd[5] = result % 100000 / 10000;
            lcd[6] = result % 10000 / 1000;
            lcd[7] = result % 1000 / 100;
            lcd[8] = result % 100 / 10;
            lcd[9] = result % 10;
            lcd[10] = LCD_P;
            lcd[11] = LCD_F;
            break;
    default: mode = 1; break;
    }
    DISP_ALL();
}

/************************************************************************

/************************************************************************
牛顿法开方函数
************************************************************************/
float abs_f(float x)
{
    float i;
    i=x;
    if(i<0)
    i=-i;
    else
    i=i;
    return i;
}

float SqrtByNewton(float x)
{
    float val = x;//最终
    float last;//保存上一个计算的值
    do
    {
        last = val;
        val =(val + x/val) / 2;
    }
    while(abs_f(val-last) > 0.01);   //精度控制
    return val;
}
/************************************************************************
调整放大倍数
************************************************************************/
void ChangeRange(int measure)
{
    static int approx = 0;
    if(mode == 1)
    {
        if (measure < (THRESH_DOWN + approx+20))
        {
            if(range < 3) range++;
            clk_cnt = 0;
            single = 0;
            sum = 0;
            //change_flag = 0;
            approx = 0;
        }
        else if (measure > (THRESH_UP- approx))
        {
            if(range > 1) range--;
            clk_cnt = 0;
            single = 0;
            sum = 0;
            //change_flag = 0;
            approx = 0;
        }
        else
        {
            if(approx < 30) approx += 5;
        }
    }
    else if(mode == 2)
    {
        if (measure < (THRESH_DOWN + approx-80 ))
        {
            if(range < 3) range++;
            clk_cnt = 0;
            single = 0;
            sum = 0;
            //change_flag = 0;
            approx = 0;
        }
        else if (measure > (THRESH_UP - approx - 800))
        {
            if(range > 1) range--;
            clk_cnt = 0;
            single = 0;
            sum = 0;
            //change_flag = 0;
            approx = 0;
        }
        else
        {
            if(approx < 20) approx += 5;
        }
    }
     switch (range)
    {
    case 1: P1OUT &= ~(BIT6+BIT7); break;   //都写0   1倍
    case 2: P1OUT |= BIT6; P1OUT &= ~BIT7; break;  //1.6写1   10倍
    case 3: P1OUT &= ~BIT6;P1OUT |= BIT7; break;  //1.7写1   100倍
    default : P1OUT &= ~(BIT6+BIT7); break;
    }
}


void main(void)

   {
    Init_Devices( );

    while(1)
    {
        //test = SqrtByNewton(9830931);
        count++;
       if(change_flag == 1)
        {
            switch (mode)
            {
            case 1: result = (float) sum / single / 1023 * REF;
                    break;
            case 2: sqr_sum = (float) sum / single;
                    //test = Sqrt(9830931);
                    result = SqrtByNewton(sqr_sum) / 1023 * REF;
                    break;
            case 3: result = 1000000000 / (3 * 560 * ln2 * frequency_c);
                    break;
            default: break;
            }
            single = 0;
            sum = 0;
            result = Correct(result);
            ChangeRange(result);
            change_flag = 0;
        }

       FlushLcd();
    }
}