/********************************************/
// TC Original Version
// Modify : Change processing, processing to 
//		stop switching pulses of timer interrupts 
//		of 300bps communication signals
// Modify: 2025.07.28
/********************************************/

//#include <p24FJ32GA002.h>
#include <p24FJ16GA002.h>
#include <xc.h>        // PIC24F ??????
#include <libpic30.h>  // Nop() ???
//#include "i2c.h"

//_CONFIG2( POSCMOD_NONE & ALTI2C_OFF & LPOL_ON & IOL1WAY_ON & OSCIOFNC_ON & FCKSM_CSECMD & FNOSC_FRC & WDTWIN_WDTWIN50 & PWMPIN_OFF & PWMLOCK_ON & IESO_OFF )
//_CONFIG1( WDTPOST_PS32768 & WDTPRE_PR128 & PLLKEN_OFF & WINDIS_ON & FWDTEN_OFF & ICS_PGD1 & HPOL_ON & GWRP_OFF & GCP_ON )

_CONFIG2( POSCMOD_NONE & I2C1SEL_PRI & IOL1WAY_OFF & OSCIOFNC_ON & FCKSM_CSECMD & FNOSC_FRC & SOSCSEL_SOSC & WUTSEL_LEG & IESO_OFF )
_CONFIG1( WDTPS_PS32768 & FWPSA_PR128 & WINDIS_ON & FWDTEN_OFF & ICS_PGx1 & GWRP_OFF & GCP_OFF & JTAGEN_OFF )
//set GCP_ON


#include <math.h>

#define ON 1
#define OFF 0
#define LED_RED _LATA2
#define LED_GRN _LATA3
#define PORT_RX _RB4
#define PORT_TX _LATA4
#define PORT_CUTTER _RA1
#define PORT_BOR _RB1
#define tens_ref 105
#define pwm_zero OC1RS
#define pwm_vol OC2RS
#define data_total 200
#define pid_scale 1000 //1600时,110T-96-262,日本试褋E600
/*************************************************************
Low side driver table is as below.  In this StateLoTable,
the Low side driver is PWM while the high side driver is
either on or off.  This table is used in this exercise
*************************************************************/
//const unsigned int StateLoTable[8] = {0x0000, 0x0210, 0x2004, 0x0204, 0x0801, 0x0810, 0x2001, 0x2A00};//Version2.0,硬件变竵E男?
//const unsigned int StateLoTable[8] = {0x0000, 0x0021, 0x0018, 0x0009, 0x0006, 0x0024, 0x0012, 0x0000};//电机反转，眮E-6改为6-1，此段取消了底端PWM。
//const unsigned int StateLoTable[8] = {0x0000, 0x2001, 0x0810, 0x0801, 0x0204, 0x2004, 0x0210, 0x2A00};//电机反转，眮E-6改为6-1，底端PWM
//const unsigned int StateLoTable[8] = {0x0000, 0x0120, 0x1008, 0x0108, 0x0402, 0x0420, 0x1002, 0x0000};//高段PWM

const unsigned int StateLoTable[8] = {0x0000, 0x8400, 0x1800, 0x9000, 0x6000, 0x2400, 0x4800, 0x0000};//电机换向眮E

volatile float Kp;
volatile float Ki;
volatile float Kd;
volatile signed int Ka;

// ファイル先頭付近（既存の volatile 定義のすぐ下など）に追加
volatile uint8_t tx_enable = 0;    // 0: 出力抑制, 1: 送信中

volatile unsigned char setlen[3];		//设定的长度（对藖E屑醴ㄊ迪殖ざ瓤刂疲?
volatile signed char len_1[3];		//剩余的长度,计算发生负数
volatile signed char lastlen[3];		//长度（已经菌洹的长度
volatile unsigned char settens;
volatile unsigned int tens_set;			//设定张力值
volatile unsigned char txtens[2];			//发送的张力值
volatile signed char tmp_1;
volatile signed char tmp_2;

volatile unsigned int tens_desired;		//运算用设定张力值
volatile unsigned int tens_actual;		//存放张力平均值
//volatile unsigned int tens_sample[16];	//A/D结果存放的变量
volatile unsigned int tens_sum;			//存放16次采集的AD值的总和
volatile unsigned int tens_zero;		    //存放零点张力值
volatile unsigned int tens_temp;
volatile signed int tens_deviation;			//误阐虻
volatile signed int last_deviation;		//前次误瞾E
volatile signed int prev_deviation;		//上次误瞾E
volatile float pwm_add;
volatile float pwm_out_temp;
volatile signed int pwm_out;
volatile signed long pwm_check;    //just for checking

volatile unsigned int rx_timer;		//接收时间的计数苼E
volatile unsigned int rx_bitcntr;
volatile unsigned int rx_bytecntr;

volatile unsigned int rx_waitcntr;
volatile unsigned char rx_buffer_1;
volatile unsigned char rx_buffer[6];			//接收数据莵E
volatile unsigned int tx_timer;			//发送时间的计数苼E
volatile unsigned int tx_cntr;
volatile unsigned int tx_bitcntr;
volatile unsigned int tx_bytecntr;
volatile signed char txreg_1;
volatile unsigned int stop_delay;			//电机停止时紒E
volatile unsigned int cutter_timer;		//剪刀复位计数苼E

volatile unsigned int check_timer;			//端口查询时间计数苼E
volatile signed int pulse_cntr;				//丝长计数,在减法运算时需要负数

volatile unsigned int display_timer;				//LED闪烁延迟计数
volatile unsigned int ledcntr;				//LED闪烁延迟计数
volatile unsigned int HallValue;
volatile unsigned int eeprom_address;	//保存在eeprom中的个数
volatile unsigned char zero_delay;
//调试
volatile signed int tens_addt[data_total];//记录不同时间段的补偿值
//2014.8.24改为190个数据,200个数据为极限,再增加变量时，reset的会产生代陙E
volatile signed int tens_complast[data_total];//记录不同时间段的张力偏瞾E
volatile signed int tens_comp;
volatile signed int tens_addt_temp;
volatile signed int tens_add;
volatile signed char special_error_set;
volatile unsigned char running_timer;//特殊菌锩定时苼E
volatile unsigned char special_cycletimer;
volatile unsigned int speed_cntr;
volatile unsigned int speed_cntr1;
volatile unsigned char speed_deltalast;
volatile unsigned char speed_delta;
volatile unsigned char speed_minlast;
volatile unsigned char speed_min;
volatile unsigned char speed_max;
volatile unsigned char cnt_FIR;
volatile unsigned int speed_min_sum;
volatile unsigned char speed_min_avg;//记录平均臁磁极数
volatile unsigned char speed_min_array[16];//记录先前数据
volatile unsigned char start_delay;
volatile unsigned int velocity;
volatile unsigned char life[3];//备用，记录寿脕E

unsigned int rampup_timer;//启动延迟时间，启动+10g


typedef union
{
	unsigned char byte;
	struct
	{
		unsigned buf0 : 1;
		unsigned buf1 : 1;
		unsigned buf2 : 1;
		unsigned buf3 : 1;
		unsigned buf4 : 1;
		unsigned buf5 : 1;
		unsigned buf6 : 1;
		unsigned buf7 : 1;
	}BIT;
}rxbuffer;


volatile union 
{
	int twobyte;
	struct
	{
		unsigned motor_running :1;	    //电机状态位
		unsigned tx_300 :1;			//接收信号处历蛠E疚?
		unsigned rx_300 :1;			//AD转换处历蛠E疚?
		unsigned port_check :1;		//端口查询处历蛠E疚?
		unsigned ad_conv :1;			//
		unsigned rx_start :1;			//接收信号眮E疚?
		unsigned display :1;			//LED显示眮E疚?
		unsigned lowvolt :1;			//低电压循环眮E疚?
		unsigned speed_drop :1;		//used in future for brake
		unsigned speed_count :1;		//速度计算眮E疚?
		unsigned pulse_count :1;		//丝长计算眮E疚?
		unsigned cutter_down :1;		//剪刀动作眮E疚?
		unsigned adj_start :1;			//张力校准眮E疚?
		unsigned zero_error :1;
		unsigned special_start :1;		//调零蛠E?
		unsigned speedlow :1;
	}BIT;	
}flag1;

volatile union
{
	char byte;
	struct
	{
		unsigned l_0 :1;
		unsigned l_1 :1;
		unsigned l_2 :1;
		unsigned command :1;			//传送字节中的状态位（柄喵当前字节是否是脕E故鞘荩?
		unsigned zero_end :1;
		unsigned no_tension :1;
		unsigned  :1;
		unsigned  :1;
	}BIT;
}flag2;

volatile union
{
	char byte;
	struct
	{
		unsigned wind_end :1;
		unsigned wind_start :1;		
		unsigned special_wind :1;
		unsigned special_signal :1;
		unsigned  :1;
		unsigned  :1;
		unsigned  :1;
		unsigned  :1;
	}BIT;
}flag3;


/****************************************************************************************/
/*                                     各子函数                                         */
/****************************************************************************************/
void tension_compensate(void);
void check_port(void);
void GetSpeed(void);
void pulse_count(void);
void led_mode(unsigned char led_chose);
void led_display(void);
void do_command(unsigned char a);
void do_reset(void);
void do_sensadj(void);
void rxsend(void);
void rxadj(void);
void rxreset(void);
void rxerror(void);
void auto_zero(void);
void table_rx(unsigned char b);
void data_rx(void);
void tx_cntrset(void);
void tx_300_1(void);
void tx_300_2(void);
void tx_300_3(void);
void data_tx(void);
void motor_control(void);
void data_backup1(void);
void data_backup2(void);
void Delay(unsigned int DelayTime);

void data_load1(void);
//void data_load2(void);
void StartI2C(void);
void RestartI2C(void);
void StopI2C(void);
void WriteI2C(unsigned char byte);
void IdleI2C(void);
//unsigned int LDByteWriteI2C(unsigned char LowAdd, unsigned char data);
//unsigned int OneByteReadI2C(unsigned char Address);
unsigned int ACKStatus(void);
void NotAckI2C(void);
void AckI2C(void);
unsigned int getI2C(void);
//unsigned int EEAckPolling(unsigned char control);


void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)//TMR1每50uS进葋E酥卸?
{
	IFS0bits.T1IF = 0; //Clear Timer1 interrupt flag
#if 0	// bit Pluse output stop!!
	if(tx_timer!=0)
	{
		tx_timer--;//125*13.2uS=1.65mS处历罨次
	}
	else
	{
		tx_timer=32;
		flag1.BIT.tx_300=1;		
	}
#endif
	// 送信中のみ、300bps 用フラグを立てる
	if (tx_enable) {
		if (tx_timer != 0) {
			tx_timer--;
		} else {
			tx_timer = 32;            // ≒1.65ms
			flag1.BIT.tx_300 = 1;     // 次の data_tx() 呼び出しを許可
		}
	}
	// 以下 rx_timer/display_timer 等はそのまま…

	if(rx_timer!=0)
	{
		 rx_timer--;//250*13.2uS=3.3mS处历罨次
	}
	else
	{
		rx_timer=65;
		flag1.BIT.rx_300=1;		
	}
	if(check_timer != 0) 
	{
		check_timer --;;//19*13.2uS=250uS处历罨次
	}
	else
	{
		check_timer=4;
		flag1.BIT.port_check=1;		
	}
	if(display_timer!=0) 
	{
		display_timer--;//(999+1)*50us=50mS处历罨次
	}
	else
	{
		display_timer=999;
		flag1.BIT.display=1;

		speed_cntr1 = speed_cntr;
		speed_cntr = 0;
		flag1.BIT.speed_count = 1;
	}
//PORT_CUTTER^=1;//调试用
}


void __attribute__((__interrupt__, auto_psv)) _T3Interrupt( void )//period =31.5us
{
	IFS0bits.T3IF = 0;// Clear Timer 3 interrupt flag
	AD1CON1bits.SAMP = 1; // start sampling then after 31Tad go to conversion
//PORT_CUTTER^=1;//调试用
}

void __attribute__((__interrupt__, auto_psv)) _CNInterrupt(void)//基于6000转电机，阐位秮E00us进葋E淮沃卸?
{
	//int k;
	IFS1bits.CNIF = 0;// clear flag
	HallValue = PORTB & 0x00E0;// mask RB5,6,7
	HallValue = HallValue >> 5;// shift right 5 times
	PORTB &= 0x03FF;
	asm("nop");
	PORTB |= StateLoTable[HallValue];// Load the overide control register

	speed_cntr++;
	flag1.BIT.pulse_count=1;
}
void para_initial(void)
{
	unsigned int i;
	for(i = 0; i < data_total; i++)
	{
		tens_complast[i] = 0;
		tens_addt[i] = 0;
	}

	Delay(60000);	//EEPROM启动延迟
	data_load1();	//读取
	special_error_set = setlen[0];

	tension_compensate();
	tens_desired = tens_set;
	flag1.twobyte = 0x0000;
	flag2.byte = 0x00;
	pulse_cntr = 143;
	ledcntr = 5;
	zero_delay = 2;
	running_timer=0;
	//pid_scale = 800;
	//Kp = 135;
	//Ki = 210;
	//Kd = 90;
	//Ka = 0;
	//pwm_out_temp1 = 0;
	pwm_out_temp = 0;
	pwm_out = 0;// Write the duty cycle for the second PWM pulse
	cnt_FIR = 0;
	speed_min_sum = 0;
	speed_min_avg = 0;//记录平均臁磁极数
	cnt_FIR = 0;
	stop_delay = 1000;
	start_delay = 0;
	rampup_timer = 0;
}
int main(void)
{

__builtin_write_OSCCONH(0x01);
__builtin_write_OSCCONL(0x01);
// Wait for Clock switch to occur
while (OSCCONbits.COSC != 0b001)
// Wait for PLL to lock
while(OSCCONbits.LOCK != 1)

loop:
CLKDIV 	= 0x0000;	// FRC-Postscaler = 1:1 => 8MHz
LATA = 0x0000;//写 LATx 寄存器觼E?PORTx 寄存器的效果相同
LATB = 0x0000;//写 LATx 寄存器觼E?PORTx 寄存器的效果相同
//PORT_CUTTER = 0;//防止初始化以后剪刀误动讈E
TRISA = 0x0001;// Port A : RA0=Inputs, RA1,RA2,RA3,RA4=output
TRISB = 0x03F3;// Port B : RB0,RB1,RB4,RB5,RB6,RB7=Inputs, all other Output
//CNPU1 = 0x0030;//CN4,CN5 PULL-UP
/******************** Initialize CN interrupt *********************/
//CNEN1 = 0x0000;
CNEN2 = 0x0980;// CN23,24,27 enabled
//CNPU1 = 0x0000;// disable internal pullups
//CNPU2 = 0x0000;// disable internal pullups
/******************** Initialize Output Compare Module ************/
OC1R = 0;// Write the duty cycle for the first PWM pulse
OC1RS = 0;// Write the duty cycle for the second PWM pulse,PWM input
OC1CON = 0x0006;
//OC1CONbits.OCTSEL = 0;// Select Timer 2 as output compare time base
//OC1CONbits.OCM = 0b110;// Select the Output Compare mode

OC2R = 0;// Write the duty cycle for the first PWM pulse
OC2RS = 0;// Write the duty cycle for the second PWM pulse,PWM input
OC2CON = 0x0006;

RPOR1 = 0x1213;//OC1 is assigned to RP3,OC2 is assigned to RP2	

//OC1RS = 120;//debug
//OC2RS =220;		//update this in MOTOR FUNCTION
/******************** Initialize Timer2 ***************************/
T1CON = 0x0000;
//T1CONbits.TON = 0;// Disable Timer
//T1CONbits.TCS = 0;// Select internal instruction cycle clock
//T1CONbits.TGATE = 0;// Disable Gated Timer mode
//T1CONbits.TCKPS = 0b00;// Select 1:1 Prescaler
//T1CONbits.TSIDL = 0;// Continues module operation in Idle mode
TMR1 = 0x0000;// Clear timer register
PR1 = 799;// Load the period value, period =(799+1)*Tcy=50us, Tcy=0.0625us
/******************** Initialize Timer1 ***************************/
T2CON = 0x0000;
//T2CONbits.TON = 0;// Disable Timer
//T2CONbits.T32 = 0;// Timer2 and Timer3 act as two 16-bit timers
//T2CONbits.TCS = 0;// Select internal instruction cycle clock
//T2CONbits.TGATE = 0;// Disable Gated Timer mode
//T2CONbits.TCKPS = 0b00;// Select 1:1 Prescaler
//T2CONbits.TSIDL = 0;// Continues module operation in Idle mode
TMR2 = 0x0000;// Clear timer register
PR2 = 499;// Load the period value, period =(499+1)*Tcy=31.25us
/******************** Initialize Timer3 ***************************/
T3CON = 0x0010;// Select 1:8 Prescaler
TMR3 = 0x0000;// Clear timer register
//PR3 = 62;// Load the period value, ADC period =(PR3+1)*Tcy*8, 16 samples = 500us
PR3 = 124;// Load the period value, ADC period =(PR3+1)*Tcy*8, 16 samples = 1ms
/******************** intializes ADC ******************************/
AD1PCFG = 0xFFFE;// RA0 => analog, all other PORT = Digital;
AD1CSSL = 0x0001;// scan AN0
AD1CHS = 0x0000;// RA0/AN0 => CH0
AD1CON1 = 0x00E0;// SSRC bit = 111, auto converting
AD1CON2 = 0x003C;// Channel A, 16th convert sequence, Converts CH0
AD1CON3 = 0x1F03; // auto Sample time = 31Tad, Tad = 4 Tcy, inverting = 12 Tad，total = 43Tad =11.5us
/******************** intializes I2C ******************************/
I2C1BRG = 0x0012; 	//First set the I2C(1) BRG Baud Rate.计算值为10，改成12留余量
I2C1CON = 0x1200;	//I2C peripheral for Master Mode, No Slew Rate Control, peripheral switched off.
I2C1RCV = 0x0000;
I2C1TRN = 0x0000;
I2C1CON = 0x9200;//enable the peripheral
/******************** Set interrupt priority **********************/
IPC4bits.CNIP = 5;
IPC0bits.T1IP = 2; // Set Timer1 Interrupt Priority Level
//IPC1bits.T2IP = 2;// Set Timer 2 Interrupt Priority Level
IPC2bits.T3IP = 2;// Set Timer 3 Interrupt Priority Level
//IPC3bits.AD1IP = 4;
//IPC7bits.T5IP = 1;// Set Timer 5 Interrupt Priority Level
//IPC14bits.PWM1IP = 1;
/*********************** Enable interrupt *************************/
IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
IEC0bits.T1IE = 1; // Enable Timer1 interrupt
T1CONbits.TON = 1; // Start Timer,50us计时打开
//IFS0bits.T2IF = 0;// Clear Timer 2 Interrupt Flag
//IEC0bits.T2IE = 1;// Enable Timer 2 interrupt
T2CONbits.TON = 1;// Start Timer2
IFS0bits.T3IF = 0;// Clear Timer 3 Interrupt Flag
IEC0bits.T3IE = 1;// Enable Timer 3 interrupt
T3CONbits.TON = 1;// Start Timer3
//IFS1bits.T5IF = 0;// Clear Timer 5 Interrupt Flag
//T5CONbits.TON = 1;// Start Timer5

IFS0bits.AD1IF = 0;
//IEC0bits.AD1IE = 1;//Disable ADIE
AD1CON1bits.ADON = 1;// turn ADC ON
IFS1bits.CNIF = 0;// Reset CN interrupt
IEC1bits.CNIE = 1;// enable CN interrupt
//IFS3bits.PWM1IF = 0;
//IEC3bits.PWM1IE = 1;
//PWM1CON1 = 0x0777;// complementary mode, enable PWM output
/***************************** 参数初始化 **************************/
para_initial();
while(1)
{
	if(flag1.BIT.port_check==1)					//250uS一次的端口紒E丒
	{
		flag1.BIT.port_check=0;
		check_port();	
	}
	if(flag1.BIT.lowvolt==1)						//如果电压低就进葋E缪共檠?
	{
		while(!PORT_BOR)						//等待电压恢复
		goto loop;				
	}
	if(flag1.BIT.ad_conv==1)						//AD计藖E输出值改写
	{
		//PORT_CUTTER=1;
		flag1.BIT.ad_conv=0;
		motor_control();//debug
		//PORT_CUTTER=0;
	}
	if(IFS0bits.AD1IF == 1)
	{
		IFS0bits.AD1IF = 0;// Clear AD interrupt flag

		tens_sum = 0;
		tens_sum = tens_sum + ADC1BUF0;
		tens_sum = tens_sum + ADC1BUF1;
		tens_sum = tens_sum + ADC1BUF2;
		tens_sum = tens_sum + ADC1BUF3;
		tens_sum = tens_sum + ADC1BUF4;
		tens_sum = tens_sum + ADC1BUF5;
		tens_sum = tens_sum + ADC1BUF6;
		tens_sum = tens_sum + ADC1BUF7;
		tens_sum = tens_sum + ADC1BUF8;
		tens_sum = tens_sum + ADC1BUF9;
		tens_sum = tens_sum + ADC1BUFA;
		tens_sum = tens_sum + ADC1BUFB;
		tens_sum = tens_sum + ADC1BUFC;
		tens_sum = tens_sum + ADC1BUFD;
		tens_sum = tens_sum + ADC1BUFE;
		tens_sum = tens_sum + ADC1BUFF;

		tens_actual = tens_sum >>4;
		if(tens_actual >= tens_zero)
		{
			tens_temp = tens_actual - tens_zero;
		}
		else
		{
			tens_temp = 0;
		}
		flag1.BIT.ad_conv = 1;
		//PORT_CUTTER^=1;//调试用
	}

	if(flag1.BIT.rx_300==1)						//3.3mS一次的数据接收
	{
		flag1.BIT.rx_300=0;
		data_rx();
	}
	if(flag1.BIT.tx_300==1)						//1.65mS一次的数据发送
	{
		flag1.BIT.tx_300=0;
		data_tx();
	}
	if(flag1.BIT.display==1)					//50ms一次的LED指示
	{
		flag1.BIT.display=0;
		led_display();
		//剪刀延时程衼E
		if(flag1.BIT.cutter_down==1)
		{
			PORT_CUTTER=1;								//剪刀动讈E		
			cutter_timer--;
			if(cutter_timer==0)
			{
				flag1.BIT.cutter_down=0;
				PORT_CUTTER=0;
			}
		}
	}
	if(flag1.BIT.pulse_count==1)					//线长计藖E
	{
        flag1.BIT.pulse_count=0;
		pulse_count();
	}
	if(flag1.BIT.speed_count==1)
	{
        flag1.BIT.speed_count=0;
        GetSpeed();
	}
}
// end of while (1)
}// end of main

/**************************************端口查询函数****************************************/
/******************************************************************************************/
void check_port(void)
{
	/*if(PORT_BOR == 0)						//电压偏低
	{
		INTCON1bits.NSTDIS = 0;				//关中断
		flag1.BIT.lowvolt = 1;				//置低电压眮E疚?
		P1DC1 = 0;// set PWM 1, 2 and 3 to 0
		P1DC2 = 0;
		P1DC3 = 0;
		LED_RED = 1;								//灭灯,测试，灯亮
		LED_GRN = 1;
		//AD1CON1bits.ADON = 0;			//关闭AD
		data_backup1();
		INTCON1bits.NSTDIS = 1;			//开中断	
		
	}
	else */if(flag1.BIT.rx_start == 0)	//电压正常,紒E橥ㄑ?
	{
		if(PORT_RX == 0)				//如果RA4有低电平
		{
			rx_timer = 33;				//过1.65mS再紒E丒
			rx_bitcntr = 0;
			rx_waitcntr = 17;
			flag1.BIT.rx_300 = 0;
			flag1.BIT.rx_start = 1;		
		}
	}

}
/************************************************************************
GetSpeed, determins the exact speed of the motor by using the value in
TMR3 for every mechanical cycle. 
测试发现：以下程序使用时间为8us(电机未动)，10us(电机运转普通緛E
*************************************************************************/
void GetSpeed(void)
{
	if ((speed_cntr1 > 3) && (flag1.BIT.cutter_down == 0))// 600RPM:12pu/50ms;24pu/100ms//原5个磁极 0902//
	{
		flag1.BIT.motor_running = 1;
		flag3.BIT.wind_start = 1;

/**********************************20200322 updated************************************/
		//speed check for state
		if((speed_cntr1 + 10) < speed_min_avg) //敾抐懍搙壓崀丆掆婘敾掕.
		{
			flag1.BIT.speed_drop = 1;
		}
		else if(speed_cntr1 < (speed_min_avg + 4))//敾抐懍搙惀斲愙嬤?愜揰.
		{
			flag1.BIT.speedlow = 1;
		}
		else
		{
			flag1.BIT.speedlow = 0;
			flag1.BIT.speed_drop = 0;
		}
/**************************************************************************************/

		//每50ms循环一次,200个循环为10S，判断1次畜值和臁值，竵E滤俣炔?
		if(++special_cycletimer == 200)
		{
			special_cycletimer = 0;
			//speed_deltaprev = speed_deltalast;
			speed_deltalast = speed_delta;//赋值，竵E律弦淮嗡俣炔?
			speed_minlast = speed_min;//保存当前淹速度，下一次10脕E芷谑褂?
			speed_delta = speed_max - speed_min;//竵E麓舜嗡俣炔?

			cnt_FIR++;
			cnt_FIR = cnt_FIR & 0x07;
			speed_min_sum = speed_min_sum + speed_min - speed_min_array[cnt_FIR];
			speed_min_array[cnt_FIR] = speed_min;

			if(start_delay > 10)//延迟10个周期
			{
				//start_delay = 10;//使变量保持不眮E
				speed_min_avg = (unsigned char)(speed_min_sum >> 3);//取平均臁磁极瞾E
			}
			else //运转初期，淹速度取10脕E谧椭担笕∑骄?
			{
				start_delay++;
				speed_min_avg = speed_minlast;
			}

			speed_max=0;
			speed_min=255;
		}
		else
		{
			if(speed_max < speed_cntr1)
				{speed_max = speed_cntr1;}
			if(speed_min > speed_cntr1)
				{speed_min = speed_cntr1;}
		}

	//特殊运行判断
	//条件1：速度畜值-臁值<=5脉硜E
	//条件2：速度低于600转结束特殊緛E
	//条件3：超时结束特殊緛E
	//条件4：特殊菌怍动延迟
		if((speed_delta > special_error_set) && (speed_deltalast > special_error_set) && (flag3.BIT.special_wind == 1))//3次判断，起延迟作用
		{
			flag1.BIT.special_start = 1;
		}
		else
		{
			flag1.BIT.special_start = 0;
		}
	//stop_delay = 1000;
    }
	else //if (speed_timer > 0)//速度计数器在3000以内，说明电机在运转，切换进葋E缁俗刺?
	{
        flag1.BIT.motor_running = 0;				//开始驱动电机时眮E疚晃?
		//pwm_out_temp = 0;
		pwm_out = 0;
		pwm_add = 0;
		flag1.BIT.special_start = 0;
		special_cycletimer = 0;//保持
		speed_max=0;
		speed_min=255;
		speed_delta = 0;
		speed_deltalast = 0;
		//speed_deltaprev = 0;
		speed_minlast = 0;

		cnt_FIR++;
		cnt_FIR = cnt_FIR & 0x07;
		speed_min_array[cnt_FIR] = 0;//清楚之前保存的臁磁极阐楷防止由于运算而导致的清羴E

		speed_min_sum = 0;
		speed_min_avg = 0;//记录平均臁磁极数
		start_delay = 0;//启动延迟

		tens_deviation = 0;
		prev_deviation = 0; 
		last_deviation = 0;
		//stop_delay = 0;

		rampup_timer = 8000;
	}

//特殊菌喂偿根据时间来定，
//每50ms紒E庖淮嗡俣龋∷俣茸偷阕魑鹗嫉悖?0ms取张力偏阐虻，计葋E娲⑵鳌?
//共200个存储器，10脕E奔洹?
//根据下一周期的时间点，取出对应的张力偏阐虻，做PI运算，作为补偿值，加葋E怂恪?
//tens_error_s[special_timer]记录上一次该时间点的偏瞾E
//tens_addt[special_timer]记录补偿值，增量式补偿法
	if(flag1.BIT.special_start == 1)
	{
		running_timer ++;
		if(running_timer>data_total)
		{
			running_timer=data_total;
		}

		if(flag1.BIT.speedlow == 1)//淹祦E
		{
			//PORT_CUTTER^=1;//调试用
			running_timer = 0;
		}
		tens_addt_temp = tens_comp + tens_complast[running_timer+1] - tens_complast[running_timer];//PID,张力补偿值增量式PI算法
		tens_addt[running_timer] += (tens_addt_temp / 8);
		tens_complast[running_timer] = tens_comp;
		//tens_add = tens_addt[running_timer];
		tens_add = tens_addt[running_timer];//特殊菌圬制有延迟，当补偿了以后，需要0.5脕E拍芷鹱饔?于是提前补偿200ms以后的张力偏瞾E
	}

	//velocity = speed_cntr1 * 25; //48相位/圈, 速度=50ms脉冲数*20*60/48，20次为1脕E?0脕E分钟

}
/***************************************Pulse Count丒**********************************/
/****************************************************************************************/
void pulse_count(void)
{
	if(flag3.BIT.wind_end == 0)
	{
		pulse_cntr --;
		//length_compen --;				//如果为6圈1米则每25米补1米（6.5圈）
		//if(length_compen <= 0)
		//{
		//	length_compen = 625;
		//	pulse_cntr = pulse_cntr + 25;
		//}
		if(pulse_cntr == 0)
		{
			len_1[0] --;
			if(len_1[0] < 0)
			{
				len_1[1] --;
				if(len_1[1] < 0)
				{
					len_1[2] --;
					if(len_1[2] < 0)
					{
						len_1[0] = 0;
						len_1[1] = 0;
						len_1[2] = 0;
						flag3.BIT.wind_end = 1;
						flag1.BIT.cutter_down = 1;
						cutter_timer = 15;
					}
					else
					{
						len_1[0] = 99;
						len_1[1] = 99;
					}
				}
				else
				{
					len_1[0] = 99;
				}
			}		
			pulse_cntr = 338;					//puls cntr = (48 x 1000)/(45.2 *3.14) = 338.2
		}
	}
}
/*************************************led簛E棠Ｊ阶映绦丒**********************************************/
/*输葋E问?：绿灯闪。2：绿灯亮。3：簛E痰仆绷痢?：簛E屏痢?：簛E粕痢?：簛E痰平惶嫔痢?：簛E聘咚偕痢?：簛E聘咚偕?*/
/*******************************************************************************************************/
void led_mode(unsigned char led_chose)
{
	if(led_chose == 1)
	{
		ledcntr --;
		if(ledcntr == 0)
		{ ledcntr=5; LED_RED=0; LED_GRN=LED_GRN^1; }
	}
	if(led_chose==2)
	{ LED_RED=0; LED_GRN=1; }
	if(led_chose==3)
	{ LED_RED=1; LED_GRN=1; }
	if(led_chose==4)
	{ LED_GRN=0; LED_RED=1; }
	if(led_chose==5)
	{
		ledcntr--;
		if(ledcntr==0)
		{ ledcntr=5; LED_GRN=0; LED_RED=LED_RED^1; }
	}
	if(led_chose==6)
	{
		ledcntr--;
		if(ledcntr==0)
		{ ledcntr=5; LED_RED=LED_RED^1; LED_GRN=LED_GRN^1; }
	}
	if(led_chose==7)
	{ LED_GRN=0; LED_RED=LED_RED^1; }
	if(led_chose==8)
	{ LED_RED=0; LED_GRN=LED_GRN^1; }
}

/**************************************led显示子程衼E************************************/
/****************************************************************************************/
void led_display(void)
{	unsigned char i;
	i = (flag2.byte & 0x07);
	if(flag1.BIT.zero_error == 1)
	{
		led_mode(5);	
	}
	else if(flag1.BIT.adj_start == 1)//张力范围调节状态
	{
		if(tens_temp < 780)
		{ led_mode(5); }
		else if(tens_temp < 790)
		{ led_mode(4); }
		else if(tens_temp < 810)
		{ led_mode(3); }
		else if(tens_temp < 820)
		{ led_mode(2); }
		else
		{ led_mode(1); }
	}
	else
	{
		switch(i)
		{
			case 0 :
				if(flag3.BIT.wind_end == 1)//菌淦结蕘E
				{ led_mode(3); }
				else if(flag3.BIT.wind_start == 1)//菌淦过程中
				{
					if(flag1.BIT.motor_running == 0)
					{ led_mode(5); }
					else
					{ led_mode(2); }
				}
				else//reset后菌淦开始前的状态
				{ led_mode(4); }
				break;
			case 1 :
				led_mode(7);
				break;
			case 2 :
				led_mode(7);
				break;
			case 3 :
				led_mode(6); 
				break;
			case 4 :
				led_mode(6); 
				break;
			case 5 :
				led_mode(3);
				break;
			case 6 :
				led_mode(3);
				break;
			case 7 :
				led_mode(6);
				break;
		}	
	}
}



/***************************************执行脕E雍?***********************************/
/*****************************************************************************************/
void do_command(unsigned char a)
{
	switch (a)
	{
		case 0 : break;
		case 1 : do_reset();
				 break;
		case 2 : do_sensadj();
				 break;
		case 3 : break;
		case 4 : break;
		case 5 : //do_setting();
				 break;
		case 6 : //do_cont_const();
				 break;
		case 7 : break;
	}	
}
/****************************************reset**sensadj***setting****const**************/
/***************************************************************************************/
void do_reset(void)
{
	if(flag2.BIT.zero_end == 0)
	{
		auto_zero();
		if(flag2.BIT.zero_end == 0)
		{
			return;
		}
	}
/*	if(WR==1)//查EEPROM寄存器是否忙
	{
		return;
	}*/
	else
	{

			switch (eeprom_address)
			{
				case 0 : 	if(setlen[0] > 5)//长度末位数非0时为特殊緛E
							{
								flag3.BIT.special_wind = 1;
								special_error_set = setlen[0];
								//PWM1CON2 = 0x0c00;//0x0004:Updates PxDC registers are immediate
							}
							else
							{
								flag3.BIT.special_wind = 0;
								//PWM1CON2 = 0x0F00;//0x0004:Updates PxDC registers are immediate
							}
							eeprom_address ++;
							break;
				case 1 : 	data_backup1();
							eeprom_address ++;
							break;
				case 2 : 	eeprom_address ++;//Nop();			//延迟一个周期
							break;
				case 3 : 	eeprom_address ++;//Nop();			//延迟一个周期
							break;
				case 4 : 	data_backup2();			//设定长度中两位
							eeprom_address ++;
							break;
				case 5:		eeprom_address = 0;
							flag2.byte = 0;
							break;
				default:   	eeprom_address = 0;
							break;
			}
		
	}
}
/***************************************adj*********************************************/
/***************************************************************************************/
void do_sensadj(void)
{
	auto_zero();
	if(flag2.BIT.zero_end == 0)
	{
		return;
	}
	flag2.byte = 0;
	//flag2.BIT.zero_end=0;
	flag1.BIT.adj_start = 1;//执行
}

/***************************************zero11******************************************/
/***************************************************************************************/
void auto_zero(void)
{
	if(zero_delay == 0)//延迟用
	{
	    if(tens_actual <= tens_ref)
	    {
	        tens_zero = tens_actual;
	        flag2.BIT.zero_end = 1;
	        //flag1.BIT.zero_reset=1;
	    }
	    else if(pwm_zero == 500)//调零出磥E
	    {
	        flag1.BIT.zero_error = 1;
	        //flag1.BIT.zero_reset = 1;
	        flag2.BIT.zero_end = 1;
	    }
	    else 
	    {
	        pwm_zero ++;
	    }
		zero_delay = 5;
	}
	else
	{
		zero_delay -- ;
	}
}

/*************************************rx_error******************************************/
/***************************************************************************************/
void rxerror(void)
{
	flag2.byte = 7;			//通讯代陙E
	LED_GRN = 1;
	LED_RED = 0;
	rx_bytecntr = 0;
}
/************************************rx_reset*******************************************/
/**************************************************************************************/
void rxreset(void)
{
	unsigned char c;
	unsigned int i;
	if(flag1.BIT.motor_running == 0)
	{
		//可在此进行id的判断
		if(rx_bytecntr == 5)
		{
			c = 0;
			c = c+rx_buffer[0];
			c = c+rx_buffer[1];
			c = c+rx_buffer[2];
			c = c+rx_buffer[3];
			c = (c&0x7f);
			if(c == rx_buffer[4])
			{
				//竵E律瓒ㄖ?
				setlen[0] = rx_buffer[0];
				len_1[0] = setlen[0];
				setlen[1] = rx_buffer[1];
				len_1[1] = setlen[1];
				setlen[2] = rx_buffer[2];
				len_1[2] = setlen[2];
				pulse_cntr = 143;
				settens=rx_buffer[3];
				tension_compensate();           //张力补偿/计藖E
				tens_desired = tens_set;
				eeprom_address = 0;
				rx_bytecntr = 0;
				flag2.byte = 1;					//reset的处历喧号
				flag3.byte = 0;					//reset的处历喧号
				//调零准备
				OC1R = 0;// Write the duty cycle for the first PWM pulse
                pwm_zero = 0;// Write the duty cycle for the second PWM pulse
				pwm_out_temp = 0;
				zero_delay = 5;
				flag1.BIT.adj_start = 0;			//聛E髁惚丒疚?
				flag1.BIT.zero_error = 0;			//清除调零代牦的信息
				//flag1.BIT.reset_end = 1;			//防止未调零就启动
				stop_delay = 1000;	//stop delay timing set
				speed_min_avg = 0;
				for(i = 0; i < data_total; i++)
				{
					tens_complast[i] = 0;
					tens_addt[i] = 0;
				}

			}
			else
			{
				rxerror();
			}
		}
		else
		{
			rxerror();
		}
	}
	else
	{
		rx_bytecntr = 0;
	}
	
}

/************************************rx_adj*******************************************/
/*************************************************************************************/
void rxadj(void)
{
	if(flag1.BIT.motor_running == 0)//电机静止状态
	{
		if(rx_bytecntr == 0)
		{
			//rx_bytecntr = 0;
			flag2.byte = 2;			//senadj的处历喧号
			//调零准备
			OC1R = 0;// Write the duty cycle for the first PWM pulse
            pwm_zero = 0;// Write the duty cycle for the second PWM pulse
			zero_delay = 5;
			flag1.BIT.adj_start = 0;
			flag1.BIT.zero_error = 0;//清除调零代牦的信息
		}
		else
		{
			rxerror();
		}
	}
	else
	{
		rx_bytecntr = 0;
	}
}

/*************************************rx_send****************************************/
/************************************************************************************/
void rxsend(void)
{
	if(rx_bytecntr == 1)
	{
		settens = rx_buffer[0];
		tension_compensate();//张力补偿
		tens_desired = tens_set;
		tx_enable = 1;       // ここからビットパルス出力を開始
	}
	else if(rx_bytecntr==2)
	{
	        rx_bytecntr = 0;
		tx_enable = 1;       // ここからビットパルス出力を開始
	}
	else
	{
        	rxerror();
	}

}
/************************************************************************************/
void table_rx(unsigned char b)
{
	switch (b)
	{
		case 0 : 	rxerror();
					break;
		case 1 : 	rxreset();
					break;
		case 2 : 	rxadj();
					break;
		case 3 : 	rxsend();
					break; 
		case 4 : 	
					break;
		case 5 : 	
					break;
		case 6 : 	
					break;
	}
}


/**************************************接收信息子函数*************************************/
/*****************************************************************************************/
void data_rx(void)
{
	unsigned char a;
	a = (flag2.byte&0x07);
	if(a != 0)
	{
		do_command(a);
	}
	if(flag1.BIT.rx_start == 1)		//开始接收数据
	{
		rx_bitcntr ++;
		if(rx_bitcntr == 10)
		{
			if(PORT_RX == 1)
			{
				flag2.BIT.command = 1;
			}
			else
			{
				flag2.BIT.command = 0;
			}
		}
		else if(rx_bitcntr == 11)
		{
			rx_bitcntr = 0;
			//进行是否是数据还是脕E卸?
			if(flag2.BIT.command == 0)
			{
				flag1.BIT.rx_start = 0;
				rx_buffer[rx_bytecntr] = rx_buffer_1;
				rx_bytecntr ++;
				if(rx_bytecntr == 6)
				{
					rxerror();
				}
			}
			else
			{
				flag2.BIT.command = 0;
				flag1.BIT.rx_start = 0;
				if((flag2.byte&0x07) == 0)
				{
					table_rx(rx_buffer_1&0x07);
				}
				else if((flag2.byte&0x07)==7)
				{
					table_rx(rx_buffer_1&0x07);
				}
				else
				{
					rx_bytecntr = 0;
				}
			}
		}
		else
		{	//移位接收数据
			if(PORT_RX == 1)
			{
				rx_buffer_1 = (rx_buffer_1 >> 1);
				rx_buffer_1 = (rx_buffer_1 | 0x80);
			}
			else
			{
				rx_buffer_1 = (rx_buffer_1 >> 1);
			}
			
		}
	}
	else if(rx_bytecntr != 0)
	{
		rx_waitcntr --;
		if(rx_waitcntr == 0)
		{
			rx_waitcntr = 17;
			rx_bytecntr = 0;
		}
	}	
}

/*********************************txcntrset*******************************************/
/**************************************************************************************/
void tx_cntrset(void)
{
	tx_bitcntr ++;
	if(tx_bitcntr == 17)
	{
		tx_bitcntr = 0;
		tx_bytecntr ++;

		if(tx_bytecntr == 6)//进行长度减法运藖E
		{
			tx_bytecntr = 0;
			tmp_1 = setlen[1];
			tmp_2 = setlen[2];
			lastlen[0] = setlen[0] - len_1[0];
			if(lastlen[0] < 0)
			{
				lastlen[0] = lastlen[0] + 100;
				tmp_1 --;
				if(tmp_1 < 0)
				{
					tmp_1 = tmp_1 + 100;
					tmp_2 --;
				}
			}

			lastlen[1] = tmp_1 - len_1[1];
			if(lastlen[1] < 0)
			{
				lastlen[1] = lastlen[1] + 100;
				tmp_2 --;
			}
			lastlen[2] = tmp_2 - len_1[2];

		}
		else if(tx_bytecntr == 3)
			{
				//张力处纴E
				txtens[1] = tens_actual / 10;
				txtens[0] = tens_actual % 10;
			}

	}

}

/********************************* Main Controller **************************************/
/***************************************************************************************/
void tx_300_1(void)
{
	switch (tx_bytecntr)
	{
		case 0 : //	txreg_1 = speed_delta;
				//	txreg_1 = tens_temp/8;
				//	txreg_1 = pwm_vol%100;
				//	txreg_1 = (int)pwm_add;
					txreg_1=lastlen[0];
				//	txreg_1 = tens_temp/8;
					break;
		case 1 : //	txreg_1= running_timer / 100;
					txreg_1=lastlen[1];
				//	txreg_1 = special_error_set;
				//	txreg_1 = pwm_vol/100;
				//	txreg_1 = (int)pwm_out_temp;
				//	txreg_1 = velocity%100;
					break;
		case 2 :	txreg_1=lastlen[2];
				//	txreg_1 = pwm_out/100;
				//	txreg_1 = tens_deviation/8;
				//	txreg_1 = tens_temp/8;
				//	txreg_1 = HallValue;
				//	txreg_1 = velocity/100;
					break;
		case 3 : 	txreg_1 = txtens[0];
					break;
		case 4 : 	txreg_1 = txtens[1];
					break;
		case 5 : 	txreg_1 = 0x7f;
					break;
	}
	PORT_TX = 1;
	
}

/**********************************发送时的移位操讈E***********************************/
/**************************************************************************************/
void tx_300_2(void)
{
	if((txreg_1 & 0x01) == 0)
	{
		PORT_TX = 1;
	}
	else
	{
		PORT_TX = 0;
	}
	txreg_1 = (txreg_1 >> 1);
	
}

/**********************************发送基准数据****************************************/
/**************************************************************************************/
void tx_300_3(void)
{
	PORT_TX = 0;
}

/**************************************发送信息子函数*************************************/
/*****************************************************************************************/
void data_tx(void)
{
	
	if((tx_cntr & 0x01) == 0)
	{
		switch (tx_bitcntr)
		{
			case 0 : 	tx_300_1();
						break;
			case 1 : 	tx_300_2();
						break;
			case 2 : 	tx_300_2();
						break;
			case 3 : 	tx_300_2();
						break;
			case 4 : 	tx_300_2();
						break;
			case 5 : 	tx_300_2();
						break;
			case 6 : 	tx_300_2();
						break;
			case 7 : 	tx_300_2();
						break;
			case 8 : 	tx_300_3();
						break;
			case 9 : 	tx_300_3();
						break;
			case 10 : 	tx_300_3();
						break;
			case 11 : 	tx_300_3();
						break;
			case 12 : 	tx_300_3();
						break;
			case 13 : 	tx_300_3();
						break;
			case 14 : 	tx_300_3();
						break;
			case 15 : 	tx_300_3();
						break;
			case 16 : 	tx_300_3();
						break;
			
		}
		tx_cntrset();
	}
	tx_cntr ++;
	if(tx_cntr == 127)
	{
		tx_cntr = 1;
	}
}
/***********************************张力调諄E*******************************************/
/***************************************************************************************/
void tension_compensate(void)
{
	unsigned int t_backup;
	tens_set = settens;               //主控板在发送时乘以2，所以当设定100克时，settens的值是200
	if(tens_set > 200)//100g为畜值
	{
		tens_set = 200;
	}
	else if(tens_set < 10)
	{
		tens_set = 10;
	}
	t_backup = tens_set*4; //(tens_set*8)/2;	//1g = 8个AD值,100g的AD总值为800
	t_backup = t_backup - 16;	//去2克阻力
	tens_set = t_backup;

	if(tens_set < 200)//20120105, tension set <25g;  200/8=25;
	{
		Kp = 0.13; //Ref 0.03
		Ki = 0.02; //Ref 0.008
		Kd = 0.005; //Ref 0.011
	}
	else //for tension >=25g
	{
		Kp = 0.08;//Ref 0.03
		Ki = 0.02; //Ref 0.008
		Kd = 0.004; //Ref 0.011
	}
	//Kp = 0.029; //Ref 0.03
	//Ki = 0.00025; //Ref 0.008
	//Kd = 0.011; //Ref 0.011
}
/**************************************AD转换子程衼E**************************************/
/*************************************电机控制子程衼E*************************************/
/***********************24us停止，畜运行状态,long=65us,int=35us*************************/
/*****************************************************************************************/
void motor_control(void)
{

	if(flag1.BIT.motor_running == 1)
	{
		prev_deviation = last_deviation;
		last_deviation = tens_deviation;
		tens_deviation = tens_temp - tens_desired;//当前误瞾E

		/*if(rampup_timer != 0)	//for start, 10 sec, add tension
		{
			rampup_timer --;
			tens_deviation = tens_deviation -80;//增加10g张力

			pwm_add = Kp * (tens_deviation - last_deviation); 
		}
		else */
/****************************20200321 updated**************************************/
		if(flag1.BIT.speed_drop == 1)		// speed drop, 20200321
		{
			pwm_add = 0.06 * (tens_deviation - last_deviation) + Ki * tens_deviation + Kd * (tens_deviation + prev_deviation - 2 * last_deviation);;
		}
/**********************************************************************************/
		else if(flag1.BIT.special_start == 0)//normal running
		{
			/*************** step 1: find Kp *********************/
			//1. set Kp = 0.03
			//2. check tension, if shock, change Kp, 0.03->0.035->0.04 ....
			//3. If worse, change Kp, 0.03->0.025->0.02 ....
			//find the best value
			//pwm_add = Kp * (tens_deviation - last_deviation);

			/*************** step 2: find Ki *********************/
			//1. Commented-out the line 1480, add "//".
			//2. Use the following line: Remove "//"

			//pwm_add = Kp * (tens_deviation - last_deviation) + Ki * tens_deviation;

			//3. Set Kp = 5/6(Kp), set Ki = 0.008
			//4. check tension, if shock, change Ki, 0.008->0.009->0.01 .....
			//5. If worse, change Ki, 0.008->0.025->0.02 ....

			/*************** step 3: find Kd *********************/
			//1. Commented-out the line 1486, add "//".
			//2. Use the following line: Remove "//"

			pwm_add = Kp * (tens_deviation - last_deviation) + Ki * tens_deviation + Kd * (tens_deviation + prev_deviation - 2 * last_deviation);

			//3. Set Kd=0.011
			//4. check tension, if shock, change Kd, 0.011->0.012->0.013 .....
			//5. If worse, change Kd, 0.011->0.010->0.009 .... 

		}
		else//filling running
		{
			tens_deviation = tens_deviation + tens_add;//add tension compensation, calculated by last 10 sec
			tens_comp = tens_temp - tens_desired;
			//PI control
			//pwm_add = Kp * (tens_deviation - last_deviation) + Ki * tens_deviation;
			//PID control
			pwm_add = Kp * (tens_deviation - last_deviation) + Ki * tens_deviation + Kd * (tens_deviation + prev_deviation - 2 * last_deviation);
			//please replace Kp by 0.029, or replace Ki by 0.00025;  or replace Kd by 0.011;..
		}

		pwm_out_temp = pwm_out_temp + pwm_add;
		//pwm_out = (int)pwm_out_temp;
		//pwm_out = (int)(pwm_out_temp / pid_scale);

		if(pwm_out_temp <= 0)
		{
		//	pwm_out = 0;
			pwm_out_temp = 0;
		}
		else if(pwm_out_temp>0 && pwm_out_temp<80)
        {
            //pwm_out=80;
			pwm_out_temp = 80;
        }
        else if(pwm_out_temp>500)
        {
            pwm_out_temp=500;
        }

	}
	//else if(stop_delay != 0) //小于5克，开始制动计数
	//{
		//stop_delay--;
		//flag1.BIT.brake = 1;		
	//}
	else
	{
		//flag1.BIT.brake = 0;
		pwm_out_temp = 0;
		pwm_out = 0;
	}

	if(tens_temp < 35) //tension <5g, brake timing
	{
		if(stop_delay != 0)
		{stop_delay--;}
		else
		{
			pwm_out_temp = 0;
			pwm_out = 0;		
		}
	}
	else
	{stop_delay=50;} //timing=25ms(500us x 50)


//pwm_out=500;
	pwm_vol = (int)pwm_out_temp;
	//pwm_vol = tens_temp;
}
/**********************************************************************/
/**********************************************************************/
/**********************************************************************/
/********************END***********************************************/
/**********************************************************************/
/**********************************************************************/
/**************************************************/
/* 延时子程衼E*/
/**************************************************/
void Delay(unsigned int DelayTime)
{
	while(DelayTime--);
}
/**************************************数据备份函数****************************************/
/******************************************************************************************/
void data_backup1(void)
{
	IdleI2C();						//Ensure Module is Idle
	StartI2C();						//Generate Start COndition
	WriteI2C(0xA0);			//Write Control byte
	IdleI2C();
	//ErrorCode = ACKStatus();		//Return ACK Status
	WriteI2C(0x00);				//Write Low Address
	IdleI2C();
	//ErrorCode = ACKStatus();		//Return ACK Status

	WriteI2C(settens);					//Write Data, address 0
	IdleI2C();
	WriteI2C(len_1[0]);					//Write Data, address 01
	IdleI2C();
	WriteI2C(len_1[1]);					//Write Data, address 02
	IdleI2C();
	WriteI2C(len_1[2]);					//Write Data, address 03
	IdleI2C();
	WriteI2C(flag3.byte);				//Write Data, address 04
	IdleI2C();
	WriteI2C(setlen[0]);				//Write Data, address 05
	IdleI2C();
	WriteI2C(setlen[1]);				//Write Data, address 06
	IdleI2C();
	WriteI2C(setlen[2]);				//Write Data, address 07
	IdleI2C();
	StopI2C();						//Initiate Stop Condition
									//Generate Stop
	//CLRWDT();
}
/**************************************数据备份函数****************************************/
/******************************备份pwm_zero,tens_zero**************************************/
/******************************************************************************************/
void data_backup2(void)
{
	IdleI2C();						//Ensure Module is Idle
	StartI2C();						//Generate Start COndition
	WriteI2C(0xA0);			//Write Control byte
	IdleI2C();
	//ErrorCode = ACKStatus();		//Return ACK Status
	WriteI2C(0x08);				//Write Low Address
	IdleI2C();
	//ErrorCode = ACKStatus();		//Return ACK Status

	WriteI2C(pwm_zero&0xff);			//Write Data, address 8
	IdleI2C();
	WriteI2C((pwm_zero>>8)&0xff);		//Write Data, address 9
	IdleI2C();
	WriteI2C(tens_zero&0xff);			//Write Data, address a
	IdleI2C();
	WriteI2C((tens_zero>>8)&0xff);		//Write Data, address b
	IdleI2C();
	WriteI2C(speed_min_avg);			//Write Data, address c
	IdleI2C();
	WriteI2C(life[0]);			//Write Data, address d
	IdleI2C();
	WriteI2C(life[1]);			//Write Data, address e
	IdleI2C();
	WriteI2C(life[2]);			//Write Data, address f
	IdleI2C();
	StopI2C();						//Initiate Stop Condition
									//Generate Stop
	//CLRWDT();
}
/**************************************数据读取函数****************************************/
/******************************************************************************************/
void data_load1(void)
{
unsigned char tempdata;
	IdleI2C();					//wait for bus Idle
	StartI2C();					//Generate Start Condition
	WriteI2C(0xA0);		//Write Control Byte
	IdleI2C();					//wait for bus Idle
	WriteI2C(0x00);			//Write start address
	IdleI2C();					//wait for bus Idle

	RestartI2C();				//Generate restart condition
	WriteI2C(0xA1);	//Write control byte for read
	IdleI2C();					//wait for bus Idle

	settens = getI2C();		//get a single byte, address 0
	AckI2C();				//Acknowledge until all read
	len_1[0] = getI2C();	//Read Data, address 01
	AckI2C();				//Acknowledge until all read		
	len_1[1] = getI2C();	//Read Data, address 02
	AckI2C();				//Acknowledge until all read	
	len_1[2] = getI2C();	//Read Data, address 03
	AckI2C();				//Acknowledge until all read
	flag3.byte = getI2C();	//Read Data, address 04
	AckI2C();				//Acknowledge until all read
	setlen[0] = getI2C();	//Read Data, address 05
	AckI2C();				//Acknowledge until all read
	setlen[1] = getI2C();	//Read Data, address 06
	AckI2C();				//Acknowledge until all read
	setlen[2] = getI2C();	//Read Data, address 07
	AckI2C();

	tempdata = getI2C();	//Read Data, address 08
	AckI2C();				//Acknowledge until all read
	pwm_zero = getI2C();	//Read Data, address 09
	pwm_zero = ((pwm_zero<<8) | tempdata);	//make out pwm_zero
	AckI2C();				//Acknowledge until all read
	tempdata = getI2C();	//Read Data, address 0a
	AckI2C();				//Acknowledge until all read
	tens_zero = getI2C();	//Read Data, address 0b
	tens_zero = ((tens_zero<<8) | tempdata);	//make out tens_zero
	AckI2C();				//Acknowledge until all read
	speed_min_avg = getI2C();	//Read Data, address 0c
	NotAckI2C();				//Send Not Ack
	StopI2C();					//Generate Stop
}


/*********************************************************************
* Function:        StartI2C()
*
* Input:		None.
*
* Output:		None.
*
* Overview:		Generates an I2C Start Condition
*
* Note:			None
********************************************************************/
void StartI2C(void)
{
	//This function generates an I2C start condition and returns status 
	//of the Start.

	I2C1CONbits.SEN = 1;		//Generate Start COndition
	while (I2C1CONbits.SEN);	//Wait for Start COndition
	//return(I2C1STATbits.S);	//Optionally return status
}


/*********************************************************************
* Function:        RestartI2C()
*
* Input:		None.
*
* Output:		None.
*
* Overview:		Generates a restart condition and optionally returns status
*
* Note:			None
********************************************************************/
void RestartI2C(void)
{
	//This function generates an I2C Restart condition and returns status 
	//of the Restart.

	I2C1CONbits.RSEN = 1;		//Generate Restart		
	while (I2C1CONbits.RSEN);	//Wait for restart	
	//return(I2C1STATbits.S);	//Optional - return status
}


/*********************************************************************
* Function:        StopI2C()
*
* Input:		None.
*
* Output:		None.
*
* Overview:		Generates a bus stop condition
*
* Note:			None
********************************************************************/
void StopI2C(void)
{
	//This function generates an I2C stop condition and returns status 
	//of the Stop.

	I2C1CONbits.PEN = 1;		//Generate Stop Condition
	while (I2C1CONbits.PEN);	//Wait for Stop
	//return(I2C1STATbits.P);	//Optional - return status
}


/*********************************************************************
* Function:        WriteI2C()
*
* Input:		Byte to write.
*
* Output:		None.
*
* Overview:		Writes a byte out to the bus
*
* Note:			None
********************************************************************/
void WriteI2C(unsigned char byte)
{
	//This function transmits the byte passed to the function
	//while (I2C1STATbits.TRSTAT);	//Wait for bus to be idle
	I2C1TRN = byte;					//Load byte to I2C1 Transmit buffer
	while (I2C1STATbits.TBF);		//wait for data transmission

}


/*********************************************************************
* Function:        IdleI2C()
*
* Input:		None.
*
* Output:		None.
*
* Overview:		Waits for bus to become Idle
*
* Note:			None
********************************************************************/
void IdleI2C(void)
{
	while (I2C1STATbits.TRSTAT);		//Wait for bus Idle
}


/*********************************************************************
* Function:        LDByteWriteI2C()
*
* Input:		Control Byte, 8 - bit address, data.
*
* Output:		None.
*
* Overview:		Write a byte to low density device at address LowAdd
*
* Note:			None
********************************************************************/
unsigned int LDByteWriteI2C(unsigned char LowAdd, unsigned char data)
{
	unsigned int ErrorCode;

	IdleI2C();						//Ensure Module is Idle
	StartI2C();						//Generate Start COndition
	WriteI2C(0xA0);			//Write Control byte
	IdleI2C();

	ErrorCode = ACKStatus();		//Return ACK Status
	
	WriteI2C(LowAdd);				//Write Low Address
	IdleI2C();

	ErrorCode = ACKStatus();		//Return ACK Status

	WriteI2C(data);					//Write Data
	IdleI2C();
	StopI2C();						//Initiate Stop Condition
	//EEAckPolling(0xA0);		//Perform ACK polling
	return(ErrorCode);
}
/*********************************************************************
* Function:     OneByteReadI2C()
* Input:		Control Byte, Address, *Data, Length.
* Output:		None.
* Overview:		Performs a low density read of a single byte from Bus
* Note:			Add by Zhang,Justin
********************************************************************/
/*unsigned int OneByteReadI2C(unsigned char Address)
{
    unsigned int OneByte;
	IdleI2C();					//wait for bus Idle
	StartI2C();					//Generate Start Condition
	WriteI2C(0xA0);		//Write Control Byte
	IdleI2C();					//wait for bus Idle
	WriteI2C(Address);			//Write start address
	IdleI2C();					//wait for bus Idle

	RestartI2C();				//Generate restart condition
	WriteI2C(0xA1);	//Write control byte for read
	IdleI2C();					//wait for bus Idle

	OneByte = getI2C();		//get a single byte
	//if(I2C1STATbits.BCL)		//Test for Bus collision
	//{
	//	return(-1);
	//}
	
	NotAckI2C();				//Send Not Ack
	StopI2C();					//Generate Stop
	return(OneByte);
}*/
/*********************************************************************
* Function:        ACKStatus()
*
* Input:		None.
*
* Output:		Acknowledge Status.
*
* Overview:		Return the Acknowledge status on the bus
*
* Note:			None
********************************************************************/
unsigned int ACKStatus(void)
{
	return (!I2C1STATbits.ACKSTAT);		//Return Ack Status
}


/*********************************************************************
* Function:        NotAckI2C()
*
* Input:		None.
*
* Output:		None.
*
* Overview:		Generates a NO Acknowledge on the Bus
*
* Note:			None
********************************************************************/
void NotAckI2C(void)
{
	I2C1CONbits.ACKDT = 1;			//Set for NotACk
	I2C1CONbits.ACKEN = 1;
	while(I2C1CONbits.ACKEN);		//wait for ACK to complete
	I2C1CONbits.ACKDT = 0;			//Set for NotACk
}


/*********************************************************************
* Function:        AckI2C()
*
* Input:		None.
*
* Output:		None.
*
* Overview:		Generates an Acknowledge.
*
* Note:			None
********************************************************************/
void AckI2C(void)
{
	I2C1CONbits.ACKDT = 0;			//Set for ACk
	I2C1CONbits.ACKEN = 1;
	while(I2C1CONbits.ACKEN);		//wait for ACK to complete
}

/*********************************************************************
* Function:        getI2C()
*
* Input:		None.
*
* Output:		contents of I2C1 receive buffer.
*
* Overview:		Read a single byte from Bus
*
* Note:			None
********************************************************************/
unsigned int getI2C(void)
{
	I2C1CONbits.RCEN = 1;			//Enable Master receive
	Nop();
	while(!I2C1STATbits.RBF);		//Wait for receive bufer to be full
	return(I2C1RCV);				//Return data in buffer
}


/*********************************************************************
* Function:        EEAckPolling()
*
* Input:		Control byte.
*
* Output:		error state.
*
* Overview:		polls the bus for an Acknowledge from device
*
* Note:			None
********************************************************************/
unsigned int EEAckPolling(unsigned char control)
{
	IdleI2C();				//wait for bus Idle
	StartI2C();				//Generate Start condition
	
	if(I2C1STATbits.BCL)
	{
		return(-1);			//Bus collision, return
	}

	else
	{
		WriteI2C(control);
		IdleI2C();			//wait for bus idle
		if(I2C1STATbits.BCL)
		{
			return(-1);		//error return
		}

		while(ACKStatus())
		{
			RestartI2C();	//generate restart
			if(I2C1STATbits.BCL)
			{
				return(-1);	//error return
			}

			WriteI2C(control);
			IdleI2C();
		}
	}
	StopI2C();				//send stop condition
	if(I2C1STATbits.BCL)
	{
		return(-1);
	}
	return(0);
}
/**********************************************************************/
/**********************************************************************/
/**********************************************************************/
/********************END***********************************************/
/**********************************************************************/
/**********************************************************************/
