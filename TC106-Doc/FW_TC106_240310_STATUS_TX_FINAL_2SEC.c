/******************************************/
/******************************************/

#include <p24FJ16GA002.h>

#include <xc.h>




_CONFIG2( POSCMOD_NONE & I2C1SEL_PRI & IOL1WAY_OFF & OSCIOFNC_ON & FCKSM_CSECMD & FNOSC_FRC & SOSCSEL_SOSC & WUTSEL_LEG & IESO_OFF )
_CONFIG1( WDTPS_PS32768 & FWPSA_PR128 & WINDIS_ON & FWDTEN_OFF & ICS_PGx1 & GWRP_OFF & GCP_OFF & JTAGEN_OFF )



#include <math.h>

#define ON 1
#define OFF 0
#define LED_RED _LATA2
#define LED_GRN _LATA3
#define PORT_RX _RB4
#define PORT_TX _LATA4
#define PORT_CUTTER _RA1
#define Button_in _RB0
#define PORT_BOR _RB1
#define tens_ref 105
#define pwm_zero OC1RS
#define pwm_vol OC2RS
#define data_total 200 
#define pid_scale 1000 
/*************************************************************
Low side driver table is as below.  In this StateLoTable,
the Low side driver is PWM while the high side driver is
either on or off.  This table is used in this exercise
*************************************************************/





const unsigned int StateLoTable[8] = {0x0000, 0x8400, 0x1800, 0x9000, 0x6000, 0x2400, 0x4800, 0x0000};

volatile float Kp;
volatile float Ki;
volatile float Kd;
volatile signed int Ka;

volatile unsigned char setlen[3];		
volatile signed char len_1[3];			
volatile signed char lastlen[3];		
volatile unsigned char settens;
volatile unsigned int tens_set;			
volatile unsigned char txtens[2];		
volatile signed char tmp_1;
volatile signed char tmp_2;

volatile unsigned int tens_desired;		
volatile unsigned int tens_actual;		

volatile unsigned int tens_sum;			
volatile unsigned int tens_zero;		
volatile unsigned int tens_temp;
volatile signed int tens_deviation;		
volatile signed int last_deviation;		
volatile signed int prev_deviation;		
volatile float pwm_add;				
volatile float pwm_out_temp;			
volatile signed int pwm_out;			
volatile signed long pwm_check;    		

volatile unsigned int rx_timer;			
volatile unsigned int rx_bitcntr;
volatile unsigned int rx_bytecntr;

volatile unsigned int rx_waitcntr;
volatile unsigned char rx_buffer_1;
volatile unsigned char rx_buffer[6];		
volatile unsigned int tx_timer;			
volatile unsigned int tx_cntr;
volatile unsigned int tx_bitcntr;
volatile unsigned int tx_bytecntr;
volatile signed char txreg_1;
volatile unsigned int stop_delay;		
volatile unsigned int cutter_timer;		

volatile unsigned int check_timer;		
volatile signed int pulse_cntr;			

volatile unsigned int display_timer;
volatile unsigned char flag_status_tx = 0;
volatile unsigned char status_tx_counter = 0;
		
volatile unsigned int ledcntr;			
volatile unsigned int HallValue;
volatile unsigned int eeprom_address;		
volatile unsigned char zero_delay;

volatile signed int tens_addt[data_total];	

volatile signed int tens_complast[data_total];	
volatile signed int tens_comp;
volatile signed int tens_addt_temp;
volatile signed int tens_add;
volatile signed char special_error_set;
volatile unsigned char running_timer;		
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
volatile unsigned char speed_min_avg;		
volatile unsigned char speed_min_array[16];	
volatile unsigned char start_delay;
volatile unsigned int velocity;
volatile unsigned char life[3];			

unsigned int rampup_timer;			


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
		unsigned motor_running :1;	
		unsigned tx_300 :1;		
		unsigned rx_300 :1;		
		unsigned port_check :1;		
		unsigned ad_conv :1;		
		unsigned rx_start :1;		
		unsigned display :1;		
		unsigned lowvolt :1;		
		unsigned speed_drop :1;		
		unsigned speed_count :1;	
		unsigned pulse_count :1;	
		unsigned cutter_down :1;	
		unsigned adj_start :1;		
		unsigned zero_error :1;
		unsigned special_start :1;	
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
		unsigned command :1;			
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
/*                                                                                      */
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

void StartI2C(void);
void RestartI2C(void);
void StopI2C(void);
void WriteI2C(unsigned char byte);
void IdleI2C(void);


unsigned int ACKStatus(void);
void NotAckI2C(void);
void AckI2C(void);
unsigned int getI2C(void);



void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
	IFS0bits.T1IF = 0; 
	if(tx_timer!=0)
	{
		tx_timer--;
	
    status_tx_counter++;
    if (status_tx_counter >= 62) {
        status_tx_counter = 0;
        flag_status_tx = 1;
    }
}
	else
	{
		tx_timer=32;
		flag1.BIT.tx_300=1;		
	}
	if(rx_timer!=0)
	{
		 rx_timer--;
	}
	else
	{
		rx_timer=65;
		flag1.BIT.rx_300=1;		
	}
	if(check_timer != 0) 
	{
		check_timer --;;
	}
	else
	{
		check_timer=4;
		flag1.BIT.port_check=1;		
	}
	if(display_timer!=0) 
	{
		display_timer--;
	}
	else
	{
		display_timer=999;
		flag1.BIT.display=1;

		speed_cntr1 = speed_cntr;
		speed_cntr = 0;
		flag1.BIT.speed_count = 1;
	}

}


void __attribute__((__interrupt__, auto_psv)) _T3Interrupt( void )	
{
	IFS0bits.T3IF = 0;	
	AD1CON1bits.SAMP = 1; 	

}

void __attribute__((__interrupt__, auto_psv)) _CNInterrupt(void)	
{
	
	IFS1bits.CNIF = 0;			
	HallValue = PORTB & 0x00E0;		
	HallValue = HallValue >> 5;		
	PORTB &= 0x03FF;
	asm("nop");
	PORTB |= StateLoTable[HallValue];	

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

	Delay(60000);			
	data_load1();			
	special_error_set = setlen[0];

	tension_compensate();
	tens_desired = tens_set;
	flag1.twobyte = 0x0000;
	flag2.byte = 0x00;
	pulse_cntr = 143;
	ledcntr = 5;
	zero_delay = 2;
	running_timer=0;
	
	
	
	
	
	
	pwm_out_temp = 0;
	pwm_out = 0;		
	cnt_FIR = 0;
	speed_min_sum = 0;
	speed_min_avg = 0;	
	cnt_FIR = 0;
	stop_delay = 1000;
	start_delay = 0;
	rampup_timer = 0;
}

int main(void)
{

	__builtin_write_OSCCONH(0x01);
	__builtin_write_OSCCONL(0x01);
	
	while (OSCCONbits.COSC != 0b001)
	
	while(OSCCONbits.LOCK != 1)

	loop:
	CLKDIV 	= 0x0000;	
	LATA = 0x0000;		
	LATB = 0x0000;		
	
	TRISA = 0x0001;		
	TRISB = 0x03F3;		
	
	/******************** Initialize CN interrupt *********************/
	
	CNEN2 = 0x0980;		
	
	
	/******************** Initialize Output Compare Module ************/
	OC1R = 0;		
	OC1RS = 0;		
	OC1CON = 0x0006;
	
	

	OC2R = 0;		
	OC2RS = 0;		
	OC2CON = 0x0006;

	RPOR1 = 0x1213;		

	
	
	/******************** Initialize Timer2 ***************************/
	T1CON = 0x0000;
	
	
	
	
	
	TMR1 = 0x0000;		
	PR1 = 799;		
	/******************** Initialize Timer1 ***************************/
	T2CON = 0x0000;
	
	
	
	
	
	
	TMR2 = 0x0000;		
	PR2 = 499;		
	/******************** Initialize Timer3 ***************************/
	T3CON = 0x0010;		
	TMR3 = 0x0000;		
	
	PR3 = 124;		
	/******************** intializes ADC ******************************/
	AD1PCFG = 0xFFFE;	
	AD1CSSL = 0x0001;	
	AD1CHS = 0x0000;	
	AD1CON1 = 0x00E0;	
	AD1CON2 = 0x003C;	
	AD1CON3 = 0x1F03; 	
	/******************** intializes I2C ******************************/
	I2C1BRG = 0x0012; 	
	I2C1CON = 0x1200;	
	I2C1RCV = 0x0000;
	I2C1TRN = 0x0000;
	I2C1CON = 0x9200;	
	/******************** Set interrupt priority **********************/
	IPC4bits.CNIP = 5;
	IPC0bits.T1IP = 2;	
	
	IPC2bits.T3IP = 2;	
	
	
	
	/*********************** Enable interrupt *************************/
	IFS0bits.T1IF = 0;	
	IEC0bits.T1IE = 1;	
	T1CONbits.TON = 1;	
	
	
	T2CONbits.TON = 1;	
	IFS0bits.T3IF = 0;	
	IEC0bits.T3IE = 1;	
	T3CONbits.TON = 1;	
	
	

	IFS0bits.AD1IF = 0;
	
	AD1CON1bits.ADON = 1;	
	IFS1bits.CNIF = 0;	
	IEC1bits.CNIE = 1;	
	
	
	
	/***************************** initial **************************/
	para_initial();
	while(1)
	{
		if(flag1.BIT.port_check==1)					
		{
			flag1.BIT.port_check=0;
			check_port();	
		
        if (flag_status_tx == 1) {
            flag_status_tx = 0;
            send_status();
        }
}
		if(flag1.BIT.lowvolt==1)					
		{
			while(!PORT_BOR)					
			goto loop;
		}
		if(flag1.BIT.ad_conv==1)					
		{
			
			flag1.BIT.ad_conv=0;
			motor_control();
			
		}
		if(IFS0bits.AD1IF == 1)
		{
			IFS0bits.AD1IF = 0;

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
			
		}

		if(flag1.BIT.rx_300==1)						
		{
			flag1.BIT.rx_300=0;
			data_rx();
		}
		if(flag1.BIT.tx_300==1)						
		{
			flag1.BIT.tx_300=0;
			data_tx();
		}
		if(flag1.BIT.display==1)					
		{
			flag1.BIT.display=0;
			led_display();
			
			if(flag1.BIT.cutter_down==1)
			{
				PORT_CUTTER=1;								
				cutter_timer--;
				if(cutter_timer==0)
				{
					flag1.BIT.cutter_down=0;
					PORT_CUTTER=0;
				}
			}
		}
		if(flag1.BIT.pulse_count==1)					
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
	
}

/******************************************************************************************/
void check_port(void)
{
	/*if(PORT_BOR == 0)					
	{
		INTCON1bits.NSTDIS = 0;				
		flag1.BIT.lowvolt = 1;				
		P1DC1 = 0;					
		P1DC2 = 0;
		P1DC3 = 0;
		LED_RED = 1;					
		LED_GRN = 1;
		
		data_backup1();					
		INTCON1bits.NSTDIS = 1;				
		
	}
	else */
	
	if(flag1.BIT.rx_start == 0)				
	{
		if(PORT_RX == 0)				
		{
			rx_timer = 33;				
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
*************************************************************************/
void GetSpeed(void)
{
	if ((speed_cntr1 > 3) && (flag1.BIT.cutter_down == 0))	
	{
		flag1.BIT.motor_running = 1;
		flag3.BIT.wind_start = 1;

/**********************************20200322 updated************************************/
		
		if((speed_cntr1 + 10) < speed_min_avg)		
		{
			flag1.BIT.speed_drop = 1;
		}
		else if(speed_cntr1 < (speed_min_avg + 4))	
		{
			flag1.BIT.speedlow = 1;
		}
		else
		{
			flag1.BIT.speedlow = 0;
			flag1.BIT.speed_drop = 0;
		}
/**************************************************************************************/

		
		if(++special_cycletimer == 200)
		{
			special_cycletimer = 0;
			
			speed_deltalast = speed_delta;		
			speed_minlast = speed_min;		
			speed_delta = speed_max - speed_min;	

			cnt_FIR++;
			cnt_FIR = cnt_FIR & 0x07;
			speed_min_sum = speed_min_sum + speed_min - speed_min_array[cnt_FIR];
			speed_min_array[cnt_FIR] = speed_min;

			if(start_delay > 10)			
			{
				
				speed_min_avg = (unsigned char)(speed_min_sum >> 3);	
			}
			else	
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


		
		
		
		
		
		if((speed_delta > special_error_set) && (speed_deltalast > special_error_set) && (flag3.BIT.special_wind == 1))
		{
			flag1.BIT.special_start = 1;
		}
		else
		{
			flag1.BIT.special_start = 0;
		}
	
    }
	else 
	{
        flag1.BIT.motor_running = 0;	
		
		pwm_out = 0;
		pwm_add = 0;
		flag1.BIT.special_start = 0;
		special_cycletimer = 0;
		speed_max=0;
		speed_min=255;
		speed_delta = 0;
		speed_deltalast = 0;
		
		speed_minlast = 0;

		cnt_FIR++;
		cnt_FIR = cnt_FIR & 0x07;
		speed_min_array[cnt_FIR] = 0;	

		speed_min_sum = 0;
		speed_min_avg = 0;		
		start_delay = 0;		

		tens_deviation = 0;
		prev_deviation = 0; 
		last_deviation = 0;
		

		rampup_timer = 8000;
	}







	if(flag1.BIT.special_start == 1)
	{
		running_timer ++;
		if(running_timer>data_total)
		{
			running_timer=data_total;
		}

		if(flag1.BIT.speedlow == 1)
		{
			
			running_timer = 0;
		}
		tens_addt_temp = tens_comp + tens_complast[running_timer+1] - tens_complast[running_timer];
		tens_addt[running_timer] += (tens_addt_temp / 8);
		tens_complast[running_timer] = tens_comp;
		
		tens_add = tens_addt[running_timer];
	}

	

}


/****************************************************************************************/
void pulse_count(void)
{
	if(flag3.BIT.wind_end == 0)
	{
		pulse_cntr --;
		
		
		
		
		
		
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
			pulse_cntr = 338;					
		}
	}
}

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

/****************************************************************************************/
void led_display(void)
{	unsigned char i;
	i = (flag2.byte & 0x07);
	if(flag1.BIT.zero_error == 1)
	{
		led_mode(5);	
	}
	else if(flag1.BIT.adj_start == 1)
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
				if(flag3.BIT.wind_end == 1)
				{ led_mode(3); }
				else if(flag3.BIT.wind_start == 1)
				{
					if(flag1.BIT.motor_running == 0)
					{ led_mode(5); }
					else
					{ led_mode(2); }
				}
				else
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
		case 5 : 
				 break;
		case 6 : 
				 break;
		case 7 : break;
	}	
}

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
/*	if(WR==1)
	{
		return;
	}*/
	else
	{

			switch (eeprom_address)
			{
				case 0 : 	if(setlen[0] > 5)
							{
								flag3.BIT.special_wind = 1;
								special_error_set = setlen[0];
								
							}
							else
							{
								flag3.BIT.special_wind = 0;
								
							}
							eeprom_address ++;
							break;
				case 1 : 	data_backup1();
							eeprom_address ++;
							break;
				case 2 : 	eeprom_address ++;
							break;
				case 3 : 	eeprom_address ++;
							break;
				case 4 : 	data_backup2();			
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

/***************************************************************************************/
void do_sensadj(void)
{
	auto_zero();
	if(flag2.BIT.zero_end == 0)
	{
		return;
	}
	flag2.byte = 0;
	
	flag1.BIT.adj_start = 1;
}

/***************************************************************************************/
void auto_zero(void)
{
	if(zero_delay == 0)
	{
	    if(tens_actual <= tens_ref)
	    {
	        tens_zero = tens_actual;
	        flag2.BIT.zero_end = 1;
	        
	    }
	    else if(pwm_zero == 500)
	    {
	        flag1.BIT.zero_error = 1;
	        
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
	flag2.byte = 7;			
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
				
				setlen[0] = rx_buffer[0];
				len_1[0] = setlen[0];
				setlen[1] = rx_buffer[1];
				len_1[1] = setlen[1];
				setlen[2] = rx_buffer[2];
				len_1[2] = setlen[2];
				pulse_cntr = 143;
				settens=rx_buffer[3];
				tension_compensate();           
				tens_desired = tens_set;
				eeprom_address = 0;
				rx_bytecntr = 0;
				flag2.byte = 1;					
				flag3.byte = 0;					
				
				OC1R = 0;
                pwm_zero = 0;
				pwm_out_temp = 0;
				zero_delay = 5;
				flag1.BIT.adj_start = 0;			
				flag1.BIT.zero_error = 0;			
				
				stop_delay = 1000;	
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
	if(flag1.BIT.motor_running == 0)
	{
		if(rx_bytecntr == 0)
		{
			
			flag2.byte = 2;			
			
			OC1R = 0;
            pwm_zero = 0;
			zero_delay = 5;
			flag1.BIT.adj_start = 0;
			flag1.BIT.zero_error = 0;
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
		tension_compensate();
		tens_desired = tens_set;
	}
	else if(rx_bytecntr==2)
    {
        rx_bytecntr = 0;
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


/*****************************************************************************************/
void data_rx(void)
{
	unsigned char a;
	a = (flag2.byte&0x07);
	if(a != 0)
	{
		do_command(a);
	}
	if(flag1.BIT.rx_start == 1)		
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
		{	
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

		if(tx_bytecntr == 6)
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
		case 0 : 
				
					txreg_1 = pwm_vol%100;
				
				
				
					break;
		case 1 : 
				
				
					txreg_1 = pwm_vol/100;
				
				
					break;
		case 2 :
				
				
					txreg_1 = tens_temp/8;
				
				
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

/**************************************************************************************/
void tx_300_3(void)
{
	PORT_TX = 0;
}

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


/***************************************************************************************/
void tension_compensate(void)
{
	unsigned int t_backup;
	tens_set = settens;			
	if(tens_set > 200)			
	{
		tens_set = 200;
	}
	else if(tens_set < 10)
	{
		tens_set = 10;
	}
	t_backup = tens_set*4;			
	t_backup = t_backup - 16;	
	tens_set = t_backup;

	if(tens_set < 200)
	{
		Kp = 0.029; 
		Ki = 0.00025; 
		Kd = 0.011; 
	}
	else 
	{
		Kp = 0.025; 
		Ki = 0.001; 
		Kd = 0.011; 
	}
	
	
	
}

/*****************************************************************************************/
void motor_control(void)
{

	if(flag1.BIT.motor_running == 1)
	{
		prev_deviation = last_deviation;
		last_deviation = tens_deviation;
		tens_deviation = tens_temp - tens_desired;

		/*if(rampup_timer != 0)	
		{
			rampup_timer --;
			tens_deviation = tens_deviation -80;

			pwm_add = Kp * (tens_deviation - last_deviation); 
		}
		else */
/****************************20200321 updated**************************************/
		if(flag1.BIT.speed_drop == 1)		
		{
			pwm_add = 0.06 * (tens_deviation - last_deviation) + Ki * tens_deviation + Kd * (tens_deviation + prev_deviation - 2 * last_deviation);;
		}
/**********************************************************************************/
		else if(flag1.BIT.special_start == 0)
		{
			/*************** step 1: find Kp *********************/
			
			
			
			
			

			/*************** step 2: find Ki *********************/
			
			

			

			
			
			

			/*************** step 3: find Kd *********************/
			
			

			pwm_add = Kp * (tens_deviation - last_deviation) + Ki * tens_deviation + Kd * (tens_deviation + prev_deviation - 2 * last_deviation);

			
			
			

		}
		else
		{
			tens_deviation = tens_deviation + tens_add;
			tens_comp = tens_temp - tens_desired;
			
			
			
			pwm_add = Kp * (tens_deviation - last_deviation) + Ki * tens_deviation + Kd * (tens_deviation + prev_deviation - 2 * last_deviation);
			
		}

		pwm_out_temp = pwm_out_temp + pwm_add;
		
		

		if(pwm_out_temp <= 0)
		{
		
			pwm_out_temp = 0;
		}
		else if(pwm_out_temp>0 && pwm_out_temp<80)
        {
            
			pwm_out_temp = 80;
        }
        else if(pwm_out_temp>500)
        {
            pwm_out_temp=500;
        }

	}

	else
	{
		
		pwm_out_temp = 0;
		pwm_out = 0;
	}

	if(Button_in == 0) 
	{
		pwm_out_temp = 200;
		pwm_out = 200;	
	}
	else if(tens_temp < 35) 
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
	{stop_delay=50;} 



	pwm_vol = (int)pwm_out_temp;
	
}
/**********************************************************************/
/**********************************************************************/
/**********************************************************************/
/**********************************************************************/
/**************************************************/
/**************************************************/
void Delay(unsigned int DelayTime)
{
	while(DelayTime--);
}


/******************************************************************************************/
void data_backup1(void)
{
	IdleI2C();						
	StartI2C();						
	WriteI2C(0xA0);			
	IdleI2C();
	
	WriteI2C(0x00);				
	IdleI2C();
	

	WriteI2C(settens);					
	IdleI2C();
	WriteI2C(len_1[0]);					
	IdleI2C();
	WriteI2C(len_1[1]);					
	IdleI2C();
	WriteI2C(len_1[2]);					
	IdleI2C();
	WriteI2C(flag3.byte);				
	IdleI2C();
	WriteI2C(setlen[0]);				
	IdleI2C();
	WriteI2C(setlen[1]);				
	IdleI2C();
	WriteI2C(setlen[2]);				
	IdleI2C();
	StopI2C();						
									
	
}


/******************************************************************************************/
void data_backup2(void)
{
	IdleI2C();						
	StartI2C();						
	WriteI2C(0xA0);			
	IdleI2C();
	
	WriteI2C(0x08);				
	IdleI2C();
	

	WriteI2C(pwm_zero&0xff);			
	IdleI2C();
	WriteI2C((pwm_zero>>8)&0xff);		
	IdleI2C();
	WriteI2C(tens_zero&0xff);			
	IdleI2C();
	WriteI2C((tens_zero>>8)&0xff);		
	IdleI2C();
	WriteI2C(speed_min_avg);			
	IdleI2C();
	WriteI2C(life[0]);			
	IdleI2C();
	WriteI2C(life[1]);			
	IdleI2C();
	WriteI2C(life[2]);			
	IdleI2C();
	StopI2C();						
									
	
}

/******************************************************************************************/
void data_load1(void)
{
unsigned char tempdata;
	IdleI2C();					
	StartI2C();					
	WriteI2C(0xA0);		
	IdleI2C();					
	WriteI2C(0x00);			
	IdleI2C();					

	RestartI2C();				
	WriteI2C(0xA1);	
	IdleI2C();					

	settens = getI2C();		
	AckI2C();				
	len_1[0] = getI2C();	
	AckI2C();				
	len_1[1] = getI2C();	
	AckI2C();				
	len_1[2] = getI2C();	
	AckI2C();				
	flag3.byte = getI2C();	
	AckI2C();				
	setlen[0] = getI2C();	
	AckI2C();				
	setlen[1] = getI2C();	
	AckI2C();				
	setlen[2] = getI2C();	
	AckI2C();

	tempdata = getI2C();	
	AckI2C();				
	pwm_zero = getI2C();	
	pwm_zero = ((pwm_zero<<8) | tempdata);	
	AckI2C();				
	tempdata = getI2C();	
	AckI2C();				
	tens_zero = getI2C();	
	tens_zero = ((tens_zero<<8) | tempdata);	
	AckI2C();				
	speed_min_avg = getI2C();	
	NotAckI2C();				
	StopI2C();					
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
	
	

	I2C1CONbits.SEN = 1;		
	while (I2C1CONbits.SEN);	
	
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
	
	

	I2C1CONbits.RSEN = 1;		
	while (I2C1CONbits.RSEN);	
	
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
	
	

	I2C1CONbits.PEN = 1;		
	while (I2C1CONbits.PEN);	
	
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
	
	
	I2C1TRN = byte;					
	while (I2C1STATbits.TBF);		

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
	while (I2C1STATbits.TRSTAT);		
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

	IdleI2C();						
	StartI2C();						
	WriteI2C(0xA0);			
	IdleI2C();

	ErrorCode = ACKStatus();		
	
	WriteI2C(LowAdd);				
	IdleI2C();

	ErrorCode = ACKStatus();		

	WriteI2C(data);					
	IdleI2C();
	StopI2C();						
	
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
	IdleI2C();					
	StartI2C();					
	WriteI2C(0xA0);		
	IdleI2C();					
	WriteI2C(Address);			
	IdleI2C();					

	RestartI2C();				
	WriteI2C(0xA1);	
	IdleI2C();					

	OneByte = getI2C();		
	
	
	
	
	
	NotAckI2C();				
	StopI2C();					
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
	return (!I2C1STATbits.ACKSTAT);		
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
	I2C1CONbits.ACKDT = 1;			
	I2C1CONbits.ACKEN = 1;
	while(I2C1CONbits.ACKEN);		
	I2C1CONbits.ACKDT = 0;			
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
	I2C1CONbits.ACKDT = 0;			
	I2C1CONbits.ACKEN = 1;
	while(I2C1CONbits.ACKEN);		
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
	I2C1CONbits.RCEN = 1;			
	Nop();
	while(!I2C1STATbits.RBF);		
	return(I2C1RCV);				
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
	IdleI2C();				
	StartI2C();				
	
	if(I2C1STATbits.BCL)
	{
		return(-1);			
	}

	else
	{
		WriteI2C(control);
		IdleI2C();			
		if(I2C1STATbits.BCL)
		{
			return(-1);		
		}

		while(ACKStatus())
		{
			RestartI2C();	
			if(I2C1STATbits.BCL)
			{
				return(-1);	
			}

			WriteI2C(control);
			IdleI2C();
		}
	}
	StopI2C();				
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


void send_status(void)
{
    unsigned char motor_flag = flag1.BIT.motor_running ? 1 : 0;
    unsigned char buf[7];

    buf[0] = 0xAA; // Start byte
    buf[1] = pwm_vol % 100;
    buf[2] = pwm_vol / 100;
    buf[3] = tens_temp / 8;
    buf[4] = motor_flag;
    buf[5] = (buf[1] + buf[2] + buf[3] + buf[4]) & 0xFF; // Checksum
    buf[6] = 0x7F; // End byte

    for (int i = 0; i < 7; i++) {
        txreg_1 = buf[i];
        for (int j = 0; j < 8; j++) {
            PORT_TX = (txreg_1 & 0x01) ? 0 : 1;
            txreg_1 >>= 1;
            Delay(10);
        }
        PORT_TX = 1;
        Delay(10);
    }
}
