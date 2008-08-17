/*****************************************************************************	
 *	bbot_util.c                                                                           
 *                      
 *	Useful utilities for FlutterBot.
 *
 *	Revisions:
 *	                 
 *   Hardware:
 *		See Comments in main.c
 *                         
 ****************************************************************************
*/

#include <avr/io.h>
#include <math.h>
#include <avr/interrupt.h>

#include "button.h"
#include "LCD_driver.h"
#include "bbot_util.h"
#include "parameters.h"

#include "avrlibtypes.h"

/*
	motor control via H-bridge and PWM using Timer/Counter 1

	A motor will turn when its inputs differ. The direction depends on which
	input is positive and which is ground. (If both are the same, the motor
	will stop moving!)

	When we go forward, we send a low (0) signal to the H-bridge for direction.
	We also send a PWM (pulse width modulation) signal to the H-bridge. When
	that signal is high, the motor turns; when it is low, the motor stops.

	So, we send a pulse that's high for the width specified, then drop it
	low for the remainder of the pulse frame, returning it to high at the
	end of that frame, when the next pulse starts.
	 ___   ___   ___
	|   |_|   |_|   |_		wave for 75% full speed forward

	When we go in reverse, we send a high (1) signal to the H-bridge for
	direction. In this case, the motor turns when the PWM signal is low,
	and stops when it's high.

	In this case, we sent a pulse that's low for the width specified, then
	raise it for the remainder of the frame.
	     _     _     _
	 ___| |___| |___| |		wave for 75% full speed in reverse


	The code below sets up Timer/Counter 1 as follows:
	- Waveform Generation Mode: Fast PWM, 8-bit (0101) -- counting from 0-255
			(then wrapping)
	- Compare Output Mode: this determines how the signals on OC1A/PB6
				(right motor) and OC1B/PB5 (left motor) change when the
				counter matches the value in OCR1A or OCR1B (our "PWM"
				values indicating the desired speeds of the motors)
				and at the maximum count value (255, end of frame).
			clear on match, set at TOP (10) [for forward, see above], or
			set on match, clear at TOP (11) [for reverse, see above]
	- Clock Select: clkIO/8 (010)
			8 Mhz / 64 (prescaler) = 128 kHz for counter
			128 kHz / 256 (clock pulses/frame) = 512 Hz for PWM [frame ~2 ms]

	See the Atmel ATmega169 data sheet for details on these settings and how
	they correspond to bits in TCCR1A/B (Timer/Counter 1 Control Registers).
*/

/* PB2/PB3 control left/right motor direction (forward/reverse) */
#define SetupLDir()	( SetBit(DDRB, DDB2), ClearBit(PORTB, PB2) )
#define SetupRDir()	( SetBit(DDRB, DDB3), ClearBit(PORTB, PB3) )

/* PWM output on PB5/OC1B for left motor, PB6/OC1A for right motor. */
#define SetupLPWM()	( SetBit(DDRB, DDB6), ClearBit(PORTB, PB6) )
#define SetupRPWM()	( SetBit(DDRB, DDB5), ClearBit(PORTB, PB5) )

/* we compare to OCR1A/B for R/L motor speeds */
#define lPWM		OCR1B	// Counter tied to PB6
#define rPWM		OCR1A	// Counter tied to PB5 

/* set direction (input to H-bridge) and wave output mode */
#define LFwd()		(ClearBit(PORTB, PB2),   SetBit(TCCR1A, COM1B1), ClearBit(TCCR1A, COM1B0))
#define LRev()		(  SetBit(PORTB, PB2),   SetBit(TCCR1A, COM1B1),   SetBit(TCCR1A, COM1B0))
#define RFwd()		(ClearBit(PORTB, PB3),   SetBit(TCCR1A, COM1A1), ClearBit(TCCR1A, COM1A0))
#define RRev()		(  SetBit(PORTB, PB3),   SetBit(TCCR1A, COM1A1),   SetBit(TCCR1A, COM1A0))

void init_pwm (void)
{
	/* set up ports */
	SetupLDir();
	SetupRDir();
	SetupLPWM();
	SetupRPWM();

	TCNT1 = 0;

	/* start with motors disconnected from Timer/Counter output */
    // fast PWM 8 bit, clock div 64 both pwm output compares cleared on match
    TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM10);  
    TCCR1B = (1<<WGM12) | (1<<CS11)| (1<<CS10);                   
  
	/* OCR1A/B are the values that the timer is compared to; a match will
	   cause the output to change; small values mean the motor runs for a
	   short period (slower); larger values are longer times (faster)			*/
	lPWM = rPWM = 0;
}


void SetLeftMotorPWM(int pwm)
{
	if (pwm >= 0) {
		LFwd();
	} else {
		LRev();
		pwm = -pwm;
	}

	if (pwm > 0 ) 
		pwm += lmb_PARAM;	// add motor bias (dead band);


	if (pwm > 255)
		pwm = 255;
	lPWM = pwm;
}

void SetRightMotorPWM(int pwm)
{
	if (pwm >= 0) {
		RFwd();
	} else {
		RRev();
		pwm = -pwm;
	}

	if (pwm > 0 ) 
		pwm += rmb_PARAM;	// add motor bias (dead band);
	
	if (pwm > 255)						
		pwm = 255;

	rPWM = pwm;
}


/*********************************************************************
*	Analog-to-Digital Converter (ADC) 
**********************************************************************/
void InitADC(void) 
{
	// need to disable digital inputs on portF pins we are using as adc.
	DIDR0 = (1<<ADC4D) | (1<<ADC5D);  // just using adc4 & adc5 for now
    
	// prob don't need to set adc in as input, but can't hurt.  
	ClearBit(DDRF, DDF4);    // bits 4 & 5 are inputs  
	ClearBit(DDRF, DDF5);   

//	SetBit(PORTF, PF4);      // pullup's on bit 4 & 5
//	SetBit(PORTF, PF5);      

	// Turn on port F bit 3, it's used as ADC Ref voltage.  
	// (it also powers thermistor, light sensor, and volt reader, but we can't help that.)
	SetBit(PORTF, PF3);    // turn bit "on" 
	SetBit(DDRF, DDF3);    // make it an output  
	
//    ADMUX	= (1<<ADLAR) | 0x04;					// select external AREF, left adjusted result, mux input 4
    ADMUX	= 0x04;					// select external AREF, left adjusted result, mux input 4
	ADCSRA	= (1<<ADPS2)| (1<<ADPS1) | (0<<ADPS0)	// set ADC prescaler to 64, 8MHz / 64 = 128kHz   
			| (1<<ADEN);							// Enable ADC
}


/*********************************************************************
*	Reads ADC channel (0-7) and returns a value (0-255)
**********************************************************************/
int GetADC_old(char ad_mux)
{	
	ADMUX &= ~0x1F;				// clear channel selection (low 5 bits)
	ADMUX |= ad_mux;			// select specified channel

	SetBit(ADCSRA, ADSC);		// ADC start conversion

	/* wait for conversion to complete */
	while (bit_is_clear(ADCSRA, ADIF))
		;

	return (ADC);  
}


/*********************************************************************
*	Reads ADC channel (0-7) and returns a value (0-255)
**********************************************************************/
int GetADC(char ad_mux)
{	
	int adc_sum = 0;
	
	ADMUX &= ~0x1F;				// clear channel selection (low 5 bits)
	ADMUX |= ad_mux;			// select specified channel

	for(int i = 1; i <= 8; i++) { 
		SetBit(ADCSRA, ADSC);		// ADC start conversion

		/* wait for conversion to complete */
		while (bit_is_clear(ADCSRA, ADIF))
			;
			
		adc_sum += ADC;
	}

	return (adc_sum / 8);  
}


/************************************************************************************
*		Timer/Counter 0																*																			*					
************************************************************************************/
static volatile unsigned long s_msTimer = 0;

void InitTimer()
{
	TCNT0 = 0;		// start counting at 0

	// clock rate = 8Mhz/64 = 128kHz  
	// CTC (Clear Timer on Compare Match) waveform generation mode
	// OCR0 (top/clear value) = ~127 for match every 1ms

	TCCR0A = (1<<WGM01) | (0<<CS02) | (1<<CS01) | (1<<CS00);  //CTC, clkio/64  128kHz   
	OCR0A = 125;  // set to 1ms     

    TIMSK0 = (1<<OCIE0A);	// set Compare Match Interrupt Enable for timer/counter 0
	sei();					// set I-bit in SREG, global enable for interrupts
}


// returns number of ms since Timer started
unsigned long TimerMsCur()
{
	char sregSav;
	unsigned long ms;

	/* disable (delay) interrupts while we read
	   s_msTimer since it's 4 bytes and we don't
	   want it changing during the read! */
	sregSav = SREG;	// cache register holding Global Interrupt Flag
	cli();			// clear Global Interrupt Flag
	ms = s_msTimer;
	SREG = sregSav;	// restore register holding Global Interrupt Flag

	return ms;
}

/* returns true if the specified number of milliseconds
   has passed since the start time */
char TimerCheck(unsigned long msStart, unsigned int msWait)
{
	return TimerMsCur() - msStart > msWait;
}

/* pauses for the specified number of milliseconds */
void TimerWait(unsigned int ms)
{
	unsigned long msStart;

	msStart = TimerMsCur();
	while (!TimerCheck(msStart, ms))
		;
}

/* this interrupt happens when TCNT0 reaches OCR0 */
ISR(TIMER0_COMP_vect)
//ISR(SIG_OUTPUT_COMPARE0)
{
	++s_msTimer;
	/* toggle Pin B0 for Oscilloscope Timings */
//	(PINB & 0x01) ? ClearBit(PORTB, PB0): SetBit(PORTB, PB0);	// used to check mstimer on O'scope
}


// Disable JTAG port, making PF[4-7] available for other use.
void disable_JTAG(void)
{
    uint8_t save_status = SREG;   // save flags, in particular the global int enable flag
    cli();                        // make sure int disabled
    MCUCR |= (1<<JTD);            // set jtag disable bit in MCU control reg
    MCUCR |= (1<<JTD);            // need to set twice in row for it to take effect (hope gcc won't optimize out...)
    SREG = save_status;           // restore global int enable to whatever it was before.
}

/*****************************************************************************
*
*   Function name : OSCCAL_calibration
*
*   Returns :       None
*
*   Parameters :    None
*
*   Purpose :       Calibrate the internal OSCCAL byte, using the external 
*                   32,768 kHz crystal as reference.
*                   NOTE: The internal RC clock is automatically calibrated
*                   to be within 10% of 8MHz.  This should make it within 2%.
*                   iow, without OSCCAL it should be between 7.2  & 8.8  MHz,
*                      and after OSCCAL it should be between 7.84 & 8.16 MHz.
*
*****************************************************************************/
void OSCCAL_calibration(void)
{
    unsigned char calibrate = FALSE;
    int temp;
    unsigned char tempL;
	
    CLKPR = (1<<CLKPCE);                // set Clock Prescaler Change Enable
    CLKPR = (1<<CLKPS1) | (1<<CLKPS0); // set prescaler = 8
 
    TIMSK2 = 0;             //disable OCIE2A and TOIE2

    ASSR = (1<<AS2);        //select asynchronous operation of timer2 (32,768kHz)
    
    OCR2A = 200;            // set timer2 compare value 

    TIMSK0 = 0;             // delete any interrupt sources
        
    TCCR1B = (1<<CS10);     // start timer1 with no prescaling
    TCCR2A = (1<<CS20);     // start timer2 with no prescaling

    while((ASSR & 0x01) | (ASSR & 0x04));       //wait for TCN2UB and TCR2UB to be cleared

    Delay(1000);    // wait for external crystal to stabilise
    
    while(!calibrate)
    {
        cli(); // mt __disable_interrupt();  // disable global interrupt
        
        TIFR1 = 0xFF;   // delete TIFR1 flags
        TIFR2 = 0xFF;   // delete TIFR2 flags
        
        TCNT1H = 0;     // clear timer1 counter
        TCNT1L = 0;
        TCNT2 = 0;      // clear timer2 counter
           
        while ( !(TIFR2 & (1<<OCF2A)) );   // wait for timer2 compareflag

        TCCR1B = 0; // stop timer1

        sei(); // enable global interrupt
    
        if ( (TIFR1 & (1<<TOV1)) )
        {
            temp = 0xFFFF;      // if timer1 overflows, set the temp to 0xFFFF
        }
        else
        {   // read out the timer1 counter value
            tempL = TCNT1L;
            temp = TCNT1H;
            temp = (temp << 8);
            temp += tempL;
        }
    
        if (temp > 6250)
        {
            OSCCAL--;   // the internRC oscillator runs to fast, decrease the OSCCAL
        }
        else if (temp < 6120)
        {
            OSCCAL++;   // the internRC oscillator runs to slow, increase the OSCCAL
        }
        else
            calibrate = TRUE;   // the interRC is correct

        TCCR1B = (1<<CS10); // start timer1
    }

    CLKPR = (1<<CLKPCE);                 // set Clock Prescaler Change Enable
	CLKPR = (0<<CLKPS1) | (0<<CLKPS0);  // 8Mhz=0, 4Mhz=1, 2Mhz=2, 1Mhz=3
}


// This is used by OSCCAL
void Delay(unsigned int millisec)
{
	uint8_t i;

	while (millisec--) {
		for (i=0; i<125; i++) {
			asm volatile ("nop"::);
		}
	}
}


void Initialization(void)
{   
    OSCCAL_calibration();	// Calibrate the OSCCAL byte and set clock to 8MHz

    InitTimer();			// Initialize 1ms timer
	
    ACSR = (1<<ACD);		// ACD = analog comparator disable bit
    DIDR0 = (7<<ADC0D);	// Disable Digital input on PF0-2 (power save)
 
    Button_Init();			// Initialize pin change interrupt on joystick
    LCD_Init();				// initialize the LCD
	disable_JTAG();			// Disable JTAG port, making pins available use. 
}

