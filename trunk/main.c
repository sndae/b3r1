/*
 *	main.c
 */

// standard libs
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <inttypes.h>
#include <util/delay.h>

#include "main.h"
#include "LCD_driver.h"
#include "button.h"
#include "bbot_util.h"
#include "tilt.h"
#include "parameters.h"

#include "avrlibtypes.h"
#include "uart.h"
#include "vt100.h"
#include "rprintf.h"

// Motor control circuit is connected to PortB  
//							+---+---+---+---+---+
//            Port B		| 1 | 3 | 5 | 7 | V |  
//     Motor Contorl pins	+---+---+---+---+---+
//							| 0 | 2 | 4 | 6 | G |  
//							+---+---+---+---+---+
//
//	PB2 - Left Motor	FWD/REV		Yellow
//	PB3 - Right Motor	FWD/REV		Blue  
//	PB5 - Right Motor	PWM			Blue/Black
//	PB6 - Left Motor 	PWM			Yellow/Black 
//
//
// Analog sensors can be connected to Port F ADC4(PF4) thru ADC7(PF7). By default Port F
// is used for JTAG and such JTAG functionality must be disabled.
// The pins are NOT numbered the same as the other connectors.
//
//							+---+---+---+---+---+
//		Port F  (JTAG)		| x	| x | x | x | x |  Pins marked with "x" should not be used.  
//		Usable Pins			+---+---+---+---+---+
//							| 7	| x	| 5	| 6 | 4	|  
//                          +---+---+---+---+---+

#define rightLineSensor		4		//PF4 - Bottom Right pin
#define leftLineSensor		5		//PF5 - 2 to the left of PF4	
#define RightDistSensor		6		//PF6 - 1 to the left of PF4 	
#define LeftDistSensor		7		//PF7 - Bottom Left pin 

#define gyroRawSensor		4
#define accelRawSensor		5

// Balance
float balance_torque;
float overspeed;
float overspeed_integral;
float gyro_integrated;

//	Proportional Konstant
float Kp = Kp_PARAM;				// balance loop P gain
//	Differentiate konstant
float Kd = Kd_PARAM;				// balance loop D gain

//float Ksteer;						// steering control gain
//float Ksteer2;					// steering control tilt sensitivity
//float Kspeed; 					// speed tracking gain
float neutral = neutral_PARAM;		// "angle of natural balance"


//float KpTurn;						// turn rate loop P gain
//float KiTurn;						// turn rate loop I gain
//float KdTurn;						// turn rate loop D gain

// Outputs
int left_motor_torque;
int right_motor_torque;

volatile unsigned int accelRaw;
volatile unsigned int gyroRaw;

//volatile unsigned int potRaw;

//volatile float accel;
//volatile float gyro;
//volatile float pot;

float current_angle = 0;
float current_rate = 0;

extern volatile char KEY_VALID;

// Variable from "button.c" to prevent button-bouncing
extern unsigned char gButtonTimeout;

int main (void) 
{ 
    Initialization();

    show_mhz();				// Display CPU speed
    TimerWait(1000);		// Wait 1 second
	
    while(1) 
	{
		printLCD("Show ADC");			// Display the text in quotes on the LCD
        while (!KEY_VALID);				// Wait for joystick to be moved or pressed.
        if (getkey() == 1)				// If enter was pressed then do what is in the braces, just skip over it. 
		{   
			TimerWait(500);				// debounce joystick button
            showADC();						                   
			printLCD("Back ADCs");
            TimerWait(2000);
		}
  
        printLCD("Balance");			// Display the text in quotes on the LCD
        while (!KEY_VALID);				// Wait for joystick to be moved or pressed.
        if (getkey() == 1)				// If enter was pressed then do what is in the braces, else just continue.
		{	
			TimerWait(500);				// debounce joystick button
			balance();                  
			printLCD("Back Balance");
            TimerWait(2000);
		}
  
        printLCD("rprintf");			// Display the text in quotes on the LCD
        while (!KEY_VALID);				// Wait for joystick to be moved or pressed.
        if (getkey() == 1)				// If enter was pressed then do what is in the braces, else just continue.
		{	
			TimerWait(500);				// debounce joystick button
			rprintf_test();                  
			printLCD("Back rprintf");
            TimerWait(2000);
		}
  
        printLCD("PWM Test");			// Display the text in quotes on the LCD
        while (!KEY_VALID);				// Wait for joystick to be moved or pressed.
        if (getkey() == 1)				// If enter was pressed then do what is in the braces, else just continue.
        {
			TimerWait(500);				// debounce joystick button
            PWM_Test();
			printLCD("Back PWM Test");
            TimerWait(3000);
        }
    }   
}  


/*****************************************************************************
 *  showADC - Displays ADC reading on the LCD display. 
 *****************************************************************************/  
void showADC(void) 
{
    int adcL, adcR = 0;
   
    InitADC();
   
    while (!(getkey() == 1))
	{	
        adcL = GetADC(leftLineSensor);
        adcR = GetADC(rightLineSensor);
        TimerWait(10);
        show12bits(adcL, adcR);
    }
	LCD_ShowColons(0);
}

/*****************************************************************************
 *	Balance - 
 *****************************************************************************/  
void balance(void)
{
	unsigned long TimerMsWork;
	
	InitADC();
	init_pwm();

	// initialize the UART (serial port)
	uartInit();
	
	// set the baud rate of the UART for our debug/reporting output
	uartSetBaudRate(38400);

	// initialize rprintf system
	rprintfInit(uartSendByte);

	// initialize vt100 library
	vt100Init();

	// clear the terminal screen
	vt100ClearScreen();

	TimerMsWork = TimerMsCur();
	
	DDRB |= (1 << PB0);	// Make B0 an output

    while (!(getkey() == 1))
	{
		/* insure loop runs at specified Hz */
		while (!TimerCheck(TimerMsWork, (dt_PARAM * 1000) -1))
			;
		TimerMsWork = TimerMsCur();
		
		// toggle pin B0 for oscilloscope timings.
		PORTB = PINB ^ (1 << PB0);
		
		gyroRaw = GetADC(gyroRawSensor);
		accelRaw = GetADC(accelRawSensor);
		
		rprintf("gyroRaw: %d  accelRaw: %d\r\n",gyroRaw, accelRaw);
//		show12bits(gyroRaw, accelRaw);

		// no scaling needed.. but I'm doing it anyway
//		gyro = scale_gyro(gyroRaw);
//		accel = scale_accel(accelRaw);

		// Have measurement, find error and update K
		kalman_update(accelRaw);

		// Produce new current angle and rate estimate
		// real units assumed (they're pretty close)
		state_update((float)(int)( gyroRaw - GYRO_OFFSET));

		// If tilt is positive, move wheels forward to balance;
		// + cmds are cw (right side, fwd)
		//				 (left side, bkwd)
		// So left side gets - cmds

		// This is just a casualty of the sensor mounting
		current_angle = angle;
		current_rate = rate;
		rprintf("angle: ");
		rprintfFloat(8, angle);
		rprintf("  rate: ");
		rprintfFloat(8, rate);
		rprintfCRLF();

		// Balance.  The most important line in the entire program.
		balance_torque = Kp * (current_angle - neutral) + Kd * current_rate;
		rprintf("bal_torq: ");
		rprintfFloat(8, balance_torque);
		rprintfCRLF();

		//steer_knob = 0;

		// change from current angle to something proportional to speed
		// should this be the abs val of the cur speed or just curr speed?
		//float steer_cmd = (1.0 / (1.0 + Ksteer2 * fabs(current_angle))) * (Ksteer * steer_knob);
		float steer_cmd = 0.0;

		// Get current rate of turn
		//float current_turn = left_speed - right_speed; //<-- is this correct
		//float turn_accel = current_turn - prev_turn;
		//prev_turn = current_turn;

		// Closed-loop turn rate PID
		//float steer_cmd = KpTurn * (current_turn - steer_desired)
		//					+ KdTurn * turn_accel;
		//					//+ KiTurn * turn_integrated;

		// Possibly optional
		//turn_integrated += current_turn - steer_cmd;

		//	Differential steering
		left_motor_torque	= balance_torque + steer_cmd; //+ cur_speed + steer_cmd;
		right_motor_torque	= balance_torque - steer_cmd; //+ cur_speed - steer_cmd;


		// Limit extents of torque demand
		//left_motor_torque = flim(left_motor_torque, -MAX_TORQUE, MAX_TORQUE);
//		if (left_motor_torque < -MAX_TORQUE) left_motor_torque = -MAX_TORQUE;
//		if (left_motor_torque > MAX_TORQUE)  left_motor_torque =  MAX_TORQUE;

		//right_motor_torque = flim(right_motor_torque, -MAX_TORQUE, MAX_TORQUE);
//		if (right_motor_torque < -MAX_TORQUE) right_motor_torque = -MAX_TORQUE;
//		if (right_motor_torque > MAX_TORQUE)  right_motor_torque =  MAX_TORQUE;

		// Set PWM values for both motors
		if (fabs(current_angle) < 45) 
		{	//	if bot has not fallen over then go 
			SetLeftMotorPWM(left_motor_torque);
			SetRightMotorPWM(right_motor_torque);
		} 
		else 
		{	// Otherwise stop motors
			SetLeftMotorPWM(0);
			SetRightMotorPWM(0);
		}
	}
	SetLeftMotorPWM(0);
	SetRightMotorPWM(0);
}



/*******************************************************************
*		rprintf_test
*******************************************************************/  
void rprintf_test(void)
{
	u16 val;
	u08 mydata;
	u08 mystring[10];
	float b;
	u08 small;
	u16 medium;
	u32 big;

	// initialize the UART (serial port)
	uartInit();
	
	// set the baud rate of the UART for our debug/reporting output
	uartSetBaudRate(38400);

	// initialize rprintf system
	// - use uartSendByte as the output for all rprintf statements
	//   this will cause all rprintf library functions to direct their
	//   output to the uart
	// - rprintf can be made to output to any device which takes characters.
	//   You must write a function which takes an unsigned char as an argument
	//   and then pass this to rprintfInit like this: rprintfInit(YOUR_FUNCTION);
	rprintfInit(uartSendByte);

	// initialize vt100 library
	vt100Init();

	// clear the terminal screen
	vt100ClearScreen();

    while (!(getkey() == 1))		//do the folling block until enter is pressed
	{
	
		// print a little intro message so we know things are working
		rprintf("\r\nWelcome to rprintf Test!\r\n");

		// print single characters
		rprintfChar('H');
		rprintfChar('e');
		rprintfChar('l');
		rprintfChar('l');
		rprintfChar('o');
		// print a constant string stored in FLASH
		rprintfProgStrM(" World!");
		// print a carriage return, line feed combination
		rprintfCRLF();
		// note that using rprintfCRLF() is more memory-efficient than
		// using rprintf("\r\n"), especially if you do it repeatedly

		mystring[0] = 'A';
		mystring[1] = ' ';
		mystring[2] = 'S';
		mystring[3] = 't';
		mystring[4] = 'r';
		mystring[5] = 'i';
		mystring[6] = 'n';
		mystring[7] = 'g';
		mystring[8] = '!';
		mystring[9] = 0;	// null termination

		// print a null-terminated string from RAM
		rprintfStr(mystring);
		rprintfCRLF();

		// print a section of a string from RAM
		// - start at index 2
		// - print 6 characters
		rprintfStrLen(mystring, 2, 6);
		rprintfCRLF();


		val = 24060;
		mydata = 'L';

		// print a decimal number
		rprintf("This is a decimal number: %d\r\n", val);

		// print a hex number
		rprintf("This is a hex number: %x\r\n", mydata);
	
		// print a character
		rprintf("This is a character: %c\r\n", mydata);

		// print hex numbers
		small = 0x12;		// a char
		medium = 0x1234;	// a short
		big = 0x12345678;	// a long

		rprintf("This is a 2-digit hex number (char) : ");
		rprintfu08(small);
		rprintfCRLF();

		rprintf("This is a 4-digit hex number (short): ");
		rprintfu16(medium);
		rprintfCRLF();

		rprintf("This is a 8-digit hex number (long) : ");
		rprintfu32(big);
		rprintfCRLF();

		// print a formatted decimal number
		// - use base 10
		// - use 8 characters
		// - the number is signed [TRUE]
		// - pad with '.' periods
		rprintf("This is a formatted decimal number: ");
		rprintfNum(10, 8, TRUE, '.', val);
		rprintfCRLF();

		b = 1.23456;

		// print a floating point number
		// use 10-digit precision
	
		// NOTE: TO USE rprintfFloat() YOU MUST ENABLE SUPPORT IN global.h
		// use the following in your global.h: #define RPRINTF_FLOAT

		rprintf("This is a floating point number: ");
		rprintfFloat(8, b);
		rprintfCRLF();
	}
}


/******************************************************* 
*	PWM test - check wiring and such
*******************************************************/ 
void PWM_Test(void) 
{
	init_pwm();
	
    while (!(getkey() == 1)) 
	{
	
		for (int i = 0; i <= 255; i++) {
	        show12bits(i, i);
			SetLeftMotorPWM(i);
			SetRightMotorPWM(i);
			TimerWait(100);
		}
			
/*
			
	
 	    printLCD("LFWD");
	    SetLeftMotorPWM(255);
        SetRightMotorPWM(0);
        TimerWait(3000);
 
        printLCD("LREV");
	    SetLeftMotorPWM(-255);
        SetRightMotorPWM(0);
        TimerWait(3000);
    
        printLCD("RFWD");
	    SetLeftMotorPWM(0);
        SetRightMotorPWM(255);
        TimerWait(3000);
	
        printLCD("RREV");
	    SetLeftMotorPWM(0);
        SetRightMotorPWM(-255);
        TimerWait(3000);
	
        printLCD("2 FWD low");
		SetRightMotorPWM(128);
	    SetLeftMotorPWM(128);
	    TimerWait(3000);

        printLCD("2 REV med");
	    SetRightMotorPWM(-200);
	    SetLeftMotorPWM(-200);
	    TimerWait(3000);
    
        printLCD("CCW");
		SetRightMotorPWM(200);
	    SetLeftMotorPWM(-200);
	    TimerWait(3000);

        printLCD("CW");
	    SetRightMotorPWM(-200);
	    SetLeftMotorPWM(200);
	    TimerWait(3000);
*/
	}
    // Done
	SetLeftMotorPWM(0);
	SetRightMotorPWM(0);
}
