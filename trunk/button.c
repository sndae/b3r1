//*****************************************************************************
//
//  File........: button.c
//  Author(s)...: ATMEL Norway
//  Target(s)...: ATmega169
//  Compiler....: AVR-GCC 3.4.5; avr-libc 1.4.3
//  Description.: AVR Butterfly button handling routines
//  Revisions...: 1.0
//
//  YYYYMMDD - VER. - COMMENT                                       - SIGN.
//
//  20030116 - 1.0  - Created                                       - KS
//  20031009          port to avr-gcc/avr-libc                      - M.Thomas
//  20060128          modifications for gcc 3.4.5, avr-libc 1.4.3   - M.Thomas
//
//*****************************************************************************

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#include "bbot_util.h"
#include "button.h"
#include "avrlibtypes.h"


volatile unsigned char gButtonTimeout = TRUE;
volatile char KEY = NULL;
volatile char KEY_VALID = FALSE;

volatile char tbird = 0;

/*****************************************************************************
*
*   Function name : Button_Init
*   Returns :       None
*   Parameters :    None
*   Purpose :       Initializes the five button pin
*
*****************************************************************************/
void Button_Init(void)
{
    // Init port pins
    ClearBit(DDRB,7);      // joystick B is input
	ClearBit(DDRB,6);      // joystick A is input
    ClearBit(DDRB,4);      // joystick center is input
    PORTB |= PINB_MASK; // turn on pullups
	 
    DDRE = 0x00;        // port E is all input (joystick left & right are bits 2 &3)
    PORTE |= PINE_MASK; // turn on pullups

    // Enable pin change interrupt on PORTB and PORTE
	PCMSK0 = PINE_MASK;
	PCMSK1 = PINB_MASK;
	EIFR = (1<<PCIF0)|(1<<PCIF1);
	EIMSK = (1<<PCIE0)|(1<<PCIE1);

	// Clear getkey 
	getkey();	
}

ISR(PCINT0_vect)
{
    PinChangeInterrupt();
}

ISR(PCINT1_vect)
{
    PinChangeInterrupt();
}



/*****************************************************************************
*
*   Function name : PinChangeInterrupt
*   Returns :       None
*   Parameters :    None
*   Purpose :       Check status on the joystick
*
*****************************************************************************/
void PinChangeInterrupt(void)
{
    char buttons;

    char key;

/*
    Read the buttons:

    Bit             7   6   5   4   3   2   1   0
    ---------------------------------------------
    PORTB           B   A       O
    PORTE                           D   C
    ---------------------------------------------
    PORTB | PORTE   B   A       O   D   C
    =============================================
*/

	tbird++;

    buttons = (~PINB) & PINB_MASK;
    buttons |= (~PINE) & PINE_MASK;

    // Output virtual keys
    if (buttons & (1<<BUTTON_A))
        key = KEY_PLUS;
    else if (buttons & (1<<BUTTON_B))
        key = KEY_MINUS;
    else if (buttons & (1<<BUTTON_C))
        key = KEY_PREV;
    else if (buttons & (1<<BUTTON_D))
        key = KEY_NEXT;
    else if (buttons & (1<<BUTTON_O))
        key = KEY_ENTER;
    else
        key = KEY_NULL;

    
	// ignore any "key up's"
    if(key != KEY_NULL)
    {
	    // ignore any "down's" that come to close to last key.  
        if(gButtonTimeout)  
        {
            if (!KEY_VALID)
            {
                KEY = key;          // Store key in global key buffer
                KEY_VALID = TRUE;
                //if (gKeyClickStatus)
                //  PlayClick();
            }
             
            gButtonTimeout = FALSE;
           
        }
    }
    
    EIFR = (1<<PCIF1) | (1<<PCIF0);     // Delete pin change interrupt flags

 ////   gPowerSaveTimer = 0;                // Reset the Auto Power Down timer
    
}


/*****************************************************************************
*
*   Function name : getkey
*   Returns :       The valid key
*   Parameters :    None
*   Purpose :       Get the valid key 
*
*****************************************************************************/
char getkey(void)
{
    char k;

    cli();                      // disable interrupt

    if (KEY_VALID)              // Check for unread key in buffer
    {
        k = KEY;
        KEY_VALID = FALSE;
    }
    else
        k = KEY_NULL;           // No key stroke available

    sei();                      // enable interrupt
	
    return k;
}
