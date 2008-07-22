// Button.h

///#define PINB_MASK ((1<<PINB4)|(1<<PINB6)|(1<<PINB7))
#define PINB_MASK ((1<<PINB4)|(1<<PINB7))
#define PINE_MASK ((1<<PINE2)|(1<<PINE3))

#define BUTTON_A    6   // UP
#define BUTTON_B    7   // DOWN
#define BUTTON_C    2   // LEFT
#define BUTTON_D    3   // RIGHT
#define BUTTON_O    4   // center

//Button definitions
#define KEY_NULL    0   // no key
#define KEY_ENTER   1   // center
#define KEY_NEXT    2   // right
#define KEY_PREV    3   // left
#define KEY_PLUS    4   // up
#define KEY_MINUS   5   // down

#define js_center 0x10  // port B
#define js_up     0x40  // port B
#define js_down   0x80  // port B
#define js_left   0x04  // port E
#define js_right  0x08  // port E

// PROTOTYPES:
void PinChangeInterrupt(void);
void Button_Init(void);
char getkey(void);
////char ButtonBouncing(void);
