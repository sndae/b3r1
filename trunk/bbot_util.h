//*****************************************************************************	
//*	bbot_util.h
//*****************************************************************************	

// MACRO DEFINITIONS

// sbi and cbi are not longer supported by the avr-libc.
// To avoid version-conflicts the macro-names have been 
// changed to SetBit and ClearBit.
#define SetBit(sfr, bit) ((sfr) |= (1 << (bit)))
#define ClearBit(sfr, bit) ((sfr) &= ~(1 << (bit)))

//#define FALSE   0
//#define TRUE    (!FALSE)
#define NULL    0

// PROTOTYPES 
void init_pwm(void);
void SetLeftMotorPWM(int pwm);
void SetRightMotorPWM(int pwm);

void InitADC(void);
int GetADC(char adc);
int GetADC8(char adc);

float scale_accel(int rawAccel);
float scale_gyro(int rawGyro);

void InitTimer(void);
unsigned long TimerMsCur(void);
char TimerCheck(unsigned long msStart, unsigned int msWait);
void TimerWait(unsigned int ms);

void disable_JTAG(void);
void OSCCAL_calibration(void);
void Delay(unsigned int millisec);
void Initialization(void);
