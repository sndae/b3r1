///////////////////////////////////////////////////////////////////////
//////////////////////                         ////////////////////////
//////////////////////       parameters.h      ////////////////////////
//////////////////////                         ////////////////////////
///////////////////////////////////////////////////////////////////////
//#include "globals.h"
//
//		Parameters required for operation and tuning go here
//
//
//#define 	CPUCLK_PARAM   		16000000		//	The speed of the cpu currently in use
//												//	Also needs to be changed in FreeRTOSConfig.h
//
//
//	The FreeRTOS task stack size can be very important, especially with tasks
//	which perform a lot of floating point operations and perhaps which 
//	have many nested calls with FP Operations.  Size this carefully.
//#define		stackSize_Param		128				// number of variables on stack (plus minimum)
//
//
//	Enable or disable I2C.  App will freeze if enabled and I2C device NOT attached
//#define		I2C_OFF								//	Enable or disable I2C
//
//	Choose specific LCD
//#define		TXT_LCD								//	Text LCD
//#define		GRH_LCD								//	Graphics LCD
//
//
//#define		IGNORE_START_BUTTON					//	if set, start imediately, otherwise wait for start button
//#define		CountDown_PARAM		4				//	number of half seconds to delay after start button
//
//
///////////////////////////////////////////////////////////////////////
//	Application Specific Parameters
///////////////////////////////////////////////////////////////////////
//
//	Balancing Bot 
//
//
//	dt_PARAM is how often the Kalman state is updated with
//	gyro rate measurements.  You will have to
//	change this value if you update at a different rate.
//	For example, the 50Hz is based upon the call frequency
//	of balance() from make_it_balance().
//	If xFrequency is changed in make_inBalance() then this
//	should be changed and vice versa.
//
//	40Hz Rate for debugging because of so many rprintfs 
#define		dt_PARAM		0.100		//	10 Hz rate
//#define		dt_PARAM		0.025		//	40 Hz rate
//#define		dt_PARAM		0.02		//	50 Hz rate

//	R may be used for tuning Kalman
//	R represents the measurement covariance noise.  In this case,
//	it is a 1x1 matrix that says that we expect 0.3 rad jitter
//	from the accelerometer.
//
//	Adjusting R changes the speed at which the filter converges.
//	It is an indication of trust in the measurement
#define		R_angle_PARAM		.2			//	n rad jitter  / .3


//	Q may be used for tuning Kalman
//	Q is a 2x2 matrix that represents the process covariance noise.
//	In this case, it indicates how much we trust the acceleromter
//	relative to the gyros.
//
// 	originally .001 and .003
#define		Q_angle_PARAM		0.001
#define		Q_gyro_PARAM		0.001


// These are measured values.  They will change with sensor 
// orientation, so be careful to change these when necessary.
//
// value at 0 degrees / sec - unimportant, as the Kalman filter corrects this
//#define 	GYRO_OFFSET 		613		
#define 	GYRO_OFFSET 		580
		
// value at 0 degrees  / 1 unit .4 degees  (move to +90 -90 / 2)
//#define 	ACCEL_OFFSET 		650	
#define 	ACCEL_OFFSET 		540	


// gyro is 2 mv/(deg/sec), ADC is 2.5mv/tick: 2/2.5 ; degrees/sec = adc * .8 
// gyro is 2 mv/(deg/sec), ADC is 3.22mv/tick: 2/3.3  ; degrees/sec = adc * .62
// ADXRS401 gyro is 15mv/(deg/sec), ADC is 4.88mv/bit: 15/4.88  ; degrees/sec = adc * 3.07
#define 	GYRO_SCALE 			3.07


// ADC bits per 90 degrees
#define 	ACCEL_SCALE 		0x0d8	
//#define 	ACCEL_SCALE 		106.0	


//	Neutral or Natural balance angle
//	You might change this by .1 at a time until tuned    
//#define	neutral_PARAM 	-7.7		// "angle of natural balance"  (set by looking at AN on the LCD)
#define		neutral_PARAM 	-2			// "angle of natural balance"  (set by looking at AN on the LCD)


//	Motor Bias - left and right
#define		lmb_PARAM 			128			//	PWM required to get the motor turning
#define 	rmb_PARAM 			128			//	removes deadband at low power


// 	PID adjustable gains
//
//	Proportional Constant
#define		Kp_PARAM 			200			// balance loop P gain

//	Differential Constant
//	You might change this by .1 to .5 at a time until tuned // 12.5
//#define		Kd_PARAM	 		22		// balance loop D gain -- good for 8.4 volt lipol
#define		Kd_PARAM	 		12		// balance loop D gain -- good for 8.4 volt lipol


#define MAX_TORQUE	255
#define RAD_TO_DEG	(180 / 3.1415926535897932384626433832795)
