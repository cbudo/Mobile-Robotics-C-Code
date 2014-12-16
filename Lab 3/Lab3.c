/*
* Lab3.c
*
* Created: 12/15/2014 2:34:13 PM
*  Author: budocf
*/


#include "capi324v221.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


//IR functions
void getIR();
float getLeftIR();
float getRightIR();
float getFrontIR();
float getBackIR();
// movement functions
void rotateDeg(float);
void forward(float);
// Navigation functions
void avoidObstacle();
void goToGoal();


// CONSTANTS
#define C_CCW 1.53833	// Steps per degree
#define C_CW 1.56167    // Steps per degree
#define C_FWD_LW 18.80  // Steps per inch
#define C_FWD_RW 18.84  // Steps per inch
#define ftIR 0.94	// front IR
#define bkIR 0.89	// Back IR  -  multiply by this
#define rtIR 0.92	// Right IR
#define ltIR 0.92	// Left IR


// Global Variables
double roboangle=0.0;
double roboxpos=0.0;
double roboypos=0.0;
int loopState = 0;
int xpos = 0;
int ypos=0;
int relangle=0;
int goType = 0;
int angle = 0;

float move = 3;  // number of inches to move forward between "thoughts"

struct IR
{
	float ft = 0.0;
	float lt = 0.0;
	float rt = 0.0;
	float bk = 0.0;
};

int main(void)
{
	STEPPER_open(); // Enables control of the steppers
	ATTINY_open(); // Enables reading of the buttons
	SPKR_open(SPKR_BEEP_MODE); //enables beeping
	LCD_open(); // Enables use of the LCD
	LCD_clear(); // Clears the LCD screen
	getIR();
	while(1)
	{
		//TODO:: Please write your application code
	}
}

void getIR()
{
	IR.ft = getFrontIR();
	IR.bk = getBackIR();
	IR.rt = getRightIR();
	IR.lt = getLeftIR();
}

void forward(float dist)
{
	// Moves forwards or backwards a distance "dist" given in inches.
	int Lsteps = ceil(C_FWD_LW*dist);
	int Rsteps = ceil(C_FWD_RW*dist);
	
	if(dist>0){
		//Move Forwards
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_FWD, Lsteps, 200, 0, STEPPER_BRK_OFF,
		STEPPER_FWD, Rsteps, 200, 0, STEPPER_BRK_OFF );
		}else if(dist<0){
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_REV, -1*Lsteps, 200, 0, STEPPER_BRK_OFF,
		STEPPER_REV, -1*Rsteps, 200, 0, STEPPER_BRK_OFF );
	}
	// Update predicted position variables
	roboxpos += cos(roboangle*M_PI/180);
	roboypos += sin(roboangle*M_PI/180);
	//updateLCD();
}
void rotateDeg(float degree)
{
	// Rotates a given angle in degrees (positive is CCW)
	if(degree > 0){
		// Rotate CCW
		int steps = ceil(C_CCW*degree);
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_REV, steps, 200, 0, STEPPER_BRK_OFF,
		STEPPER_FWD, steps, 200, 0, STEPPER_BRK_OFF );
		}else if(degree < 0){
		//Rotate CW
		int steps = (-1)*round(C_CW*degree);
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_FWD, steps, 200, 0, STEPPER_BRK_OFF,
		STEPPER_REV, steps, 200, 0, STEPPER_BRK_OFF );
	}
	// Update predicted position variable
	roboangle += degree;
	//updateLCD();
}

// getLeftIR() converts ADC voltage to inches
float getLeftIR(){
	//float voltage;//IR range -0.4 to 5.3 V
	float distance;// (cm) 30 cm = 12 inches = 0.4 V
	float dist;//distance in inches
	ADC_SAMPLE adcsample;
	// Set the Voltage Reference first so VREF=5V.
	ADC_set_VREF( ADC_VREF_AVCC );
	// Set the channel we will sample from.
	ADC_set_channel( IRLEFT_CHAN );
	// Now sample it!
	adcsample = ADC_sample();
	//LCD_printf( "ADC: %i\n",adcsample);
	// Convert to meaningful voltage value.
	//voltage = adcsample * ( 5.0 / 1024 );
	// Convert to distance in cm
	distance = (2914/(adcsample+5.0))-1.0;
	//Convert distance to inches
	dist = distance*0.3937;
	return dist;
}

// getRightIR() converts ADC voltage to inches
float getRightIR(){
	//float voltage;//IR range -0.4 to 5.3 V
	float distance;// (cm) 30 cm = 12 inches = 0.4 V
	float dist;//distance in inches
	ADC_SAMPLE adcsample;
	// Set the Voltage Reference first so VREF=5V.
	ADC_set_VREF( ADC_VREF_AVCC );
	// Set the channel we will sample from.
	ADC_set_channel( IRRIGHT_CHAN );
	// Now sample it!
	adcsample = ADC_sample();
	// Convert to meaningful voltage value.
	//voltage = adcsample * ( 5.0 / 1024 );
	// Convert to distance in cm
	distance = (2914/(adcsample+5.0))-1.0;
	//Convert distance to inches
	dist = distance*0.3937;
	return dist;
}

// getFrontIR() simply sets the ADC to the Front IR sensor
// and returns the sampled voltage in inches. Short, nonblocking.
float getFrontIR(){
	//1 inch = 2.54 cm
	//1 cm = 0.3937 inches
	//float voltage;//IR range -0.4 to 5.3 V
	float distance;// (cm) 30 cm = 12 inches = 0.4 V
	float dist;//distance in inches
	ADC_SAMPLE adcsample;
	// Set the Voltage Reference first so VREF=5V.
	ADC_set_VREF( ADC_VREF_AVCC );
	// Set the channel we will sample from.
	ADC_set_channel(IRFRONT_CHAN);
	// Now sample it!
	adcsample = ADC_sample();
	// Convert to meaningful voltage value.
	//voltage = adcsample * ( 5.0 / 1024 );
	// Convert to distance in cm
	distance = (2914/(adcsample+5.0))-1.0;
	//Convert distance to inches
	dist = distance*0.3937;
	return dist;
}

// getBackIR() simply sets the ADC to the Back IR sensor
// and returns the sampled voltage in inches. Short, nonblocking.
float getBackIR(){
	//1 inch = 2.54 cm
	//1 cm = 0.3937 inches
	//float voltage;//IR range -0.4 to 5.3 V
	float distance;// (cm) 30 cm = 12 inches = 0.4 V
	float dist;//distance in inches
	ADC_SAMPLE adcsample;
	// Set the Voltage Reference first so VREF=5V.
	ADC_set_VREF( ADC_VREF_AVCC );
	// Set the channel we will sample from.
	ADC_set_channel(IRBACK_CHAN);
	// Now sample it!
	adcsample = ADC_sample();
	// Convert to meaningful voltage value.
	//voltage = adcsample * ( 5.0 / 1024 );
	// Convert to distance in cm
	distance = (2914/(adcsample+5.0))-1.0;
	//Convert distance to inches
	dist = distance*0.3937;
	return dist;
}

void gotToGoal(){
	
}
void avoidObstacle(){
	
}
