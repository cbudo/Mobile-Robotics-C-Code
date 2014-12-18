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
void smartGoToGoal(float,float);
void getIR();
float getLeftIR();
float getRightIR();
float getBackIR();
float getFrontIR();
// movement functions
void rotateDeg(float);
void forward(float);
// Navigation functions
char avoidObstacle(float,float,float,float);
void goToGoal(int,int);
void updateLCD(void);
char aggressiveKid(short, float, float);
int randRange(int,int);
char shyGuy(short,float,float,float,float);
void wander(char);
void levelZero(char,float);
char wallFollow(char,float,float,float);


// CONSTANTS
#define C_CCW 1.53833	// Steps per degree
#define C_CW 1.56167    // Steps per degree
#define C_FWD_LW 18.80  // Steps per inch
#define C_FWD_RW 18.80  // Steps per inch
#define ftIR 0.94	// front IR
#define bkIR 0.89	// Back IR  -  multiply by this
#define rtIR 0.92	// Right IR
#define ltIR 0.92	// Left IR

#define IRRIGHT_CHAN ADC_CHAN3
#define IRLEFT_CHAN ADC_CHAN4
#define IRFRONT_CHAN ADC_CHAN7
#define IRBACK_CHAN ADC_CHAN5

#define SPEED 100  // Speed for motor movements


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
//global declarations here
extern signed int samplevar;

float move = 0.25;  // number of feet to move forward between "thoughts"
float ft=0;
float lt=0;
float rt=0;
float bk=0;
short frontRight = 0;
short frontLeft = 0;
// IR Sensor Results


void CBOT_main(void)
{
	STEPPER_open(); // Enables control of the steppers
	ATTINY_open(); // Enables reading of the buttons
	SPKR_open(SPKR_BEEP_MODE); //enables beeping
	LCD_open(); // Enables use of the LCD
	ADC_open();//open the ADC module
	
	
	while(1)
	{
		if(ATTINY_get_SW_state(ATTINY_SW3))
		{
			smartGoToGoal(4,4);
		}
		if(ATTINY_get_SW_state(ATTINY_SW4))
		{
			wander('a');
		}
		if (ATTINY_get_SW_state(ATTINY_SW5))
		{
			char wall = 'l';
			while(wall=='l'){
				wall = wallFollow('l',4,5,10);
			}
			
		}
		//TODO:: Please write your application code
	}
}
void smartGoToGoal(float x, float y)
{
	xpos=x;
	ypos=y;
	while((roboxpos<=(x-.25)||roboxpos>=(x+.25))&&(roboypos<=(y-.25)||roboypos>=(y+.25)))
	{
		avoidObstacle(7,5,5,5);
		updateLCD();
	}
}
char avoidObstacle(float front, float right, float left, float back){
	getIR();                // Reads sensors

	if (frontRight && !frontLeft)
	{
		// Turn left
		rotateDeg(45);
		return 'f';
	}
	else if (frontLeft && !frontRight)
	{
		// Turn right
		rotateDeg(-45);
		return 'f';
	}
	else if (ft <= front || (frontRight && frontLeft))       // Checks left buffer
	{
		float leftfront;
		float rightfront;
		rotateDeg(30);
		getIR();
		leftfront = ft;
		rotateDeg(-60);
		getIR();
		rightfront = ft;
		rotateDeg(30);
		
		if(leftfront >= rightfront)
		{
			rotateDeg(90);
			forward(move);
			return 'f';
		}
		else
		{
			rotateDeg(-90);
			forward(move);
			return 'f';
		}
	}
	else if (rt <= right)
	{
		rotateDeg(30);
		forward(move);
		return 'r';
	}
	else if (lt <= left)
	{
		rotateDeg(-30);
		forward(move);
		return 'l';
	}else if (bk <= back)
	{
		forward(move);
		return 'b';
	}else
	{
		// rotate toward goal
		int xdelta = xpos - roboxpos;
		int ydelta = ypos - roboypos;
		angle = atan2(ydelta,xdelta)*180/M_PI - roboangle;
		rotateDeg(angle);
		forward(.5);              // Moves forward
		return 'w';
	}
}
void levelZero(char type,float frontDist)
{
	if(type == 'a')
	{
		while(1){
			char robostate='w';
			updateLCD();
			while (robostate=='w')
			{
				robostate = aggressiveKid(0,6,3);
				getIR();
				updateLCD();
				TMRSRVC_delay(100);
			}
			LCD_clear();
			LCD_printf("Reason for Stop: %c",robostate);
			if(ATTINY_get_SW_state(ATTINY_SW3))
			break;
		}
		
	}
}
void wander(char type)
{
	if(type == 'a')
	{
		while(1){
			char robostate='w';
			updateLCD();
			while (robostate=='w')
			{
				robostate = aggressiveKid(1,6,3);
				getIR();
				updateLCD();
				TMRSRVC_delay(100);
			}
			LCD_clear();
			LCD_printf("Reason for Stop: %c",robostate);
			if(ATTINY_get_SW_state(ATTINY_SW3))
			break;
		}
	}
	if(type=='s')
	{
		while(1){
			char robostate='w';
			updateLCD();
			while (robostate=='w')
			{
				robostate = shyGuy(1,7,4,4,6);
				getIR();
				updateLCD();
				TMRSRVC_delay(100);
			}
			LCD_clear();
			LCD_printf("Reason for Stop: %c",robostate);
			if(ATTINY_get_SW_state(ATTINY_SW3))
			break;
		}
	}
	
}
void getIR()
{
	ft = getFrontIR();
	bk = getBackIR();
	rt = getRightIR();
	lt = getLeftIR();
	frontRight = ATTINY_get_IR_state(ATTINY_IR_RIGHT);
	frontLeft =  ATTINY_get_IR_state(ATTINY_IR_LEFT);
}

void forward(float dist)
{
	// Moves forwards or backwards a distance "dist" given in feet.
	int Lsteps = ceil(C_FWD_LW*dist)*12;
	int Rsteps = ceil(C_FWD_RW*dist)*12;
	
	if(dist>0){
		//Move Forwards
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_FWD, Lsteps, SPEED, 0, STEPPER_BRK_OFF,
		STEPPER_FWD, Rsteps, SPEED, 0, STEPPER_BRK_OFF );
		}else if(dist<0){
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_REV, -1*Lsteps, SPEED, 0, STEPPER_BRK_OFF,
		STEPPER_REV, -1*Rsteps, SPEED, 0, STEPPER_BRK_OFF );
	}
	// Update predicted position variables
	roboxpos += dist * cos(roboangle*M_PI/180);
	roboypos += dist * sin(roboangle*M_PI/180);
}
void rotateDeg(float degree)
{
	// Rotates a given angle in degrees (positive is CCW)
	if(degree > 0){
		// Rotate CCW
		int steps = ceil(C_CCW*degree);
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_REV, steps, SPEED, 0, STEPPER_BRK_OFF,
		STEPPER_FWD, steps, SPEED, 0, STEPPER_BRK_OFF );
		}else if(degree < 0){
		//Rotate CW
		int steps = (-1)*round(C_CW*degree);
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_FWD, steps, SPEED, 0, STEPPER_BRK_OFF,
		STEPPER_REV, steps, SPEED, 0, STEPPER_BRK_OFF );
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

void goToGoal(int xpos, int ypos)
{
	// Given a desired (x,y) coordinate, rotates and drives to the location
	// if robot is not at (0,0) with roboangle ==0 , it adjusts accordingly
	int xdelta = xpos - roboxpos;
	int ydelta = ypos - roboypos;
	float dist = sqrt((pow(xdelta,2)+pow(ydelta,2)))*12;
	angle = atan2(ydelta,xdelta)*180/M_PI - roboangle;
	rotateDeg(angle);
	forward(dist);
	roboxpos = xpos;
	roboypos = ypos;
	//updateLCD();
}


void updateLCD()
{
	// Prints the robot position variables to the LCD screen
	LCD_clear(); // Clears the LCD screen

	LCD_printf("        %3.2f\n%3.2f         %3.2f\n       %3.2f\nX:%2.2f       Y:%2.2f",ft,lt,rt,bk,roboxpos,roboypos);

}
char aggressiveKid(short angle, float front, float side)
{
	// Returns a char if buffer distance is broken.
	// If buffer not broken, it returns 'w'
	// after randomly moving a short distance
	getIR();                // Reads sensors
	if (lt <= side)       // Checks left buffer
	{
		SPKR_play_beep(250, 250, 100); // Beep to indicate end
		return 'l';
	}
	else if (rt <= side)  // Checks right buffer
	{
		SPKR_play_beep(250, 250, 100); // Beep to indicate end
		return 'r';
	}
	else if (ft <= front)  // Checks front buffer
	{
		SPKR_play_beep(250, 250, 100); // Beep to indicate end
		return 'f';
	}
	else
	{
		// Calculate very small random angle increment
		if(angle==1)
		{
			int angle = 21-randRange(1,40);
			rotateDeg(angle);  // Rotates
		}
		forward(move);              // Moves forward
		return 'w';
	}
}
int randRange(int M,int N)
{
	// Returns a pseudo-random number between M and N
	return M+rand() / (RAND_MAX/(N-M+1.0)+1.0);
}
char shyGuy(short angle, float front, float right, float left, float back)
{
	getIR();                // Reads sensors
	
	if (frontRight && !frontLeft)
	{
		// Turn left
		rotateDeg(45);
		return 'f';
	}
	else if (frontLeft && !frontRight)
	{
		// Turn right
		rotateDeg(-45);
		return 'f';
	}
	else if (ft <= front || (frontRight && frontLeft))       // Checks left buffer
	{
		float leftfront;
		float rightfront;
		rotateDeg(30);
		getIR();
		leftfront = ft;
		rotateDeg(-60);
		getIR();
		rightfront = ft;
		rotateDeg(30);
		
		if(leftfront >= rightfront)
		{
			rotateDeg(90);
			forward(move);
			return 'f';
		}
		else
		{
			rotateDeg(-90);
			forward(move);
			return 'f';
		}
	}
	else if (rt <= right)
	{
		rotateDeg(30);
		forward(move);
		return 'r';
	}
	else if (lt <= left)
	{
		rotateDeg(-30);
		forward(move);
		return 'l';
	}else if (bk <= back)
	{
		forward(move);
		return 'b';
	}else
	{
		// Calculate very small random angle increment
		if(angle==1)
		{
			int angle = 21-randRange(1,40);
			rotateDeg(angle);  // Rotates
		}
		forward(move);              // Moves forward
		return 'w';
	}
	
}
//proportional control
char wallFollow(char side, float min, float max, float angle)
{
	getIR();
	if(ft<=2.5)
	{
		return 'e';
	}
	if(side=='r')
	{
		// Follow right wall
		if(rt < min)
		{
			// move away
			rotateDeg(2*angle);
			forward(3*move);
			return 'r';
		}
		else if (rt > 2*max)
		{
			// End of wall probably
			// End routine
			return 'e';
			
		}
		else if (rt > max){
			// move towards
			rotateDeg(-angle);
			forward(move);
			return 'r';
		}
		else
		{
			//move forwards
			forward(move);
			return 'r';
		}
	}else if(side=='l')
	{
		// Follow left wall
		if(lt < min){
			// too close
			rotateDeg(-.75*angle);
			forward(move);
			return 'l';
		}
		else if (lt > 2*max){
			// end routine
			return 'e';
		}
		else if (lt > max){
			// too far
			rotateDeg(angle);
			forward(3*move);
			return 'l';
		}
		else
		{
			forward(move);
			return 'l';
		}
	}
	else
	{
		// Error
		SPKR_beep(440);
		SPKR_beep(440);
		return 'x';
	}
}