
#ifndef _SCARA_ROBO_HEADERH_
#define _SCARA_ROBO_HEADERH_

#include <msp430.h>
#include <math.h>

/*////////////////////////////////////////////
ANGLES DATA
*/////////////////////////////////////////////
#define PI 3.1415
#define DEG_TO_RAD(r) ((r/180) * PI)
#define RAD_TO_DEG(r) ((r/PI) * 180)
#define POWW(r) (r*r)

/*////////////////////////////////////////////
PWM DATA and ARM SPECS
*/////////////////////////////////////////////
#define PWM_PERIOD 31333


#define LGRIP_SERVO TA0CCR1
#define RGRIP_SERVO TA0CCR2
#define BASE_SERVO TA1CCR1
#define ARM_SERVO TA2CCR1
#define TOOL_SERVO TA2CCR2


#define ARM1_MAX 5800
#define ARM1_MIN 1800
#define ARM1_MULTI (-28.89)
#define ARM2_MAX 7200
#define ARM2_MIN 2600
#define ARM2_MULTI (-22.5)
#define TOOL_MAX 7100
#define TOOL_MIN 2000
#define TOOL_MULTI (29.16)
#define LGRIP_MIN 4400
#define RGRIP_MIN 1700
#define LGRIP_MAX 6900
#define RGRIP_MAX 4600


#define SERVO_1ARM_HOME 2250
#define SERVO_2ARM_HOME 6500
#define SERVO_TROT_HOME 4950
#define SERVO_RGRIP_HOME 2500
#define SERVO_LGRIP_HOME 6000

#define LGRIP_CLOSE 4630
#define RGRIP_CLOSE 4370

#define ARM1_LENGTH 169
#define ARM2_LENGTH 149
#define TOOL_LENGTH 70
#define HEIGHT_OFF_BASE 103

/*////////////////////////////////////////////
DATA STRUCTURES NEEDED
*/////////////////////////////////////////////
#define TRUE 1
#define FALSE 0

typedef struct angles
{
	double base;
	double arm1;
	double arm2;
	double tool;
}ANGLE;

typedef struct coords
{
	double x;
	double y;
	double z;
}COORDINATE;


typedef struct data
{
	COORDINATE cords;
	ANGLE	   angle;
	int grab;
}DATA_POSITION;

volatile int stepper_current_location;
volatile char RXBuffer[6];
volatile char * PTRxBuffer;
volatile int RXCount;
volatile int rx_flag;

int TOOL_GRIP;
int TOOL_POSITION;

void gripper_position(int);
/*////////////////////////////////////////////
SCARA ROBOT CONTROL FUNCTIONS
*/////////////////////////////////////////////

int INVERSE_KINEMATIC(DATA_POSITION * data_position);


void FORWARD_KINEMATIC(DATA_POSITION * data_position);

int get_tool_position(DATA_POSITION*);

void STEPPER_MOVE_HOME(double);
void JOINT_TO_SERVO(DATA_POSITION * data_position);			//instantiating the position moves
void STEPPER_MOVE(double);
void SERVO_BASE(int position);
void SERVO_ARM(int position);
void SERVO_TOOL(int);
void GRAB_OBJECT(DATA_POSITION*);

void INIT_PWM();
void INIT_STEPPER();


/*
 * SERIAL COMMUNICATION TO THE C++ MACHINE VISION PROGRAM
 */
void uart_init();
void overclock();



#endif
