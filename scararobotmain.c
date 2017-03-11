#include "scara_robo_header.h"
/*
 * main.c
 */

int GET_POSITION_TEMP(DATA_POSITION * data_position);



int main(void)

{
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
    overclock();
    uart_init();
	INIT_PWM();
	INIT_STEPPER();
	DATA_POSITION data_position[2];
	int move_flag = 0;
	while(1)
	{
		UCA1IFG = 0x00;
		UCA1RXBUF = 0x00;
		UCA1IE |= UCRXIE + UCTXIE;
		__bis_SR_register(GIE);
		move_flag = GET_POSITION_TEMP(data_position);
		int k = INVERSE_KINEMATIC(data_position);
		if(k == -1 || move_flag == -1 || move_flag == 2)
		{
			if(k == -1)
			{
			UCA1TXBUF  = -1;
			}
		}
		else
		{
		JOINT_TO_SERVO(data_position);
		//GRAB_OBJECT(data_position);
		}
	}
    return 0;
}



/*
 * INVERSE KINEMATICS TO GET FROM GIVEN POSITION TO JOINT ANGLES
 */
void GRAB_OBJECT(DATA_POSITION * data_position)
{
	INVERSE_KINEMATIC(data_position);
	JOINT_TO_SERVO(data_position);
	gripper_position(TOOL_GRIP);
	TOOL_GRIP = 1 - TOOL_GRIP;
}


int get_tool_position(DATA_POSITION* data_position)
{
	double rp = sqrt(data_position[0].cords.y*data_position[0].cords.y + data_position[0].cords.x*data_position[0].cords.x);
	int length = (int)rp;
	int val1 = 2;
	if(length < 175)val1 = 1;
	return val1;

}

int INVERSE_KINEMATIC(DATA_POSITION *data_position)
{
	//find RP and THETA1 first
	double height = (data_position[0].cords.z-HEIGHT_OFF_BASE);
	if(height < -100.00)
	{
		return -1;
	}
	double rp;
	if(data_position[1].grab == 1)
	{
		switch (TOOL_POSITION)
			{
			case 2:
				rp = sqrt(data_position[0].cords.y*data_position[0].cords.y + data_position[0].cords.x*data_position[0].cords.x) - 25;
				break;
			case 1:
				rp = sqrt(data_position[0].cords.y*data_position[0].cords.y + data_position[0].cords.x*data_position[0].cords.x);
				height += 25;
			}
	}
	else
	{
		TOOL_POSITION = get_tool_position(data_position);
		switch (TOOL_POSITION)
		{
		case 0:
			break;
		case 2:
			rp = sqrt(data_position[0].cords.y*data_position[0].cords.y + data_position[0].cords.x*data_position[0].cords.x)-(TOOL_LENGTH);
			break;
		case 1:
				rp = sqrt(data_position[0].cords.y*data_position[0].cords.y + data_position[0].cords.x*data_position[0].cords.x);
				height += TOOL_LENGTH;
				break;
		case -1:
			return -1;
		default:
			break;


		}

	}
	data_position[1].angle.base = atan2(data_position[0].cords.y,data_position[0].cords.x);
	//find angle beta
	double beta = atan2(height,rp);
	data_position[1].angle.tool = RAD_TO_DEG(beta);
	//find angle alpha
	double c_hyp = rp/cos(beta);
	double filter = (POWW(ARM2_LENGTH) - POWW(c_hyp) - POWW(ARM1_LENGTH)) /  (- 2 * c_hyp * ARM1_LENGTH);
	double alpha = acos(filter);

	data_position[1].angle.arm1 = beta + alpha;
	if(data_position[1].angle.arm1 >= 2.11)
	{
		data_position[1].angle.arm1 = beta - alpha;
	}
	if(0)//condition for left or right arm implementation
	{

	}
	double val1 = (height - (ARM1_LENGTH * sin(data_position[1].angle.arm1)));
	double val2 = (rp - (ARM1_LENGTH * cos(data_position[1].angle.arm1)));
	data_position[1].angle.arm2 = atan2(val1,val2);   // THIS IS THE PROBLEM AREA NEED TO SOLVE FOR angle off base of arm
	data_position[1].angle.tool = RAD_TO_DEG(-data_position[1].angle.arm2);
	data_position[1].angle.arm2 = data_position[1].angle.arm2 - data_position[1].angle.arm1;
	data_position[1].angle.arm2 = RAD_TO_DEG(data_position[1].angle.arm2);
	data_position[1].angle.arm1 = RAD_TO_DEG(data_position[1].angle.arm1);
	data_position[1].angle.base =RAD_TO_DEG(data_position[1].angle.base);

	if(data_position[1].angle.arm2 > 30)
	{
		data_position[1].angle.arm2 = 25;
	}
	if(TOOL_POSITION == 1)
	{

		data_position[1].angle.tool = (-90-(data_position[1].angle.arm2 + data_position[1].angle.arm1));

	}
	if(TOOL_POSITION == 2)
	{
		data_position[1].angle.tool = (-(data_position[1].angle.arm2 + data_position[1].angle.arm1 ));
	}
	return 1;
}

/*
 * FORWARD KINEMATICS DON"T REALLY NEED TO IMPLEMENT THEM YET
 */
void FORWARD_KINEMATIC(DATA_POSITION * data_position)
{







}

void JOINT_TO_SERVO(DATA_POSITION * data_position)
{
	STEPPER_MOVE(data_position[1].angle.base);
	SERVO_TOOL((int)data_position[1].angle.tool);
	SERVO_ARM((int)data_position[1].angle.arm2);
	SERVO_BASE((int)data_position[1].angle.arm1);

	gripper_position(0);
}

void SERVO_BASE(int position)
{
	BASE_SERVO = (int)(position * ARM1_MULTI + 4950);
}

void gripper_position(int position)
{
	switch (position)
	{
	case 0:
		TA0CCR3 = SERVO_RGRIP_HOME;
		TA0CCR4 = SERVO_LGRIP_HOME;
		break;
	case 1:
		TA0CCR3 = RGRIP_CLOSE;
		TA0CCR4 = LGRIP_CLOSE;
		break;
	default:
		TA0CCR3 = SERVO_RGRIP_HOME;
		TA0CCR4 = SERVO_LGRIP_HOME;
		break;
	}
}

void SERVO_TOOL(int position)
{
	TOOL_SERVO = (int)((position * TOOL_MULTI) +4950);

}

void SERVO_ARM(int position)
{
	ARM_SERVO = (int)((position * ARM2_MULTI)+2925);
}

int GET_POSITION_TEMP(DATA_POSITION * data_position)
{
	while(rx_flag);
	__bic_SR_register(GIE);
	int x = (RXBuffer[0]<<8) | RXBuffer[1];
	int y = (RXBuffer[2]<<8) | RXBuffer[3];
	int z = (RXBuffer[4]<<8) | RXBuffer[5];
	data_position[0].cords.x = x;
	data_position[0].cords.y = y;
	data_position[0].cords.z = z;

	if(x > 500.00)
	{
		x -=500;
		rx_flag = 1;
		switch (x)
		{
		case 0:
			stepper_current_location = 0;
			return 2;
		case 1:
			gripper_position(0);
			return 2;
		case 2:
			gripper_position(1);
			return 2;
		case 3:
			stepper_current_location = 0;
			return 2;
		case 4:
			STEPPER_MOVE_HOME(data_position[0].cords.z);
			return 2;
		default:
			return 2;
		}
	}
	rx_flag = 1;
	UCA1IE = 0x00;                         // Enable USCI_A1 RX interrupt
}

void STEPPER_MOVE_HOME(double final)
{
	int i = 0;
	int move = (int)final;
	move = move/1.8;
	if(move < 0)
	{
		P8OUT |= BIT2;
	}
	if(move > 0)
	{
		P8OUT &= ~BIT2;

	}
	move = abs(move) *2;
	int toggle = 0;
	for(i = 0;i<move;i++)
	{
		if(toggle == 0)
		{
		P3OUT |= BIT7;
		}
		else
		{
		P3OUT &= ~BIT7;
		}
		_delay_cycles(350000);
		toggle = 1-toggle;
	}
}

void STEPPER_MOVE(double final)
{
	int i = 0;
	int move = (int)final;
	int temp = move;
	move = move - stepper_current_location;
	stepper_current_location = temp;
	move = move/1.8;
	if(move < 0)
	{
		P8OUT |= BIT2;
	}
	if(move > 0)
	{
		P8OUT &= ~BIT2;

	}
	move = abs(move) *2;
	int toggle = 0;
	for(i = 0;i<move;i++)
	{
		if(toggle == 0)
		{
		P3OUT |= BIT7;
		}
		else
		{
		P3OUT &= ~BIT7;
		}
		_delay_cycles(350000);
		toggle = 1-toggle;
	}
}
/*
 *
 * INIT PWM FOR THE SCARA ROBOT ARM JOINTS
 */
void INIT_PWM()
{

		 P2DIR |= BIT0 + BIT4 + BIT5;              // P2.0 2.4 2.5
		 P2SEL |= BIT0 + BIT4 + BIT5;              // P2.0 2.4 2.5
		 P1DIR |= BIT4+BIT5;                       // P1.2 and P1.3 output
		 P1SEL |= BIT4+BIT5;                       // P1.2 and P1.3 options select
		 TA1CCR0 = PWM_PERIOD;                     // PWM Period/2
		 TA1CCTL1 = OUTMOD_6;                      //CCR1 toggle/set
		 TA1CCR1 = SERVO_1ARM_HOME;                // CCR1 PWM duty cycle
		         // SMCLK, continuos mode

		TA0CCR0 = PWM_PERIOD;                     // PWM Period
		TA0CCTL3 = OUTMOD_6;                      // CCR1 reset/set
		TA0CCR3 = SERVO_RGRIP_HOME;               // CCR1 PWM duty cycle PORT 1.4
		TA0CCTL4 = OUTMOD_6;                      // CCR2 reset/set
		TA0CCR4 = SERVO_LGRIP_HOME;               // CCR2 PWM duty cycle PORT 1.5


		 TA2CCR0 = PWM_PERIOD;                     // PWM Period/2
		 TA2CCTL1 = OUTMOD_6;                      // CCR1 toggle/set
		 TA2CCR1 = SERVO_2ARM_HOME;                // CCR1 PWM duty cycle p 2.5
		 TA2CCTL2 = OUTMOD_6;                      // CCR2 otggle/set
		 TA2CCR2 = SERVO_TROT_HOME;                // CCR2 PWM duty cycle p 2.4
		 TA2CTL = TASSEL_2 + MC_1 + TACLR +ID_3;         // SMCLK, up-down mode, clear TAR
		 TA1CTL = TASSEL_2 + MC_1 + TACLR +ID_3;
		 TA0CTL = TASSEL_2 + MC_1 + TACLR +ID_3;         // ACLK, up mode, clear TAR
}

void INIT_STEPPER()
{
	P8DIR = BIT2;
	P3DIR = BIT7;
	P8OUT = BIT2;
	stepper_current_location = 0;
}




