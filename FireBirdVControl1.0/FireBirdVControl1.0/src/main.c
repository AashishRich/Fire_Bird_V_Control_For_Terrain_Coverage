/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 14745600UL
#include <util/delay.h>

uint8_t data; //to store received data from UDR1
int status = 0;
uint8_t robot_id = (uint8_t )1;
uint8_t zeroval = (uint8_t)128;
uint8_t leftvelocity = (uint8_t)128, rightvelocity = (uint8_t)128;

void timer5_init()
{
	TCCR5B = 0x00; //Stop
	TCNT5H = 0xFF; //Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01; //Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00; //Output compare register high value for Left Motor
	OCR5AL = 0xFF; //Output compare register low value for Left Motor
	OCR5BH = 0x00; //Output compare register high value for Right Motor
	OCR5BL = 0xFF; //Output compare register low value for Right Motor
	OCR5CH = 0x00; //Output compare register high value for Motor C1
	OCR5CL = 0xFF; //Output compare register low value for Motor C1
	TCCR5A = 0xA9; //COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1
	// COM5C0=0
	//For Overriding normal port functionality to OCRnA outputs. WGM51=0, WGM50=1 Along With GM52 //in
	//TCCR5B for Selecting FAST PWM 8-bit Mode
	TCCR5B = 0x0B; //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18; //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

void velocity (uint8_t left_motor, uint8_t right_motor)
{
	//OCR5AL = (uint8_t)left_motor;
	//OCR5BL = (uint8_t)right_motor;
	OCR5AL = left_motor;
	OCR5BL = right_motor;
}


void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; //set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}

void motion_set (uint8_t Direction)
{
	uint8_t PortARestore = 0;

	Direction &= 0x0F; 			// removing upper nibbel as it is not needed
	PortARestore = PORTA; 			// reading the PORTA's original status
	PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
	PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
	PORTA = PortARestore; 			// setting the command to the port
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void)
{
	motion_set(0x00);
}


//SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
SIGNAL(USART0_RX_vect)
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable

	//UDR0 = data; 				//echo data back to PC
	
	if(status == 0 && data == 2){
		status = 1;
	}
	else if(status == 1){
		leftvelocity = data;
		status = 2;
	}
	else if(status == 2){
		rightvelocity = data;
		status = 0;
	}
	if(status == 0){
		if(leftvelocity == (uint8_t)128 && rightvelocity == (uint8_t)128)
		{
			stop();
			
		}
		
		else
		{
			
			if(leftvelocity >= (uint8_t)128 && rightvelocity >= (uint8_t)128)
			{
				forward();
				leftvelocity = 2*(leftvelocity - zeroval);
				rightvelocity = 2*(rightvelocity - zeroval);
				velocity(leftvelocity, rightvelocity);
				
			}
			
			else if(leftvelocity < (uint8_t)128 && rightvelocity < (uint8_t)128)
			{
				back();
				leftvelocity = 2*(zeroval-leftvelocity);
				rightvelocity = 2*(zeroval-rightvelocity);
				velocity(leftvelocity, rightvelocity);
				
			}
			
			else if(leftvelocity < (uint8_t)128 && rightvelocity >= (uint8_t)128)
			{
				left();
				leftvelocity = 2*(zeroval-leftvelocity);
				rightvelocity = 2*(rightvelocity - zeroval);
				velocity(leftvelocity, rightvelocity);
				
			}
			else if(leftvelocity >= (uint8_t)128 && rightvelocity < (uint8_t)128)
			{
				right();
				leftvelocity = 2*(leftvelocity - zeroval);
				rightvelocity = 2*(zeroval-rightvelocity);
				velocity(leftvelocity, rightvelocity);
				
			}
			
			else
			{
				stop();
			}
			
		}
	}
	
	
	
	
	
	/*else
	{	
		forward();
		velocity(leftvelocity, rightvelocity);
	}*/
	

	/*if(data == 0x38) //ASCII value of 8
	{
		PORTA=0x06;  //forward
	}

	if(data == 0x32) //ASCII value of 2
	{
		PORTA=0x09; //back
	}

	if(data == 0x34) //ASCII value of 4
	{
		PORTA=0x05;  //left
	}

	if(data == 0x36) //ASCII value of 6
	{
		PORTA=0x0A; //right
	}

	if(data == 0x35) //ASCII value of 5
	{
		PORTA=0x00; //stop
	}*/

	/*if(data == 0x37) //ASCII value of 7
	{
		buzzer_on();
	}

	if(data == 0x39) //ASCII value of 9
	{
		buzzer_off();
	

	}*/

}



void init_devices (void) //use this function to initialize all devices
{
	cli(); //disable all interrupts
	motion_pin_config();
	uart0_init(); //Initailize UART1 for serial communiaction
	timer5_init();
	sei(); //re-enable interrupts
}











int main (void)
{
	board_init();
	
	init_devices();

	//stop();
	forward();
	velocity(0, 0);
	//velocity(leftvelocity, rightvelocity);
	while(1);
	
	return 0;
}
