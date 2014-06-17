/********************************************************************************
 Platform: SPARK V
 Experiment: 10B_Serial_Comminucation_ZigBee_wireless
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 Last Modification: 22nd September 2010
 AVR Studio Version 4.17, Build 666

 Concepts covered: serial communication using zigbee wireless module

 This experiment demonstrates Robot control over serial port via ZigBee wireless 
 comunication module 

 There are two components to the motion control:
 1. Direction control using pins PORTB0 to 	PORTB3
 2. Velocity control by PWM on pins PD4 and PD5.

 In this experiment for the simplicity PD4 and PD5 are kept at logic 1.
 
 Pins for PWM are kept at logic 1.
  
 Connection Details:   L-1---->PB0;		L-2---->PB1;
   					   R-1---->PB2;		R-2---->PB3;
   					   PD4 (OC1B) ----> Logic 1; 	PD5 (OC1A) ----> Logic 1; 

 Serial Communication: PORTD 0 --> RXD UART receive for RS232 serial communication
					   PORTD 1 --> TXD UART transmit for RS232 serial communication

 Make sure that J5 us set towards the back side of the robot 
 i.e. XBee wireless module is connected to the serial port of the microcontroller.
 For more details on the jumper settings refer to the hardware manual.

 Use baud rate as 9600bps.
 
 To control robot use number pad of the keyboard which is located on the right hand
 side of the keyboard. Make sure that NUM lock is on.

 For more detail on hardware connection and theory refer the hardware manual.

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega16
 	Frequency: 7372800
 	Optimization: -O0 (For more information read section: Selecting proper optimization
	              options below figure 4.22 in the hardware manual)
 *********************************************************************************/

 /********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode


********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lch.h"
#include <stdio.h>

unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning
unsigned int linearDistance;
unsigned int flag;
unsigned int flag1 = 0;
unsigned char receive_data=0;   // used to save Receiving data

void motion_pin_config (void)
{
 DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
 PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic 0
 DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
 PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}

void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//Function to configure INT1 (PORTD 3) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRD  = DDRD & 0xF7;  //Set the direction of the PORTD 3 pin as input
 PORTD = PORTD | 0x08; //Enable internal pull-up for PORTD 3 pin
}

//Function to configure INT0 (PORTD 2) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRD  = DDRD & 0xFB;  //Set the direction of the PORTD 2 pin as input
 PORTD = PORTD | 0x04; //Enable internal pull-up for PORTD 2 pin
}



//Function to initialize ports
void port_init()
{
 motion_pin_config();
 buzzer_pin_config();
 left_encoder_pin_config();    //left encoder pin config
 right_encoder_pin_config();   //right encoder pin config
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

void left_position_encoder_interrupt_init (void) //Interrupt 1 enable
{
 cli(); //Clears the global interrupt
 MCUCR = MCUCR | 0x08; // INT1 is set to trigger with falling edge
 GICR = GICR | 0x80;   // Enable Interrupt INT1 for left position encoder
 sei(); // Enables the global interrupt 
}

void right_position_encoder_interrupt_init (void) //Interrupt 0 enable
{
 cli(); //Clears the global interrupt
 MCUCR = MCUCR | 0x02; // INT0 is set to trigger with falling edge
 GICR = GICR | 0x40;   // Enable Interrupt INT5 for right position encoder
 sei(); // Enables the global interrupt 
}

//ISR for right position encoder
ISR(INT0_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
}

//ISR for left position encoder
ISR(INT1_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}


//UART0 initialisation
// desired baud rate: 9600
// actual: baud rate:9600 (0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSRB = 0x00; //disable while setting baud rate
 UCSRA = 0x00;
 UCSRC = 0x86;
 UBRRL = 0x2F; //set baud rate lo  //67 is for 16MHz 9600 baudrate
 UBRRH = 0x00; //set baud rate hi
 UCSRB = 0x98; 
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortBRestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortBRestore = PORTB; 			// reading the PORTB's original status
 PortBRestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortBRestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTB status
 PORTB = PortBRestore; 			// setting the command to the port
}

void forward (void)         //both wheels forward
{
  motion_set(0x06);
}

void backward (void)        //both wheels backward
{
  motion_set(0x09);
}

void left (void)            //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void)           //Left wheel forward, Right wheel backward
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

void stop (void)            //hard stop
{
  motion_set(0x00);
}

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}


//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 12.85; // division by resolution to get shaft count 
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 12.92; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;
 while(1)
 {
  if(ShaftCountRight > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
 backward();
 linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees) 
{
// 28 pulses for 360 degrees rotation 12.92 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
// 28 pulses for 360 degrees rotation 12.92 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
 // 56 pulses for 360 degrees rotation 12.85 degrees per count
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
 // 56 pulses for 360 degrees rotation 12.85 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
 // 56 pulses for 360 degrees rotation 12.85 degrees per count
 soft_left_2(); //Turn reverse soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
 // 56 pulses for 360 degrees rotation 12.85 degrees per count
 soft_right_2();  //Turn reverse soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void init_devices (void)
{
 cli();         //Clears the global interrupts
 port_init();
 uart0_init();
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 sei();         //Enables the global interrupts
}

SIGNAL(SIG_UART_RECV) 
{
 

 receive_data = UDR;			
 
 
 unsigned int data1 = receive_data - '\0';
 //UDR = data1;           // Echo the received data plus 1
 if(flag >= 0 & flag <= 0) {
	//left_degrees(90);
	Degrees = 3*data1;
	//left_degrees(360-Degrees);
	flag = 1;
 }
 else {
	//forward_mm(100);
	linearDistance = 2*data1;
	flag = 0;
	flag1=1;
	/*
	if(Degrees > 180) {
		left_degrees(360 - Degrees);
		stop();
		_delay_ms(500);
	}
	else{
		
		stop();
		_delay_ms(500);
	}
	*/
//	forward_mm(linearDistance*10);
//	stop();
//	_delay_ms(500);	
 }

 /*

 if(receive_data == 0x38)        //ASCII value of 8
 {
	forward();                   //forward
 }

 if(receive_data == 0x32)        //ASCII value of 2
 {
	backward();                  //back
 }

 if(receive_data == 0x34)        //ASCII value of 4
 {
	left();                      //left
 }

 if(receive_data == 0x36)        //ASCII value of 6
 {
	right();                     //right
 }

 if(receive_data == 0x35)        //ASCII value of 5
 {
	stop();                      //hard stop
 }

 if(receive_data == 0x37)        //ASCII value of 7
 {
	buzzer_on();
 }

 if(receive_data == 0x39)        //ASCII value of 9
 {
	buzzer_off();
 }
 */
}

//Main Function
int main()
{
	init_devices();
	lcd_set_4bit();
	
	char str[5];
	char str1[5];
	//left_degrees(75);
	stop();
	while(1){
		if(flag1 >= 1 & flag1 <= 1) {
			lcd_init();
			flag1 = 0;
			if(Degrees >= 270 & Degrees <= 270 & linearDistance >= 180 & linearDistance <= 180){
				forward();
			}
			else if(Degrees >= 267 & Degrees <= 267 & linearDistance >= 178 & linearDistance <= 178){
				stop();
			}
			else if(Degrees >= 264 & Degrees <= 264 & linearDistance >= 176 & linearDistance <= 176){
				left_degrees(45);
			}
			else if(Degrees >= 261 & Degrees <= 261 & linearDistance >= 174 & linearDistance <= 174){
				right_degrees(45);
			}
			else {
				if(Degrees > 180) {
					lcd_cursor(1,1);
					sprintf(str,"%d",360-Degrees);
  					lcd_string(str);
					lcd_cursor(1,4);
					lcd_string(" Deg left");
					left_degrees(360-Degrees);
				}
				else {
					lcd_cursor(1,1);
					sprintf(str,"%d",Degrees);
  					lcd_string(str);
					lcd_cursor(1,4);
					lcd_string(" Deg right");
					right_degrees(Degrees);
				}
				sprintf(str1,"%d",linearDistance);
				lcd_cursor(2,1);
				lcd_string(str1);
				lcd_cursor(2,4);
				lcd_string(" cm fwd");
				forward_mm(linearDistance*10);
			}
			UDR = receive_data;
		}

	}
}
