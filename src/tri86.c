/*
 * Tritium TRI86 EV Driver Controls, version 3 hardware
 * Copyright (c) 2010, Tritium Pty Ltd.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
 *	  in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of Tritium Pty Ltd nor the names of its contributors may be used to endorse or promote products 
 *	  derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE. 
 *
 */

// Include files
#include <msp430x24x.h>
#include <signal.h>
#include <stdlib.h>
#include "tri86.h"
#include "can.h"
#include "usci.h"
#include "pedal.h"
#include "gauge.h"



// Function prototypes
void clock_init( void );
void io_init( void );
void timerA_init( void );
void timerB_init( void );
void adc_init( void );
static void __inline__ brief_pause(register unsigned int n);
void update_switches( unsigned int *state1, unsigned int *state2);
void update_encoder(unsigned int *throttle, unsigned int throttle_sel);
// call regularly to update *throttle with encoder pulses in previous period
// throttle_sel is non zero for cruise style throttle and zero for racing throttle
// if throttle_sel changes between calls then *throttle gets cleared

// Global variables
// Status and event flags
volatile unsigned int events = 0x0000;


// Data from motor controller
float motor_rpm = 0;
float motor_temp = 0;
float controller_temp = 0;
float battery_voltage = 0;
float battery_current = 0;

#define ENCODER_MAX ADC_MAX

/*
 *   ENCODER Inputs
 *  Pass in current throttle setting
 * Throttle updated via the direction and speed of change of the encoder
 */
  
  
static int encoder_states[4][4] = {{0,-1,1,10},{1,0,10,-1},{-1,10,0,1},{10,1,-1,0}}; 
static int av_counter = 1; 
static int counter = 1;
static int missed = 0;
static int delta = 0;   // integrated encoder counts between timer ticks  
static int last_inc = 0;

void read_encoder(void)
{
   static unsigned char last = 0;
   unsigned char cur;

    cur = 0;
    if(P2IN & IN_ENC_A) cur = 1;
	if (P2IN &  IN_ENC_B) cur +=2;
	
	
	if (last != cur) {
	    if (counter < 70) counter += 7;
	    if (encoder_states[cur][last] > 1) {  // error missed code
	       missed++;
//		   delta = delta  + 2 * last_inc;  // assume running in the same direction 
	       }
		else {   
		    last_inc = encoder_states[cur][last];
	        delta = delta  + last_inc;
			}
	   } 
    last = cur;
	
}

void update_encoder(unsigned int *throttle, unsigned int throttle_sel)
{
    int temp;
    static unsigned int last_throttle_sel = 0;
	
    if (counter > 2) counter--;   // if no change since last check then reduce the 

    if (throttle_sel != last_throttle_sel){
        *throttle = 0;
        last_throttle_sel = throttle_sel;
        }
    
    temp = *throttle;
    if (throttle_sel){
        temp = temp + (delta << 3);
       }
    else{
       temp = temp  + av_counter * delta;
	   }
    delta = 0;
    if (temp < 0) temp = 0;
    if (temp > ENCODER_MAX) temp = ENCODER_MAX;
    *throttle = temp;   
	av_counter = (av_counter  + counter)/2;   
}  

static long cur_speed = 0L;   // speed values stored at 0.10 * rpm
static long remote_target = 0L;  // target speed set by chase car
static long target    = 0L;   // target speed
static long esum      = 0L;        // sum of previous errors
static long prev_speed = 0L;  // last speed value
#define ERR_HYS 2

unsigned int cruising = FALSE;

static long PID_setpoint = 0L;

// PID constants * 1000
#define Kp_default 1000L    
#define Ki_default 100L
#define Kd_default 800L
#define CV_MAX ADC_MAX
#define ESUM_MAX ( CV_MAX * 1000L / Ki)   // maximum that esum can be to avoid saturating PID_setpoint
#define SCALE(K, v) (((K)*(v) + 500L) / 1000L)  // multiply v by one of the PID constants and round to nearest unit

static long Kp = Kp_default;
static long Ki = Ki_default;
static long Kd = Kd_default;


void update_PID(void)
{
    long cv;
    long err;
    long delta_speed;
    
    cur_speed = (long)(motor_rpm);
 //   cur_speed = (cur_speed + 5L) / 10L;
    
    
    err = target - cur_speed;
    if (abs(err) < ERR_HYS) {
        err = 0L;
        }
    esum = esum + err;
    if (esum > ESUM_MAX) esum = ESUM_MAX;
    if (esum < 0L) esum = 0L;
    
    delta_speed = prev_speed - cur_speed;
    prev_speed = cur_speed;
    
    cv = SCALE(Kp,err) + SCALE(Ki, esum) + SCALE(Kd, delta_speed);
    
    if (cv < 0)  cv = 0L;
    if (cv > CV_MAX) cv = CV_MAX;
 
    PID_setpoint = cv; 

}

void set_cruise_control(long throttle)
{
    target = cur_speed;
    PID_setpoint = throttle;
    esum = PID_setpoint * 1000L / Ki;   // set the integrated error to balance point
    cruising = TRUE;
}


void enable_cruise_target()
{
    target = remote_target;
	cruising = TRUE;
}

void clr_cruise_control(void)
{
    cruising = FALSE;
	target = 0L;
	esum = 0L;
	PID_setpoint = 0L;
}


group_32 vin_av;
static int vin_av_cnt = 0;
static unsigned int vin;
void update_vin(void)
// uses upper 16 bits of 32 bit unsigned number to simulate dived by 65536
// sums 16 readings to scale 12 bit ADC into 16bit figure
{
    vin_av.data_u32 += (unsigned long)(ADC12MEM3);
	
	if (++vin_av_cnt >= 16){
	   vin_av.data_u32 *= 14250L;
	   vin = vin_av.data_u16[1];
	   vin_av_cnt = 0;
	   vin_av.data_u32 = 0L;
	   }
}

//tmpl = ADC12MEM3;
//tmpl *= 14250L;
//vin = (unsigned int) (tmpl >> 12);
		
  
void init_vin(void)
{
    vin_av.data_u32 = 0L;
	vin = 12000;
}  

// Main routine
int main( void )
{ 
	// Local variables
	// Switch inputs - same bitfield positions as CAN packet spec
	unsigned int switches1 = 0x0000;
	unsigned int switches2 = 0x0000;
	
    unsigned int encoder_throttle = 0;
	unsigned int floor_throttle;
	unsigned char next_state = MODE_OFF;
	unsigned char current_egear = EG_STATE_NEUTRAL;

	
	// Comms
	unsigned int comms_event_count = 0;
	// LED flashing
	unsigned char flash_count = FLASH_SPEED;
	// Debug
	unsigned int i;
	
	// Stop watchdog timer
	WDTCTL = WDTPW + WDTHOLD;

	// Initialise I/O ports
	io_init();

	// Wait a bit for clocks etc to stabilise, and power to come up for external devices
	// MSP430 starts at 1.8V, CAN controller need 3.3V
	for(i = 0; i < 10000; i++) brief_pause(10);

	init_vin();
	// Initialise clock module - internal osciallator
	clock_init();

	// Initialise SPI port for CAN controller (running with SMCLK)
	usci_init(0);
	
	// Reset CAN controller and initialise
	// This also changes the clock output from the MCP2515, but we're not using it in this software
	can_init( CAN_BITRATE_500 );
	events |= EVENT_CONNECTED;

	// Initialise Timer A (10ms timing ticks)
	timerA_init();

	// Initialise Timer B (gauge outputs PWM / pulses)
	timerB_init();
  
	// Initialise A/D converter for potentiometer and current sense inputs
	adc_init();

	// Initialise switch & encoder positions
	update_switches(&switches1, &switches2);
	
	// Initialise command state
	command.rpm = 0.0;
	command.current = 0.0;
	command.bus_current = 1.0;
	command.flags = 0x00;
	command.state = MODE_OFF;
	
	// Init gauges
	gauge_init();

	// Enable interrupts
	eint();

	// Check switch inputs and generate command packets to motor controller
	while(TRUE){
		// Process CAN transmit queue
		can_transmit();
        
		read_encoder();
		
		// Monitor switch positions & analog inputs
		if( events & EVENT_TIMER ){
			events &= ~EVENT_TIMER;			
			ADC12CTL0 |= ADC12SC;               	// Start A/D conversions
		}

		if( events & EVENT_ADC ){
			events &= ~EVENT_ADC;
			// Check for 12V supply 
			update_vin();

			// Update motor commands based on pedal and slider positions
            update_encoder(&encoder_throttle, switches2 & SW2_THROTTLE_SEL);
            if (cruising) update_PID();			
#ifdef REGEN_ON_BRAKE
			process_pedal( ADC12MEM0, ADC12MEM1, ADC12MEM2, (switches1 & SW_BRAKE) );	// Request regen on brake switch
#else

#ifdef USE_LINEAR_THROTTLE
            if (switches1 & SW1_BRAKE)
                 floor_throttle = 0; 			
			else 
			     floor_throttle = ADC12MEM0;
				 
			if (switches2 & SW2_THROTTLE_SEL){			
		   	     process_pedal( floor_throttle, ADC12MEM2, switches2 & SW2_REGEN_PB, FALSE);	
				 encoder_throttle = 0;
				 }
			else 
                 process_pedal(encoder_throttle, ADC12MEM2, switches2 & SW2_REGEN_PB, TRUE);
#else
                 if (cruising)
                   process_pedal(PID_setpoint, ADC12MEM2, switches2 & SW2_REGEN_PB, TRUE);
                 else
                   process_pedal(encoder_throttle, ADC12MEM2, switches2 & SW2_REGEN_PB, TRUE);

#endif
                 
#endif
			
			// Update current state of the switch inputs
			update_switches(&switches1, &switches2);
			
			if (--flash_count == 0) flash_count = (FLASH_SPEED * 2);
			if (flash_count > FLASH_SPEED) switches2 |= SW2_FLASH;
			else switches2 &= ~SW2_FLASH;
				
				
			// Track current operating state
			switch(command.state){
				case MODE_OFF:
	               // Flash N LED in idle mode
							
   				    P5OUT &= ~(LED_GEAR_ALL);	
					if (switches2 & SW2_FLASH) P5OUT |= LED_GEAR_2;
					
					if(switches1 & SW1_MODE_N) next_state = MODE_N;
					else next_state = MODE_OFF;
					
					switches1 |= SW1_IGN_ON;
					break;

				case MODE_N:

					if((switches1 & SW1_MODE_R) && ((events & EVENT_SLOW) || (events & EVENT_REVERSE))) next_state = MODE_R;
					else if((switches1 & SW1_MODE_D) && ((events & EVENT_SLOW) || (events & EVENT_FORWARD))) next_state = MODE_DL;
					else next_state = MODE_N;
					P5OUT &= ~(LED_GEAR_ALL);
					P5OUT |= LED_GEAR_2;
					switches1 |= SW1_IGN_ON;
					encoder_throttle = 0;
                    clr_cruise_control();
					break;

#ifdef USE_CO                    
				case MODE_CO_R:
				case MODE_CO_DL:
				case MODE_CO_DH:
					if(switches1 & SW1_MODE_N) next_state = MODE_N;
					else if((command.state == MODE_CO_R) && (current_egear == EG_STATE_LOW)) next_state = MODE_R;
					else if((command.state == MODE_CO_DL) && (current_egear == EG_STATE_LOW)) next_state = MODE_DL;
					else if((command.state == MODE_CO_DH) && (current_egear == EG_STATE_HIGH)) next_state = MODE_DH;
					else next_state = command.state;
					break;
#endif
    			case MODE_R:
					if(switches1 & SW1_MODE_N) next_state = MODE_N;
					else if((switches1 & SW1_MODE_D) && ((events & EVENT_SLOW) || (events & EVENT_FORWARD))) next_state = MODE_DL;	// Assume already in low egear
					else next_state = MODE_R;
					
					P5OUT &= ~(LED_GEAR_ALL);
					P5OUT |= LED_GEAR_3;
		
					clr_cruise_control();
					break;

				case MODE_DL:
					if(switches1 & SW1_MODE_N) next_state = MODE_N;
					else if((switches1 & SW1_MODE_R) && ((events & EVENT_SLOW) || (events & EVENT_REVERSE))) next_state = MODE_R;		// Assume already in low egear
					else next_state = MODE_DL;
					
       //             if (switches1 & SW1_MODE_D){
         //               clr_cruise_control();
           //             }
                    if ((switches2 & SW2_CRUISE) && (!cruising) && (!(events & EVENT_SLOW))){
                        set_cruise_control(encoder_throttle);
                        }
					if ((switches2 & SW2_CRUISE_REMOTE) && (!(events & EVENT_SLOW))){
                        enable_cruise_target();
                        }	
                    
					P5OUT &= ~(LED_GEAR_ALL);
					if (cruising){
                        if (switches2 & SW2_FLASH) P5OUT |= LED_GEAR_1;
                        }
                    else{    
   				        P5OUT |= LED_GEAR_1;
                        }
                    
                          
		
					break;

#ifdef USE_CO                    
				case MODE_DH:
					if(switches1 & SW1_MODE_N) next_state = MODE_N;
					else next_state = MODE_DH;
					
					P5OUT &= ~(LED_GEAR_ALL);
					P5OUT |= LED_GEAR_1;
		
					break;
#endif
				default:
					next_state = MODE_OFF;
					break;
			}
			command.state = next_state;
		
		
			// Control brake lights
			if((switches1 & SW1_BRAKE) || (switches2 & SW2_REGEN_PB) /*|| (events & EVENT_REGEN)*/){
    			P1OUT |= BRAKE_OUT;
				encoder_throttle = 0;
                clr_cruise_control();
				}
			else P1OUT &= ~BRAKE_OUT;
			
			// Hazard Lights
			if ((switches2 & SW2_IND_ON) && (switches2 & SW2_FLASH))
     			P5OUT |= LED_HAZ;
			else 
			    P5OUT &= ~LED_HAZ;
	      	
				
			// Control reversing lights
			if(command.state == MODE_R) P1OUT |= REVERSE_OUT;
			else P1OUT &= ~REVERSE_OUT;
			
			// Control CAN bus and pedal sense power
//			if((switches & SW_IGN_ACC) || (switches & SW_IGN_ON)){
				P1OUT |= CAN_PWR_OUT;
				P6OUT |= ANLG_V_ENABLE;
//			}
//			else{
//				P1OUT &= ~CAN_PWR_OUT;
//				P6OUT &= ~ANLG_V_ENABLE;
//				events &= ~EVENT_CONNECTED;
//			}

			
			// Control front panel fault indicator
			if(switches1 & (SW1_CAN_FAULT | SW1_BRAKE_FAULT)) P3OUT &= ~LED_REDn;
			else P3OUT |= LED_REDn;
			
		}

		// Handle outgoing communications events
		if(events & EVENT_COMMS){
			events &= ~EVENT_COMMS;
			// Blink CAN activity LED
			events |= EVENT_CAN_ACTIVITY;			

			// Update command state and override pedal commands if necessary
				switch(command.state){
					case MODE_R:
					case MODE_DL:
					case MODE_DH:
#ifndef REGEN_ON_BRAKE
#ifdef CUTOUT_ON_BRAKE
						if(switches & SW_BRAKE){
							command.current = 0.0;	
							command.rpm = 0.0;
						}
#endif
#endif
						break;
					case MODE_N:
					case MODE_OFF:
					case MODE_CO_R:
					case MODE_CO_DL:
					case MODE_CO_DH:
					default:
						command.current = 0.0;
						command.rpm = 0.0;
						break;
				}
			
			// Transmit commands and telemetry
			if(events & EVENT_CONNECTED){
				// Transmit drive command frame
				can_push_ptr->address = DC_CAN_BASE + DC_DRIVE;
				can_push_ptr->status = 8;
				can_push_ptr->data.data_fp[1] = command.current;
				can_push_ptr->data.data_fp[0] = command.rpm;
				can_push();		
	
				// Transmit bus command frame
				can_push_ptr->address = DC_CAN_BASE + DC_POWER;
				can_push_ptr->status = 8;
				can_push_ptr->data.data_fp[1] = command.bus_current;
				can_push_ptr->data.data_fp[0] = 0.0;
				can_push();
				
				// Transmit switch position/activity frame and clear switch differences variables
				can_push_ptr->address = DC_CAN_BASE + DC_SWITCH;
				can_push_ptr->status = 8;
				can_push_ptr->data.data_u8[7] = command.state;
				can_push_ptr->data.data_u8[6] = command.flags;
				can_push_ptr->data.data_u16[2] = vin;   // send voltage rail
				can_push_ptr->data.data_u16[1] = switches2; 
				can_push_ptr->data.data_u16[0] = switches1;
				can_push();

				// Transmit cruise control settings 
				can_push_ptr->address = DC_CAN_BASE + DC_CRUISE1;
				can_push_ptr->status = 8;
				can_push_ptr->data.data_32[1] = target; 
				can_push_ptr->data.data_32[0] = esum;
				can_push();
				
				// Transmit cruise control settings 
				can_push_ptr->address = DC_CAN_BASE + DC_CRUISE2;
				can_push_ptr->status = 8;
				can_push_ptr->data.data_u8[7] = cruising;
				can_push_ptr->data.data_u8[6] = 0;  //unused
				can_push_ptr->data.data_u8[5] = 0;  //unused
				can_push_ptr->data.data_u8[4] = 0;  //unused
				can_push_ptr->data.data_32[0] = PID_setpoint;
				can_push();
				
                can_push_ptr->address = DC_CAN_BASE + DC_PID_READ;
				can_push_ptr->status = 8;
				can_push_ptr->data.data_u16[3] = 0;
				can_push_ptr->data.data_u16[2] = Kd;  
				can_push_ptr->data.data_u16[1] = Ki;  
				can_push_ptr->data.data_u16[0] = Kp;
				can_push();
				
				
				// Transmit our ID frame at a slower rate (every 10 events = 1/second)
				comms_event_count++;
				if(comms_event_count == 10){
					comms_event_count = 0;
					can_push_ptr->address = DC_CAN_BASE;
					can_push_ptr->status = 8;
					can_push_ptr->data.data_u8[7] = 'T';
					can_push_ptr->data.data_u8[6] = '0';
					can_push_ptr->data.data_u8[5] = '8';
					can_push_ptr->data.data_u8[4] = '6';
					can_push_ptr->data.data_u32[0] = DEVICE_SERIAL;
					can_push();		
				}
			}
		}

		// Check for CAN packet reception
		if((P2IN & CAN_INTn) == 0x00){
			// IRQ flag is set, so run the receive routine to either get the message, or the error
			can_receive();
			// Check the status
			if(can.status == CAN_OK){
				// We've received a packet, so must be connected to something
				events |= EVENT_CONNECTED;
				// Process the packet
				switch(can.address){
					case MC_CAN_BASE + MC_VELOCITY:
						// Update speed threshold event flags
						if(can.data.data_fp[0] > ENGAGE_VEL_F) events |= EVENT_FORWARD;
						else events &= ~EVENT_FORWARD;
						if(can.data.data_fp[0] < ENGAGE_VEL_R) events |= EVENT_REVERSE;
						else events &= ~EVENT_REVERSE;
						if((can.data.data_fp[0] >= ENGAGE_VEL_R) && (can.data.data_fp[0] <= ENGAGE_VEL_F)) events |= EVENT_SLOW;
						else events &= ~EVENT_SLOW;
						if(can.data.data_fp[0] >= CHANGE_VEL_LTOH) events |= EVENT_OVER_VEL_LTOH;
						else events &= ~EVENT_OVER_VEL_LTOH;
						if(can.data.data_fp[0] >= CHANGE_VEL_HTOL) events |= EVENT_OVER_VEL_HTOL;
						else events &= ~EVENT_OVER_VEL_HTOL;
						motor_rpm = can.data.data_fp[0];
						gauge_tach_update( motor_rpm );
						break;
					case MC_CAN_BASE + MC_I_VECTOR:
						// Update regen status flags
						if(can.data.data_fp[0] < REGEN_THRESHOLD) events |= EVENT_REGEN;
						else events &= ~EVENT_REGEN;
						break;
					case MC_CAN_BASE + MC_TEMP1:
						// Update data for temp gauge
						controller_temp = can.data.data_fp[1];
						motor_temp = can.data.data_fp[0];
						gauge_temp_update( motor_temp, controller_temp );
						break;
					case MC_CAN_BASE + MC_LIMITS:
						// Update neutral state of motor controller
						if(can.data.data_u8[0] == 0) events |= EVENT_MC_NEUTRAL;
						else events &= ~EVENT_MC_NEUTRAL;
						break;
					case MC_CAN_BASE + MC_BUS:
						// Update battery voltage and current for fuel and power gauges
						battery_voltage = can.data.data_fp[0];
						battery_current = can.data.data_fp[1];
						gauge_power_update( battery_voltage, battery_current );
						gauge_fuel_update( battery_voltage );
						break;
					case DC_CAN_BASE + DC_BOOTLOAD:
						// Switch to bootloader
						if (		can.data.data_u8[0] == 'B' && can.data.data_u8[1] == 'O' && can.data.data_u8[2] == 'O' && can.data.data_u8[3] == 'T'
								&&	can.data.data_u8[4] == 'L' && can.data.data_u8[5] == 'O' && can.data.data_u8[6] == 'A' && can.data.data_u8[7] == 'D' )
						{
							WDTCTL = 0x00;	// Force watchdog reset
						}
						break;
                    case DC_CAN_BASE + DC_CRUISE_SET:
                        //  Cruise control setting
                        if (can.data.data_u8[7] == 1)  remote_target = can.data.data_32[0];
                        else if (can.data.data_u8[7] == 2) clr_cruise_control();
						break; 
				
				    case DC_CAN_BASE + DC_DIRTY_HARRY:
                        //  Cruise control setting
                        if (can.data.data_u8[7] == 1){
   						     remote_target = can.data.data_32[0];
							 enable_cruise_target();
							 }
                        else if (can.data.data_u8[7] == 2) clr_cruise_control();
						break; 
				
   				    case DC_CAN_BASE + DC_PID:
                        //  Cruise control setting
                        Kp = can.data.data_u16[0];
						Ki = can.data.data_u16[1];
						Kd = can.data.data_u16[2];
						break; 
						
						
					case EG_CAN_BASE + EG_STATUS:
						if ( can.data.data_u8[0] == EG_STATE_NEUTRAL ) current_egear = EG_STATE_NEUTRAL;
						else if ( can.data.data_u8[0] == EG_STATE_LOW ) current_egear = EG_STATE_LOW;
						else if ( can.data.data_u8[0] == EG_STATE_HIGH ) current_egear = EG_STATE_HIGH;
						break;
				}
			}
			if(can.status == CAN_RTR){
				// Remote request packet received - reply to it
				switch(can.address){
					case DC_CAN_BASE:
						can_push_ptr->address = can.address;
						can_push_ptr->status = 8;
						can_push_ptr->data.data_u8[3] = 'T';
						can_push_ptr->data.data_u8[2] = '0';
						can_push_ptr->data.data_u8[1] = '8';
						can_push_ptr->data.data_u8[0] = '6';
						can_push_ptr->data.data_u32[1] = DEVICE_SERIAL;
						can_push();
						break;
					case DC_CAN_BASE + DC_DRIVE:
						can_push_ptr->address = can.address;
						can_push_ptr->status = 8;
						can_push_ptr->data.data_fp[1] = command.current;
						can_push_ptr->data.data_fp[0] = command.rpm;
						can_push();
						break;
					case DC_CAN_BASE + DC_POWER:
						can_push_ptr->address = can.address;
						can_push_ptr->status = 8;
						can_push_ptr->data.data_fp[1] = command.bus_current;
						can_push_ptr->data.data_fp[0] = 0.0;
						can_push();
						break;
					case DC_CAN_BASE + DC_SWITCH:
						can_push_ptr->address = can.address;
						can_push_ptr->status = 8;
						can_push_ptr->data.data_u8[7] = command.state;
						can_push_ptr->data.data_u8[6] = command.flags;
						can_push_ptr->data.data_u16[2] = vin;  //send voltage rail
						can_push_ptr->data.data_u16[1] = switches2; 
						can_push_ptr->data.data_u16[0] = switches1;
						can_push();
						break;
					case DC_CAN_BASE + DC_CRUISE1:
	         			can_push_ptr->address = can.address;
				        can_push_ptr->status = 8;
				        can_push_ptr->data.data_u32[1] = target; 
				        can_push_ptr->data.data_32[0] = esum;
				        can_push();
						break;
			    	case DC_CAN_BASE + DC_CRUISE2:
	         			can_push_ptr->address = can.address;
				        can_push_ptr->status = 8;
				    	can_push_ptr->data.data_u8[7] = cruising;
				        can_push_ptr->data.data_u8[6] = 0;  //unused
				        can_push_ptr->data.data_u8[5] = 0;  //unused
				        can_push_ptr->data.data_u8[4] = 0;  //unused
				        can_push_ptr->data.data_u32[0] = PID_setpoint;
				        can_push();
						break;
                    case DC_CAN_BASE + DC_PID_READ:
					    can_push_ptr->address = can.address;
				        can_push_ptr->status = 8;
				    	can_push_ptr->data.data_u16[3] = 0;
				        can_push_ptr->data.data_u16[2] = Kd;  
				        can_push_ptr->data.data_u16[1] = Ki;  
				        can_push_ptr->data.data_u16[0] = Kp;
				        can_push();
                        break;
				}
			}
			if(can.status == CAN_ERROR){
			}
		}
	}
	
	// Will never get here, keeps compiler happy
	return(1);
}


/*
 * Delay function
 */
static void __inline__ brief_pause(register unsigned int n)
{
    __asm__ __volatile__ (
		"1: \n"
		" dec	%[n] \n"
		" jne	1b \n"
        : [n] "+r"(n));
}

/*
 * Initialise clock module
 *	- Setup MCLK, ACLK, SMCLK dividers and clock sources
 *	- ACLK  = 0
 *	- MCLK  = 16 MHz internal oscillator
 *	- SMCLK = 16 MHz internal oscillator
 *
 * Note: We can also use the 2, 4, 8 or 16MHz crystal clock output from the MCP2515 CAN controller, which
 *       is a more accurate source than the internal oscillator.  However, using the internal
 *       source makes using sleep modes for both the MSP430 and the MCP2515 much simpler.
 */
void clock_init( void )
{
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;
}

/*
 * Initialise I/O port directions and states
 *	- Drive unused pins as outputs to avoid floating inputs
 *
 */
void io_init( void )
{
	P1OUT = 0x00;
	P1DIR = BRAKE_OUT | REVERSE_OUT | CAN_PWR_OUT | P1_UNUSED;
	
	P2OUT = 0x00;
	P2DIR = P2_UNUSED;
	
	P3OUT = CAN_CSn | EXPANSION_TXD | LED_REDn | LED_GREENn;
	P3DIR = CAN_CSn | CAN_MOSI | CAN_SCLK | EXPANSION_TXD | LED_REDn | LED_GREENn | P3_UNUSED;
	
	P4OUT = LED_PWM;
	P4DIR = GAUGE_1_OUT | GAUGE_2_OUT | GAUGE_3_OUT | GAUGE_4_OUT | LED_PWM | P4_UNUSED;
	
	P5OUT = 0x00;
	P5DIR = LED_HAZ | LED_FAULT_2 | LED_FAULT_3 | LED_GEAR_BL | LED_GEAR_4 | LED_GEAR_3 | LED_GEAR_2 | LED_GEAR_1 | P5_UNUSED;
	
	P6OUT = 0x00;
	P6DIR = ANLG_V_ENABLE | P6_UNUSED;
}


/*
 * Initialise Timer A
 *	- Provides timer tick timebase at 100 Hz
 */
void timerA_init( void )
{
	TACTL = TASSEL_2 | ID_3 | TACLR;			// MCLK/8, clear TAR
	TACCR0 = (INPUT_CLOCK/8/TICK_RATE);			// Set timer to count to this value = TICK_BASE overflow
	TACCTL0 = CCIE;								// Enable CCR0 interrrupt
	TACTL |= MC_1;								// Set timer to 'up' count mode
}


/*
 * Initialise Timer B
 *	- Provides PWM and pulse outputs for gauges
 *	- 10000Hz timer ISR
 *	- With 16MHz clock/8 = 200 count resolution on PWM
 *
 */
void timerB_init( void )
{
	TBCTL = TBSSEL_2 | ID_3 | TBCLR;			// MCLK/8, clear TBR
	TBCCR0 = GAUGE_PWM_PERIOD;					// Set timer to count to this value
	TBCCR3 = 0;									// Gauge 3
	TBCCTL3 = OUTMOD_7;
	TBCCR4 = 0;									// Gauge 4
	TBCCTL4 = OUTMOD_7;
	P4SEL |= GAUGE_3_OUT | GAUGE_4_OUT;			// PWM -> output pins for fuel and temp gauges (tacho and power are software freq outputs)
	TBCCTL0 = CCIE;								// Enable CCR0 interrupt
	TBCTL |= MC_1;								// Set timer to 'up' count mode
}

/*
 * Initialise A/D converter
 */
void adc_init( void )
{
	// Enable A/D input channels											
	P6SEL |= ANLG_SENSE_A | ANLG_SENSE_B | ANLG_SENSE_C | ANLG_SENSE_V | ANLG_BRAKE_I | ANLG_REVERSE_I | ANLG_CAN_PWR_I;
	// Turn on ADC12, set sampling time = 256 ADCCLK, multiple conv, start internal 2.5V reference
	ADC12CTL0 = ADC12ON | SHT0_8 | SHT1_8 | MSC | REFON | REF2_5V;	
	// Use sampling timer, ADCCLK = MCLK/4, run a single sequence per conversion start
	ADC12CTL1 = ADC12SSEL_2 | ADC12DIV_3 | SHP | CONSEQ_1;
	// Map conversion channels to input channels & reference voltages
	ADC12MCTL0 = INCH_3 | SREF_1;			// Analog A
	ADC12MCTL1 = INCH_2 | SREF_1;			// Analog B
	ADC12MCTL2 = INCH_1 | SREF_1;			// Analog C
	ADC12MCTL3 = INCH_4 | SREF_1;			// Analog V Supply
	ADC12MCTL4 = INCH_5 | SREF_1;			// Brake light current
	ADC12MCTL5 = INCH_6 | SREF_1;			// Reverse light current
	ADC12MCTL6 = INCH_7 | SREF_1 | EOS;		// CAN Bus current / End of sequence
	// Enable interrupts on final conversion in sequence
	ADC12IE = BIT6;	
	// Enable conversions
	ADC12CTL0 |= ENC;											
}

/*
 * Timer B CCR0 Interrupt Service Routine
 *	- Interrupts on Timer B CCR0 match at GAUGE_FREQUENCY (10kHz)
 */
interrupt(TIMERB0_VECTOR) timer_b0(void)
{
	static unsigned int gauge_count;
	static unsigned int gauge1_on, gauge1_off;
	static unsigned int gauge2_on, gauge2_off;
	
	// Toggle gauge 1 & 2 pulse frequency outputs
	if(gauge_count == gauge1_on){
		P4OUT |= GAUGE_1_OUT;
		gauge1_on = gauge_count + gauge.g1_count;
		gauge1_off = gauge_count + (gauge.g1_count >> 2);
	}
	if(gauge_count == gauge1_off){
		P4OUT &= ~GAUGE_1_OUT;
	}

	if(gauge_count == gauge2_on){
		P4OUT |= GAUGE_2_OUT;
		gauge2_on = gauge_count + gauge.g2_count;
		gauge2_off = gauge_count + (gauge.g2_count >> 2);
	}
	if(gauge_count == gauge2_off){
		P4OUT &= ~GAUGE_2_OUT;
	}

	// Update pulse output timebase counter
	gauge_count++;
	
	// Update outputs if necessary
	if(events & EVENT_GAUGE1){
		events &= ~EVENT_GAUGE1;
	}
	if(events & EVENT_GAUGE2){
		events &= ~EVENT_GAUGE2;
	}
	if(events & EVENT_GAUGE3){
		events &= ~EVENT_GAUGE3;
		TBCCR3 = gauge.g3_duty;		
	}
	if(events & EVENT_GAUGE4){
		events &= ~EVENT_GAUGE4;
		TBCCR4 = gauge.g4_duty;		
	}	
}

/*
 * Timer A CCR0 Interrupt Service Routine
 *	- Interrupts on Timer A CCR0 match at 100Hz
 *	- Sets Time_Flag variable
 */
interrupt(TIMERA0_VECTOR) timer_a0(void)
{
	static unsigned char comms_count = COMMS_SPEED;
	static unsigned char activity_count;
	
    	// Trigger timer based events
	    events |= EVENT_TIMER;	
	
		// Trigger comms events (command packet transmission)
		comms_count--;
		if( comms_count == 0 ){
			comms_count = COMMS_SPEED;
			events |= EVENT_COMMS;
		}
		
		// Check for CAN activity events and blink LED
		if(events & EVENT_CAN_ACTIVITY){
			events &= ~EVENT_CAN_ACTIVITY;
			activity_count = ACTIVITY_SPEED;
			P3OUT &= ~LED_GREENn;
		}
		if( activity_count == 0 ){
			P3OUT |= LED_GREENn;
		}
		else{
			activity_count--;
		}
}

/*
 * Collect switch inputs from hardware and fill out current state, and state changes
 *	- Inverts active low switches so that all bits in register are active high
 */
void update_switches( unsigned int *state1, unsigned int *state2)
{

    *state1 |= SW1_IGN_START;  // enable system automatically
 
	
	
	// Import switches into register
	if(P2IN & IN_GEAR_3) *state1 |= SW1_MODE_R;
	else *state1 &= ~SW1_MODE_R;

	if(P2IN & IN_GEAR_2) *state1 |= SW1_MODE_N;
	else *state1 &= ~SW1_MODE_N;

	if(P2IN & IN_GEAR_1) *state1 |= SW1_MODE_D;
	else *state1 &= ~SW1_MODE_D;
	
	if(P2IN & IN_CRUISE) *state2 |= SW2_CRUISE;
	else *state2 &= ~SW2_CRUISE;
	
	if ((P2IN & IN_GEAR_1) && (P2IN & IN_CRUISE))  *state2 |= SW2_CRUISE_REMOTE;
	else *state2 &= ~SW2_CRUISE_REMOTE;
	
	
	
	if(P1IN & IN_REGEN_PBn) *state2 &= ~SW2_REGEN_PB;
	else *state2 |= SW2_REGEN_PB;
	
	if(P1IN & IN_HAZ_ONn) *state2 &= ~SW2_HAZ_ON;
	else *state2 |= SW2_HAZ_ON;

	if(P1IN & IN_THROTTLE_SELn) *state2 &= ~SW2_THROTTLE_SEL;
	else *state2 |= SW2_THROTTLE_SEL;

	if(P1IN & IN_BRAKEn) *state1 &= ~SW1_BRAKE;
	else *state1 |= SW1_BRAKE;

	if(P1IN & IN_HORN) *state2 |= SW2_HORN;
	else *state2 &= ~SW2_HORN;

	if (ADC12MEM1 < ADC_LEVEL_1) *state2 |= SW2_LEFT_IND;
	else if (ADC12MEM1 > ADC_LEVEL_2) *state2 |= SW2_RIGHT_IND;
	else *state2 &= ~(SW2_LEFT_IND | SW2_RIGHT_IND);
	

}

  
  


/*
 * ADC12 Interrupt Service Routine
 *	- Interrupts on channel 6 conversion (end of sequence)
 */
interrupt(ADC12_VECTOR) adc_isr(void)
{
	// Clear ISR flag
	ADC12IFG &= ~BIT6;
	// Trigger ADC event in main loop
	events |= EVENT_ADC;
}



