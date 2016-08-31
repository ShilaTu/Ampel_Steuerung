
/* Johannes Zink
 * Shila Ampelsteuerung
 * 24.02.2016
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "usb_debug_only.h"
#include "print.h"


// Teensy 2.0: LED is active high
#if defined(__AVR_ATmega32U4__) || defined(__AVR_AT90USB1286__)
#define LED_ON		(PORTD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))
#endif

#define LED_CONFIG	(DDRD |= (1<<6))

#define RED_CFG 	(DDRD |= (1<<7))
#define YELLOW_CFG 	(DDRB |= (1<<6))
#define GREEN_CFG 	(DDRF |= (1<<1))

#define IN0_CFG		(DDRD &= ~(1<<0))
#define IN1_CFG		(DDRD &= ~(1<<1))
#define IN2_CFG		(DDRD &= ~(1<<2))

#define IN0_PORT	(PORTD &= ~(1<<0))
#define IN1_PORT	(PORTD &= ~(1<<1))
#define IN2_PORT	(PORTD &= ~(1<<2))

#define RED_ON		(PORTD |= (1<<7))
#define RED_OFF		(PORTD &= ~(1<<7))

#define YELLOW_ON	(PORTB |= (1<<6))
#define YELLOW_OFF	(PORTB &= ~(1<<6))

#define GREEN_ON	(PORTF |= (1<<1))
#define GREEN_OFF	(PORTF &= ~(1<<1))

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

typedef enum {TRAFFIC, SWITCH, DEMO} state_t;
typedef enum {START, RED, RYELLOW, GREEN, GYELLOW} traffic_t;

volatile unsigned int msec_cnt = 0;
volatile unsigned int sec_cnt = 0;
volatile unsigned int sel_trigger_cnt = 0;
volatile unsigned int sel_trigger = 0;
volatile unsigned int sel_led_cnt = 0;
volatile unsigned int sel_y = 0;

volatile traffic_t light = RED;
volatile unsigned int traffic_cnt = 0;

volatile unsigned int toggle_led = 0;

volatile unsigned int toggle_green = 0;
volatile unsigned int toggle_yellow = 0;
volatile unsigned int toggle_red = 0;

int main(void)
{
	// set 16 MHz system clock
	CPU_PRESCALE(0);
	
	// config onboard led 
	LED_CONFIG;
	
	// config traffic light outputs
	RED_CFG;
	YELLOW_CFG;
	GREEN_CFG;
	
	// config inputs
	IN0_CFG;
	IN1_CFG;
	IN2_CFG;
	
	// pullups off
	IN0_PORT;
	IN1_PORT;
	IN2_PORT;
	
	// led off 
	LED_OFF;
	
	// traffic light off
	RED_OFF;
	YELLOW_OFF;
	GREEN_OFF;
	
	// all lights on
	LED_ON;
	RED_ON;
	YELLOW_ON;
	GREEN_ON;
	
	_delay_ms(1000);
	
	// all traffic lights off
	RED_OFF;
	YELLOW_OFF;
	GREEN_OFF;

	_delay_ms(1000);
	
	// initialize USB, but don't wait for the host to
	// configure.
	usb_init();

	// start sequence
	RED_ON;

	_delay_ms(200);
	
	RED_OFF;
	YELLOW_ON;
	
	_delay_ms(200);
	
	RED_OFF;
	YELLOW_OFF;
	GREEN_ON;
	
	_delay_ms(200);
	
	RED_OFF;
	YELLOW_ON;
	GREEN_OFF;
	
	_delay_ms(200);
	
	RED_ON;
	YELLOW_OFF;
	GREEN_OFF;

	// init state = IDLE
	state_t state = TRAFFIC;
	light = START;
	
	sel_trigger_cnt = 0;
	sel_led_cnt = 0;

	// timer0 config
	TCCR0A |= (1<<WGM01);
	// timer0 start
	TCCR0B |= (1<<CS01) | (1<<CS00);
	// timer0 compare value -> 1ms tick
	OCR0A = 250-1;
	// timer0 interrupt config 
	TIMSK0 |= (1<<OCIE0A);
	
	// int0 interrupt config
	// enable INT0	
	EICRA |= (1<<ISC01);
	EICRA |= (1<<ISC00);
	// config INT1	
	EICRA |= (1<<ISC11);
	EICRA |= (1<<ISC10);
	// config INT2	
	EICRA |= (1<<ISC21);
	EICRA |= (1<<ISC20);
	// enable int0 & int1 
	EIMSK |= (1<<INT0) | (1<<INT1) | (1<<INT2);	

		
	// enabel global interrupts
	sei();

	while (1) {
	
		switch(state)
		{
			case TRAFFIC:
			{
				if(light == START)
				{
					YELLOW_OFF;
					GREEN_OFF;
					light = RED;
					RED_ON;
				}
					
				if(light == RED)
				{
					if(traffic_cnt == 40000)
					{
						cli();
						RED_ON;
						YELLOW_ON;
						traffic_cnt = 0;
						light = RYELLOW;
						sei();
					}
				}
				
				if(light == RYELLOW)
				{
					if(traffic_cnt == 3000)
					{
						cli();
						RED_OFF;
						YELLOW_OFF;
						GREEN_ON;
						traffic_cnt = 0;
						light = GREEN;
						sei();
					}
				}
				
				if(light == GREEN)
				{
					if(traffic_cnt == 40000)
					{
						cli();
						GREEN_OFF;
						YELLOW_ON;
						traffic_cnt = 0;
						light = GYELLOW;
						sei();
					}
				}
				
				if(light == GYELLOW)
				{
					if(traffic_cnt == 3000)
					{
						cli();
						YELLOW_OFF;
						RED_ON;
						traffic_cnt = 0;
						light = RED;
						sei();
					}
				}
				
				state = TRAFFIC;
				break;
			}
			case SWITCH:
			{
				if(toggle_red == 1)
				{
					RED_ON;
					YELLOW_OFF;
					GREEN_OFF;
					toggle_red = 0;
				}
				else if(toggle_yellow == 1)
				{
					RED_OFF;
					YELLOW_ON;
					GREEN_OFF;
					toggle_yellow = 0;
				}
				else if(toggle_green == 1)
				{
					RED_OFF;
					YELLOW_OFF;
					GREEN_ON;
					toggle_green = 0;
				}
				
				state = SWITCH;
				break;
			}
			case DEMO:
			{
				
				break;
			}
			default:
			{
				
				break;
			}
		
		}
		
		if(sel_trigger == 1 && sel_trigger_cnt == 3000 && (PIND & (1<<2)))
		{
			if(state == SWITCH)
			{
				RED_OFF;
				YELLOW_OFF;
				GREEN_OFF;
				state = TRAFFIC;
				light = START;
			}
			else if(state == TRAFFIC)
			{
				state = SWITCH;
			}
			
			sel_trigger_cnt = 0;
			sel_trigger = 0;
		}
		
		if(sel_trigger == 1 && sel_trigger_cnt < 3000)
		{
			if(PIND & (1<<2))
			{
			}
			else
			{
				sel_trigger_cnt = 0;
				sel_trigger = 0;
			}
		}
	}
}

ISR (TIMER0_COMPA_vect)
{
  msec_cnt++;
  
  if(sel_trigger == 1)
	sel_trigger_cnt++;
  
  sel_led_cnt++;
  traffic_cnt++;
  
  if (msec_cnt == 1000)
  {
	msec_cnt = 0;
	sec_cnt++;
	if(toggle_led == 0)
	{
		LED_ON;
		toggle_led = 1;
	}
	else
	{
		LED_OFF;
		toggle_led = 0;
	}
  }
}

ISR(INT0_vect)
{
	toggle_green = 1;
}

ISR(INT1_vect)
{
	toggle_red = 1;
}

ISR(INT2_vect)
{
	toggle_yellow = 1;
	sel_trigger = 1;
}
