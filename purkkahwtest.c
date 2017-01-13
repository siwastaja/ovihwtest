#define F_CPU 7327000UL

#include <inttypes.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#define cbi(x,y) x &= ~(1<<(y))
#define sbi(x,y) x |= (1<<(y))

#define BAUD_SET_115200 3
#define BAUD_SET_9600 47

#define CUR_LIMIT_PWM_1 OCR3A
#define CUR_LIMIT_PWM_2 OCR3B
#define PWM_1  OCR0
#define PWM_2  OCR2

#define ADMUX_CURRENT_1() {ADMUX = 0b01100000;}
#define ADMUX_CURRENT_2() {ADMUX = 0b01100010;}
#define START_ADC_CONV() {ADCSRA = 0b11001101;} // Don't configure ADCSRA outside of this. It's simply rewritten each time without read-modify-write.

/*
	Direction A (0)
	* Relay coil non-powered
	* motor over the bottom MOSFET
	* Non-inverted PWM setting

	Direction B (1)
	* Relay coil powered
	* Motor over the top MOSFET
	* Inverted PWM setting
*/

volatile uint8_t mot_dirs[2];
#define MOT_1_DIR_A() {cbi(PORTE, 2); mot_dirs[0] = 0;}
#define MOT_1_DIR_B() {sbi(PORTE, 2); mot_dirs[0] = 1;}
#define MOT_2_DIR_A() {cbi(PORTB, 0); mot_dirs[1] = 0;}
#define MOT_2_DIR_B() {sbi(PORTB, 0); mot_dirs[1] = 1;}

#define MOT_1_ENABLE() {sbi(PORTF, 1);}
#define MOT_1_DISABLE() {cbi(PORTF, 1);}
#define MOT_2_ENABLE() {sbi(PORTF, 3);}
#define MOT_2_DISABLE() {cbi(PORTF, 3);}

volatile uint8_t pwm_requests[2];

#define MIN_PWM 0
#define MAX_PWM 250 // Bottom FET needs some on-time for the bootstrap to work.

#define PWM_INCREASE_STEP 20 // was 10

volatile uint8_t overcurrent_pending[2];
volatile uint8_t latest_currents[2]; // You can read these. 1 unit = 1/2.55 A


#define print_char(c) {while((UCSR1A & 0b00100000) == 0) ; UDR1 = (c);}
void print_string(char* str)
{
	while(str[0] != 0)
	{
		print_char(str[0]);
		str++;
	}
}


// Keeps converting motor currents.
ISR(ADC_vect)
{
	static uint8_t next_input = 0;
	if(next_input)
	{
		next_input = 0;
		latest_currents[1] = ADCH;
		ADMUX_CURRENT_1();
	}
	else
	{
		next_input = 1;
		latest_currents[0] = ADCH;
		ADMUX_CURRENT_2();
	}
	START_ADC_CONV();
}

volatile uint8_t run;

#define CMD_NOP 0
#define CMD_OPEN 1
#define CMD_CLOSE 2
#define CMD_STOP 3
volatile uint8_t cmd;
volatile uint16_t cmd_time;

// Test function. Not needed.
ISR(USART1_RX_vect)
{
	char byte = UDR1;
	if(byte == 'a')
		pwm_requests[1]--;
	else if(byte == 's')
		pwm_requests[1]++;
	else if(byte == 'q')
		CUR_LIMIT_PWM_2--;
	else if(byte == 'w')
		CUR_LIMIT_PWM_2++;
	else if(byte == 'n')
	{
		MOT_2_DIR_A();
	}
	else if(byte == 'm')
	{
		MOT_2_DIR_B();
	}
	else if(byte == 'R')
		run = 20;
	else if(byte == 'r')
		run = 1;
	else if(byte == 'o')
	{
		cmd = CMD_OPEN;
		cmd_time = 0;
	}
	else if(byte == 'c')
	{
		cmd = CMD_CLOSE;
		cmd_time = 0;
	}
	else if(byte == 'p')
	{
		cmd = CMD_STOP;
		cmd_time = 0;
	}

	if(pwm_requests[1] < 1) pwm_requests[1] = 1;
	if(pwm_requests[1] > 245) pwm_requests[1] = 245;
}

ISR(INT7_vect) // Motor 1 overcurrent
{
	if(!mot_dirs[0])
		PWM_1 = MIN_PWM;
	else
		PWM_1 = MAX_PWM;
	overcurrent_pending[0] = 2; // Setting to 2 ensures that at least one PWM cycle is really forced to min or max.
	cbi(EIMSK, 7);
}

volatile uint8_t overcurrents;

ISR(INT6_vect) // Motor 2 overcurrent
{
	if(!mot_dirs[1])
		PWM_2 = MIN_PWM;
	else
		PWM_2 = MAX_PWM;
	overcurrent_pending[1] = 2;
	overcurrents++;
	cbi(EIMSK, 6);
}

ISR(TIMER0_OVF_vect)
{
	if(overcurrent_pending[0])
	{
		overcurrent_pending[0]--;
	}
	else
	{
		sbi(EIMSK, 7); // Re-enable interrupt.
		// There has been no overcurrent event, or we have waited long enough,
		// the overcurrent condition has been cleared. -> Increase PWM towards the request.
		int16_t tmp = PWM_1;
		if(!mot_dirs[0])
		{
			tmp += PWM_INCREASE_STEP;
			if(tmp > pwm_requests[0])
				tmp = pwm_requests[0];
		}
		else
		{
			tmp -= (int16_t)PWM_INCREASE_STEP;
			if(tmp < (int16_t)MAX_PWM-(int16_t)pwm_requests[0])
				tmp = (int16_t)MAX_PWM-(int16_t)pwm_requests[0];
		}

		PWM_1 = tmp;
	}

	if(overcurrent_pending[1])
	{
		overcurrent_pending[1]--;
	}
	else
	{
		sbi(EIMSK, 6);
		int16_t tmp = PWM_2;
		if(!mot_dirs[1])
		{
			tmp += PWM_INCREASE_STEP;
			if(tmp > pwm_requests[1])
				tmp = pwm_requests[1];
		}
		else
		{
			tmp -= (int16_t)PWM_INCREASE_STEP;
			if(tmp < (int16_t)MAX_PWM-(int16_t)pwm_requests[1])
				tmp = (int16_t)MAX_PWM-(int16_t)pwm_requests[1];
		}

		PWM_2 = tmp;
	}

}

// Do not touch these, they are constrained by HW.
// Peak current limit: 2.55 units == 1A
#define MIN_SPEED 5
#define MAX_SPEED 249
#define MIN_CURR_LIMIT 5
#define MAX_CURR_LIMIT 100 // == 35A

void set_main_motor_speed(uint8_t speed)
{
	if(speed < MIN_SPEED)
		speed = MIN_SPEED;
	else if(speed > MAX_SPEED)
		speed = MAX_SPEED;

	pwm_requests[0] = speed;
}

void set_aux_motor_speed(uint8_t speed)
{
	if(speed < MIN_SPEED)
		speed = MIN_SPEED;
	else if(speed > MAX_SPEED)
		speed = MAX_SPEED;

	pwm_requests[1] = speed;
}

void set_main_motor_curr_limit(uint8_t limit)
{
	if(limit < MIN_CURR_LIMIT)
		limit = MIN_CURR_LIMIT;
	else if(limit > MAX_CURR_LIMIT)
		limit = MAX_CURR_LIMIT;

	CUR_LIMIT_PWM_1 = limit;
}

void set_aux_motor_curr_limit(uint8_t limit)
{
	if(limit < MIN_CURR_LIMIT)
		limit = MIN_CURR_LIMIT;
	else if(limit > MAX_CURR_LIMIT)
		limit = MAX_CURR_LIMIT;

	CUR_LIMIT_PWM_2 = limit;
}

int main()
{

	/*
		MOTOR CONTROL RELATED
	*/

	DDRE |= 1<<2; // Motor 1 dir
	DDRB |= 1<<0; // Motor 2 dir
	DDRF |= 1<<1 /*Motor 1 enable*/ | 1<<3 /*Motor 2 enable*/;

	/*
		Timer3 Channels A and B generate PWM signals for RC DACs to generate reference levels for analog current limits.
		* 8-bit "Fast PWM" mode is used (mode = 5)
		* Non-inverting PWM generation (COM bits 0b10)
		* Prescaler = 1 for maximum frequency. (27 kHz at 7 MHz)
	*/
	TCCR3A = 0b10100001;
	TCCR3B = 0b00001001;
	DDRE |= 1<<3 | 1<<4;
	CUR_LIMIT_PWM_1 = 10;
	CUR_LIMIT_PWM_2 = 70;
	pwm_requests[0] = 0;
	pwm_requests[1] = 80;

	/*
		Timer0 and Timer2 provide 8-bit PWM signals to drive the motor half bridge.
		* 8-bit "Fast PWM" mode is used (mode = 3)
		* Non-inverting PWM generation (COM bits 0b10)
		* Prescaler = 8, PWM freq @ 7 MHz = 3.5 kHz
	*/
	TCCR0 = 0b01101010;
	PWM_1 = 10;
	TCCR2 = 0b01101010;
	PWM_2 = 10;
	DDRB |= 1<<4 | 1<<7;
	TIMSK |= 1<<0; // Enable periodic interrupt (Timer0 overflow) to process PWM settings.
	/*
		External interrupts
		* INT7 = Motor 1 overcurrent
		* INT6 = Motor 2 overcurrent
		* Rising edge sensitive
	*/

	EICRB = 0b11110000;
	EIMSK |= 1<<7 | 1<<6;

	sei();

	/*
		ADC:
		* Used in single-shot mode
		* ISR changes the channel and starts the next conversion
		* With XTAL=7327000 and prescaler=32, fADC=228968kHz
		* Result is left-adjusted, only ADCH is read (8-bit data).
	*/

	ADMUX_CURRENT_1();
	START_ADC_CONV();


	/*
		MOTOR CONTROLLER RELATED ENDS

	*/



	/*
		UART:
		115200 bps, 8 bits, no parity.
		Transmitter injects two stop bits.
	*/

	UCSR1B = 0b10011000;
	UCSR1C = 0b00001110;
	UBRR1L = BAUD_SET_115200;

//	_delay_ms(100);
//	MOT_2_ENABLE();

	DDRB |= 1<<5 /*FETOUT1*/ | 1<<6 /*FETOUT2*/;
	DDRG |= 1<<3 /*FETOUT3*/ | 1<<4 /*FETOUT4*/ | 1<<0 /*RELAY5*/ | 1<<1 /*RELAY6*/;
	DDRD |= 1<<7 /*RELAY1*/ | 1<<6 /*RELAY2*/ | 1<<0 /*RELAY3*/ | 1<<1 /*RELAY4*/;

	MOT_2_DIR_A();

	while(1)
	{

		char buf[10];
//		print_string("Mot1 current: ");
//		utoa(latest_currents[0], buf, 10);
//		print_string(buf);
		print_string("  MotB current: ");
		utoa(latest_currents[1], buf, 10);
		print_string(buf);
		print_string("  OC cnt: ");
		utoa(overcurrents, buf, 10);
		print_string(buf);
		print_string("  PWM: ");
		utoa(PWM_2, buf, 10);
		print_string(buf);
		print_string("  REQ: ");
		utoa(pwm_requests[1], buf, 10);
		print_string(buf);
		print_string("  ILIM: ");
		utoa(CUR_LIMIT_PWM_2, buf, 10);
		print_string(buf);
		print_string("  DIR: ");
		utoa(mot_dirs[1], buf, 10);
		print_string(buf);

		if(!(PINA&1))
			print_string(" A0");
		if(!(PINA&2))
			print_string(" A1");
		if(!(PINA&4))
			print_string(" A2");
		if(!(PINA&8))
			print_string(" A3");
		if(!(PINA&16))
			print_string(" A4");
		if(!(PINA&32))
			print_string(" A5");

		if(!(PINC&2))
			print_string(" C1");
		if(!(PINC&4))
			print_string(" C2");
		if(!(PINC&8))
			print_string(" C3");
		if(!(PINC&16))
			print_string(" C4");
		if(!(PINC&32))
			print_string(" C5");
		if(!(PINC&64))
			print_string(" C6");
		if(!(PINC&128))
			print_string(" C7");
		if(!(PING&4))
			print_string(" G2");

/*		if(run > 0)
		{
			run--;
			MOT_2_ENABLE();
		}
		_delay_ms(100);

		if(run == 0)
		{
			MOT_2_DISABLE();
		}
*/		_delay_ms(100);

		if(cmd == CMD_OPEN)
		{
			if(cmd_time == 0)
			{
				MOT_2_DISABLE();
				_delay_ms(50);
				pwm_requests[1] = 30;
				CUR_LIMIT_PWM_2 = 80;
				MOT_2_DIR_B();
				_delay_ms(50);
				MOT_2_ENABLE();
			}
			else if(!(PINC&4))
			{
				pwm_requests[1] = 40;
				_delay_ms(100);
				MOT_2_DISABLE();
				cmd = CMD_NOP;
			}
			else if(cmd_time < 15)
			{
				pwm_requests[1] += 7;
			}
		}
		else if(cmd == CMD_CLOSE)
		{
			if(cmd_time == 0)
			{
				MOT_2_DISABLE();
				_delay_ms(50);
				pwm_requests[1] = 20;
				CUR_LIMIT_PWM_2 = 80;
				MOT_2_DIR_A();
				_delay_ms(50);
				MOT_2_ENABLE();
			}
			else if(!(PINC&2))
			{
				pwm_requests[1] = 60;
				_delay_ms(500);
				CUR_LIMIT_PWM_2 = 72;
				_delay_ms(2000);
				MOT_2_DISABLE();
				cmd = CMD_NOP;
			}
			else if(cmd_time < 50)
			{
				pwm_requests[1] += 2;
			}
		}
		else if(cmd == CMD_STOP)
		{
			MOT_2_DISABLE();
			_delay_ms(50);
			cmd = CMD_NOP;
		}

		if(cmd != CMD_NOP)
		{
			cmd_time++;
		}

		if(cmd_time > 1000)
		{
			print_string(" TIMEOUT");
			MOT_2_DISABLE();
			_delay_ms(50);
		}

		print_char('\n');
		print_char('\r');

	}

}
