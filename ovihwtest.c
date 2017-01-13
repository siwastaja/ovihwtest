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

#define CMD_NOP 0
#define CMD_OPEN 1
#define CMD_CLOSE 2
#define CMD_STOP 3
volatile int8_t cmd;

ISR(USART1_RX_vect)
{
	char byte = UDR1;
	if(byte == 'o')
	{
		cmd = CMD_OPEN;
	}
	else if(byte == 'c')
	{
		cmd = CMD_CLOSE;
	}
	else if(byte == 's')
	{
		cmd = CMD_STOP;
	}
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

ISR(INT6_vect) // Motor 2 overcurrent
{
	if(!mot_dirs[1])
		PWM_2 = MIN_PWM;
	else
		PWM_2 = MAX_PWM;
	overcurrent_pending[1] = 2;
	cbi(EIMSK, 6);
}

volatile int16_t isr_timer;
volatile int8_t sig_100ms;

ISR(TIMER0_OVF_vect)
{
	isr_timer++;
	if(isr_timer > 358)
		sig_100ms = 1;

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

#define MIN_SPEED 5
#define MAX_SPEED 150
#define MIN_CURR_LIMIT 5
#define MAX_CURR_LIMIT 90

void set_motor_speed(uint8_t speed)
{
	if(speed < MIN_SPEED)
		speed = MIN_SPEED;
	else if(speed > MAX_SPEED)
		speed = MAX_SPEED;

	pwm_requests[1] = speed;
}

void set_motor_curr_limit(uint8_t limit)
{
	if(limit < MIN_CURR_LIMIT)
		limit = MIN_CURR_LIMIT;
	else if(limit > MAX_CURR_LIMIT)
		limit = MAX_CURR_LIMIT;

	CUR_LIMIT_PWM_2 = limit;
}

int8_t cur_state;

#define S_UNKNOWN 0

#define S_OPEN_START 1
#define S_OPEN_RAMPUP 2
#define S_OPEN_STEADY 3
#define S_OPEN_RAMPDOWN 4
#define S_OPEN_PUSH 5
#define S_OPENED 6

#define S_CLOSE_START 7
#define S_CLOSE_RAMPUP 8
#define S_CLOSE_STEADY 9
#define S_CLOSE_RAMPDOWN 10
#define S_CLOSE_PUSH 11
#define S_CLOSED 12

#define S_STOP_RAMPDOWN 13
#define S_STOPPED 14

#define S_FAULT_RAMPDOWN 15
#define S_FAULT_STOPPED 16

#define NUM_STATES 17

typedef struct
{
	char name[10];
	int16_t safety_max_duration; // if exceded, error message is generated and door goes to fault state.  0 = off (no checking)
} state_params_t;

int16_t time;

const state_params_t states[NUM_STATES] =
{
 /*0*/	{"UNKNOWN  ", 0},
	{"OP_START ", 2},
	{"OP_ACCEL ", 50},
	{"OP_STEADY", 300},
	{"OP_DECEL ", 50},
	{"OP_PUSH  ", 30},
 /*6*/	{"OPENED   ", 5000},
	{"CL_START ", 2},
	{"CL_ACCEL ", 100},
	{"CL_STEADY", 300},
	{"CL_DECEL ", 50},
	{"CL_PUSH  ", 16},
 /*12*/	{"CLOSED   ", 0},
	{"ST_DECEL ", 10},
	{"STOPPED  ", 10000},
	{"FA_DECEL ", 5},
	{"FAULTED  ", 0}
};

#define SENSOR_ALMOST_OPEN()   (!(PINC&(1<<2)))
#define SENSOR_FULLY_OPEN()    (!(PING&(1<<2)))
#define SENSOR_ALMOST_CLOSED() (!(PINC&(1<<1)))
#define SENSOR_FULLY_CLOSED()  (!(PINC&(1<<6)))
#define SENSOR_MAN_IN_MIDDLE() (!(PINA&(1<<5)))
//#define SENSOR_MAN_OUT_MIDDLE() (!(PINA&(1<<5)))
#define BUT_OPEN()  (!(PINA&(1<<2)))
#define BUT_CLOSE() (!(PINA&(1<<1)))
#define BUT_STOP()  ((PINA&(1<<0)))


int8_t errcode;

int8_t open_pending;
int8_t close_pending;

void error(int8_t code)
{
	errcode = code;
	cur_state = S_FAULT_RAMPDOWN;
}

void open()
{
	if(	cur_state == S_OPEN_START ||
		cur_state == S_OPEN_RAMPUP ||
		cur_state == S_OPEN_STEADY ||
		cur_state == S_OPEN_RAMPDOWN ||
		cur_state == S_OPEN_PUSH ||
		cur_state == S_OPENED)
		return;

	if(	cur_state == S_CLOSE_START ||
		cur_state == S_CLOSE_RAMPUP ||
		cur_state == S_CLOSE_STEADY ||
		cur_state == S_CLOSE_RAMPDOWN ||
		cur_state == S_CLOSE_PUSH)
	{
		cur_state = S_STOP_RAMPDOWN; // Stop first...
		open_pending = 1;
		return;
	}

	cur_state = S_OPEN_START;
}

void close()
{
	if(	cur_state == S_CLOSE_START ||
		cur_state == S_CLOSE_RAMPUP ||
		cur_state == S_CLOSE_STEADY ||
		cur_state == S_CLOSE_RAMPDOWN ||
		cur_state == S_CLOSE_PUSH ||
		cur_state == S_CLOSED)
		return;

	if(SENSOR_MAN_IN_MIDDLE())
		return;

	if(	cur_state == S_OPEN_START ||
		cur_state == S_OPEN_RAMPUP ||
		cur_state == S_OPEN_STEADY ||
		cur_state == S_OPEN_RAMPDOWN ||
		cur_state == S_OPEN_PUSH)
	{
		cur_state = S_STOP_RAMPDOWN; // Stop first...
		close_pending = 1;
		return;
	}

	cur_state = S_CLOSE_START;
}

void stop()
{
	if(cur_state == S_STOP_RAMPDOWN || cur_state == S_STOPPED)
		return;
	cur_state = S_STOP_RAMPDOWN;
}

void fsm()
{
	switch(cur_state)
	{
		case S_UNKNOWN:
		{

			MOT_2_DISABLE();
		}

		break;

		case S_OPEN_START:
		{

			MOT_2_DISABLE();
			_delay_ms(20); // Make sure current has decayed before switching the relay
			pwm_requests[1] = 30;  // INITIAL SPEED
			CUR_LIMIT_PWM_2 = 80;  // CURRENT LIMIT
			MOT_2_DIR_B();
			_delay_ms(50); // Let the relay switch!
			MOT_2_ENABLE();

			cur_state = S_OPEN_RAMPUP;
		}

		break;

		case S_OPEN_RAMPUP:
		{

			pwm_requests[1] += 4;
			if(pwm_requests[1] > MAX_SPEED)
				pwm_requests[1] = MAX_SPEED;

			if(time > 20)
				cur_state = S_OPEN_STEADY;

			if(SENSOR_ALMOST_OPEN())
				cur_state = S_OPEN_RAMPDOWN;

			if(SENSOR_FULLY_OPEN())
				error(1);

		}
		break;

		case S_OPEN_STEADY:
		{

			// Slow down a little bit when we
			if(time == 100)
				pwm_requests[1] -= 5;

			if(time == 120)
				pwm_requests[1] -= 5;

			if(time == 130)
				pwm_requests[1] -= 5;

			if(SENSOR_ALMOST_OPEN())
				cur_state = S_OPEN_RAMPDOWN;

			if(SENSOR_FULLY_OPEN())
				error(2);

		}
		break;

		case S_OPEN_RAMPDOWN:
		{

			if(pwm_requests[1] > 50)
				pwm_requests[1] -= 8;

			if(time > 12)
				cur_state = S_OPEN_PUSH;

			if(SENSOR_FULLY_OPEN())
				cur_state = S_OPEN_PUSH;

		}
		break;

		case S_OPEN_PUSH:
		{

			CUR_LIMIT_PWM_2 = 70;

			if(SENSOR_FULLY_OPEN())
				cur_state = S_OPENED;


		}
		break;

		case S_OPENED:
		{

			MOT_2_DISABLE();

			if(!SENSOR_FULLY_OPEN())
				error(10);
		}

		break;

		case S_CLOSE_START:
		{

			MOT_2_DISABLE();
			_delay_ms(20); // Make sure current has decayed before switching the relay
			pwm_requests[1] = 15;  // INITIAL SPEED
			CUR_LIMIT_PWM_2 = 80;  // CURRENT LIMIT
			MOT_2_DIR_A();
			_delay_ms(50); // Let the relay switch!
			MOT_2_ENABLE();

			cur_state = S_CLOSE_RAMPUP;

		}
		break;

		case S_CLOSE_RAMPUP:
		{

			pwm_requests[1] += 2;
			if(pwm_requests[1] > MAX_SPEED)
				pwm_requests[1] = MAX_SPEED;

			if(time > 50)
				cur_state = S_CLOSE_STEADY;

			if(time > 40 && (SENSOR_FULLY_OPEN() || SENSOR_ALMOST_OPEN()))
			{
				// 4 seconds passed, rampup nearly finished, but door still stuck at open or nearly open!
				error(4);
			}

			if(SENSOR_ALMOST_CLOSED())
				cur_state = S_CLOSE_RAMPDOWN;

			if(SENSOR_FULLY_CLOSED())
				error(5);

			if(SENSOR_MAN_IN_MIDDLE())
				cur_state = S_STOP_RAMPDOWN;

		}
		break;

		case S_CLOSE_STEADY:
		{

			if(SENSOR_ALMOST_CLOSED())
				cur_state = S_CLOSE_RAMPDOWN;

			if(SENSOR_FULLY_CLOSED())
				error(6);

			if(SENSOR_MAN_IN_MIDDLE())
				cur_state = S_STOP_RAMPDOWN;
		}
		break;

		case S_CLOSE_RAMPDOWN:
		{

			if(pwm_requests[1] > 50)
				pwm_requests[1] -= 4;

			if(time > 12)
				cur_state = S_CLOSE_PUSH;

			if(SENSOR_FULLY_CLOSED())
				cur_state = S_CLOSE_PUSH;

			if(SENSOR_MAN_IN_MIDDLE())
				cur_state = S_STOP_RAMPDOWN;

		}
		break;

		case S_CLOSE_PUSH:
		{

			CUR_LIMIT_PWM_2 = 70;

			if(SENSOR_FULLY_CLOSED())
				cur_state = S_CLOSED;

		}
		break;

		case S_CLOSED:
		{

			MOT_2_DISABLE();

			if(!SENSOR_FULLY_CLOSED())
				error(10);


		}
		break;

		case S_STOP_RAMPDOWN:
		{
			CUR_LIMIT_PWM_2 = 70;  // LOWER CURRENT LIMIT

			if(pwm_requests[1] > 20)
				pwm_requests[1] -= 10;
			else
				cur_state = S_STOPPED;

			if(time > 8)
				cur_state = S_STOPPED;

		}
		break;

		case S_STOPPED:
		{
			MOT_2_DISABLE();

			if(open_pending && time > 5)
			{
				open_pending = 0;
				cur_state = S_OPEN_START;
			}

			if(close_pending && time > 5)
			{
				close_pending = 0;
				cur_state = S_CLOSE_START;
			}

		}
		break;

		case S_FAULT_RAMPDOWN:
		{

			if(pwm_requests[1] > 25)
				pwm_requests[1] -= 20;
			else
				cur_state = S_FAULT_STOPPED;

			if(time > 8)
				cur_state = S_FAULT_STOPPED;
		}

		break;

		case S_FAULT_STOPPED:
		{

			MOT_2_DISABLE();
		}

		break;

		default:
		{
			error(8);
		}
		break;
	}
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
	CUR_LIMIT_PWM_2 = 10;
	pwm_requests[0] = 10;
	pwm_requests[1] = 10;

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

	DDRB |= 1<<5 /*FETOUT1*/ | 1<<6 /*FETOUT2*/;
	DDRG |= 1<<3 /*FETOUT3*/ | 1<<4 /*FETOUT4*/ | 1<<0 /*RELAY5*/ | 1<<1 /*RELAY6*/;
	DDRD |= 1<<7 /*RELAY1*/ | 1<<6 /*RELAY2*/ | 1<<0 /*RELAY3*/ | 1<<1 /*RELAY4*/;

	MOT_2_DIR_A();

	int8_t alive_time = 0;

	while(1)
	{
		char buf[10];

		cli();
		isr_timer = 0;
		sig_100ms = 0;
		sei();

		int8_t prev_state = cur_state;

		int8_t command = cmd;
		if(command == CMD_OPEN)
		{
			open();
			cmd = CMD_NOP;
		}
		else if(command == CMD_CLOSE)
		{
			close();
			cmd = CMD_NOP;
		}
		else if(command == CMD_STOP)
		{
			stop();
			cmd = CMD_NOP;
		}
		else // no command, check the buttons for manual operation
		{
			if(BUT_STOP())
				stop();
			else if(BUT_OPEN())
				open();
			else if(BUT_CLOSE())
				close();
		}

		if(cur_state != prev_state)
		{
			print_string("KAKKA1: ");
			utoa(cur_state, buf, 10);
			print_string(buf);
			print_char(',');
			utoa(prev_state, buf, 10);
			print_string(buf);
			time = 0;
			print_string("\n\r");
		}

		prev_state = cur_state;
		fsm();
		if(cur_state != prev_state)
		{
			print_string("KAKKA2\n\r");
			time = 0;
		}

		print_string(" alive_time=");
		utoa(alive_time, buf, 10);
		print_string(buf);
		if(alive_time < 10) print_char(' ');
		if(++alive_time >= 100) alive_time = 0;

		print_string(" state=");
		print_string(states[cur_state].name);

		print_string(" step_time=");
		utoa(time, buf, 10);
		print_string(buf);

		if(!(PINA&1))
			print_string(" PA0");
		if(!(PINA&2))
			print_string(" PA1");
		if(!(PINA&4))
			print_string(" PA2");
		if(!(PINA&8))
			print_string(" PA3");
		if(!(PINA&16))
			print_string(" PA4");
		if(!(PINA&32))
			print_string(" PA5");

		if(!(PINC&2))
			print_string(" PC1");
		if(!(PINC&4))
			print_string(" PC2");
		if(!(PINC&8))
			print_string(" PC3");
		if(!(PINC&16))
			print_string(" PC4");
		if(!(PINC&32))
			print_string(" PC5");
		if(!(PINC&64))
			print_string(" PC6");
		if(!(PINC&128))
			print_string(" PC7");
		if(!(PING&4))
			print_string(" PG2");


		if(SENSOR_ALMOST_CLOSED())
			print_string(" ALMOST_CLOSED");
		if(SENSOR_FULLY_CLOSED())
			print_string(" FULLY_CLOSED");
		if(SENSOR_ALMOST_OPEN())
			print_string(" ALMOST_OPEN");
		if(SENSOR_FULLY_OPEN())
			print_string(" FULLY_OPEN");
		if(SENSOR_MAN_IN_MIDDLE())
			print_string(" MAN_IN_MIDDLE");

		if(BUT_STOP())
			print_string(" BUT_STOP");
		if(BUT_OPEN())
			print_string(" BUT_OPEN");
		if(BUT_CLOSE())
			print_string(" BUT_CLOSE");


		if(states[cur_state].safety_max_duration && time > states[cur_state].safety_max_duration)
		{
			error(100);
		}

		if(errcode)
		{
			print_string(" E");
			utoa(errcode, buf, 10);
			print_string(buf);
		}

		print_char('\n');
		print_char('\r');

		while(!sig_100ms) ;  // wait for 100ms to be exceeded
		time++;

	}

}
