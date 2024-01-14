/**
 * @file main.c
 * @author Jesutofunmi Kupoluyi (jimsufficiency@gmail.com)
 * @brief This is firmware source for my final year project - BLDC ESC
 * @version 0.1
 * @date 2023-10-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* Include required header files */
#include "main.h"

/* Select mode */
// #define DEBUG

/* Normal Operation */
#ifndef DEBUG

/* Variables */
volatile uint8_t sequence = 0; 
volatile int RX_buf_idx = 0;
uint8_t motor_speed;

/* Receiver Buffer */
uint8_t RX_buffer[32] = {0};





/* ISR function for Analog Comparator */
ISR(ANALOG_COMP_vect)
{
    for(int i = 0; i < 10; i++) {           //We check the comparator output 10 times to be sure
    if(sequence & 1)             //If step = odd (0001, 0011, 0101) 1, 3 or 5
    {
      if(!(ACSR & 0B00100000)) i -= 1; //!B00100000 -> B11011111 ACO = 0 (Analog Comparator Output = 0)
    }
    else                              //else if step is 0, 2 or 4
    {
      if((ACSR & 0B00100000))  i -= 1; //else if B00100000 -> B11011111 ACO = 1 (Analog Comparator Output = 1)
    }
  }
    commutate();
    sequence++;
    sequence %= 6;
    LED_PORT ^= _BV(WORKING_LED);
}

ISR(USART_RX_vect)
{
  RX_buffer[RX_buf_idx] = UDR0;
  RX_buf_idx++;

  if(RX_buf_idx >= 32)
    RX_buf_idx = 0;
}

/* Main function */
int main ()
{
  /** TODO: 
   * Initialise and setup Serial port
   * Setup LED indicators
   * BEMF feedback functions
   * Pin Change interrupt for PWM Input
   * Setup selection modes -> PWM & DIRECTION
   * Pin Change interrupt for Speed control buttons
  */

 /**
  * Lessons learnt:
  * To turn off timer 1 OCR1A & OCR1B PWM Outputs, 
  * simply clear the COM1A1 and COM1B1 bits and 
  * DO NOT TOUCH ANY OTHER BIT IN THE TCCR1A REGISTER!!!
  * This took me days (like a week or more) to discover :(
  * 
  */

  /* Initialising RX buffer */
  // memset(RX_buffer, 0, 32);

  /* Setting up Speed control buttons */
  // BTN_DDR |= _BV(INC_BTN) | _BV(DEC_BTN);

  /* Setting up Indicator LEDs */
  LED_DDR |= _BV(ERROR_LED) | _BV(WORKING_LED);

  /* Turn on Indicators for test */
  // LED_PORT |= _BV(ERROR_LED) | _BV(WORKING_LED);

  /*** Initialising bridge -> turn off all bridge pins ***/

  /* High side */

  DRV_HS_DDR |= _BV(A_H) | _BV(B_H) | _BV(C_H);
  DRV_HS_PORT &= ~(_BV(A_H) | _BV(B_H) | _BV(C_H));

  /* Low side */

  DRV_LS_DDR |= _BV(A_L) | _BV(B_L) | _BV(C_L);
  DRV_LS_PORT &= ~(_BV(A_L) | _BV(B_L) | _BV(C_L));

  /***------------------------------------------------***/


  /* Clear registers */
  TCCR1B = 0x00;
  TCCR1A = 0x00;

  TCCR2A = 0x00;
  TCCR2B = 0x00;

  /* Initialising Timer2 -> Fast PWM Mode, Non-Inverting, 8-bit, No clock division */
  TCCR2B = _BV(CS20);

  /* Initialising Timer1 -> Fast PWM Mode, Non-Inverting, 8-bit, No clock division */
  TCCR1B =  _BV(CS10) | _BV(WGM12);
  TCCR1A = _BV(COM1A1) | _BV(WGM10);

  
  /* Initialising comparator -> Disable interrupt */
  // DIDR1 = _BV(AIN1D); /* Disable AIN1 digital input */
  // ADCSRB = _BV(ACME);
  ADCSRA &= ~(_BV(ADEN));
  ACSR = (_BV(ACD));
  ACSR &= (_BV(ACIE));

  /* Start up speed */
  set_speed(START_SPEED);


  /* Initialise UART */
  UART_init();

  
  

  char display[16];

  /* Set global interrupt */
  sei();

  /* Open loop motor start up -> test */
  
  for (int i = 2000; i > 500; i = i - 1)
  {
    _delay_us(i);
    commutate();
    sequence++;
    sequence %= 6;
  }

  motor_speed = MIN_SPEED;

  /* Activate comparator interrupt */
  ACSR |= _BV(ACIE);
  
  /* UART Intro */
  // UART_SendString("BLDC ESC Start up test\n\r");

    while(1)
    {    
      set_speed(motor_speed);
      
      /* Speed control */
      if (BTN_PIN & _BV(INC_BTN))
      {
        if (motor_speed >= MAX_SPEED)
        {
          motor_speed = MAX_SPEED;
        }
        else
        {
          motor_speed++;
        }

        set_speed(motor_speed);
       
      }
      if (BTN_PIN & _BV(DEC_BTN))
      {
        if (motor_speed <= MIN_SPEED)
        {
          motor_speed = MIN_SPEED;
        }
        else
        {
          motor_speed--;
        }

        set_speed(motor_speed);
        
      }
      
      snprintf(display, 16, "Speed %d\n\r", motor_speed);

      UART_SendString(display);

      // _delay_ms(25);


    }

    return 0;
}

/* Commutation sequence function */
void commutate()
{
  switch (sequence)
  {
  case 0:
    AH_BL();
    BEMF_C_RISING();
    break;

  case 1:
    AH_CL();
    BEMF_B_FALLING();
    break;

  case 2:
    BH_CL();
    BEMF_A_RISING();
    break;

  case 3:
    BH_AL();
    BEMF_C_FALLING();
    break;

  case 4:
    CH_AL();
    BEMF_B_RISING();
    break;

  case 5:
    CH_BL();
    BEMF_A_FALLING();
    break;
  
  default:
    break;
  }
}

/* Reverse Commutation sequence function */
void inv_commutate()
{
  switch (sequence)
  {
    case 0:
      CH_BL();
      BEMF_A_RISING();
      break;
  
    case 1:
      CH_AL();
      BEMF_B_FALLING();
      break;

    case 2:
      BH_AL();
      BEMF_C_RISING();
      break;

    case 3:
      BH_CL();
      BEMF_A_FALLING();
      break;

    case 4:
      AH_CL();
      BEMF_B_RISING();
      break;
    
    case 5:
      AH_BL();
      BEMF_C_FALLING();
      break;
  
    default:
      break;
  }
}



/* Set motor speed */
void set_speed(uint8_t speed)
{
  /* Constrain speed value */
  if(speed > MAX_SPEED)
    speed = MAX_SPEED;
  if(speed < MIN_SPEED)
    speed = MIN_SPEED;

  /* Assign speed value to appropriate registers */
  OCR1A = speed;
  OCR1B = speed;
  OCR2A = speed;
  
}

/*** Commutation steps ***/

void AH_BL()
{
  TCCR2A = 0x00;
  TCCR1A = _BV(COM1A1) | _BV(WGM10);
  DRV_LS_PORT &= ~(_BV(A_L) | _BV(C_L)); 
  DRV_LS_PORT |= _BV(B_L); 
}
void AH_CL()
{
  TCCR2A = 0x00;
  TCCR1A = _BV(COM1A1) | _BV(WGM10);
  DRV_LS_PORT &= ~(_BV(A_L) | _BV(B_L)); 
  DRV_LS_PORT |= _BV(C_L); 
}

void BH_CL()
{
  TCCR2A = 0x00;
  TCCR1A = _BV(COM1B1) | _BV(WGM10);
  DRV_LS_PORT &= ~(_BV(A_L) | _BV(B_L)); 
  DRV_LS_PORT |= _BV(C_L); 
}

void BH_AL()
{
  TCCR2A = 0x00;
  TCCR1A = _BV(COM1B1) | _BV(WGM10);
  DRV_LS_PORT &= ~(_BV(C_L) | _BV(B_L)); 
  DRV_LS_PORT |= _BV(A_L); 
}

void CH_AL()
{
  TCCR1A &= ~(_BV(COM1A1) | _BV(COM1B1));
  TCCR2A = _BV(COM2A1) | _BV(WGM20) | _BV(WGM21);
  DRV_LS_PORT &= ~(_BV(C_L) | _BV(B_L)); 
  DRV_LS_PORT |= _BV(A_L); 
}

void CH_BL()
{
  TCCR1A &= ~(_BV(COM1A1) | _BV(COM1B1));
  TCCR2A = _BV(COM2A1) | _BV(WGM20) | _BV(WGM21);
  DRV_LS_PORT &= ~(_BV(A_L) | _BV(C_L)); 
  DRV_LS_PORT |= _BV(B_L); 
}

void stop()
{
  /* Turn off high side */
  TCCR1A &= ~(_BV(COM1A1) | _BV(COM1B1));
  TCCR2A = 0x00;

  /* Turn off low side */
  DRV_LS_PORT &= ~(_BV(A_L) | _BV(B_L) | _BV(C_L)); 
}

/***---------------------***/

/*** BEMF feedback functions **/

void BEMF_A_FALLING()
{
  ADMUX &= ~(_BV(MUX0) | _BV(MUX1) | _BV(MUX2));
  ADCSRB = _BV(ACME);
  ADCSRA &= ~_BV(ADEN);
  ACSR &= ~_BV(ACIS0);
  ACSR |= _BV(ACIS1);
}

void BEMF_A_RISING()
{
  ADMUX &= ~(_BV(MUX0) | _BV(MUX1) | _BV(MUX2));
  ADCSRB = _BV(ACME);
  ADCSRA &= ~_BV(ADEN);
  ACSR |= _BV(ACIS1) | _BV(ACIS0);
}

void BEMF_B_FALLING()
{
  ADMUX &= ~(_BV(MUX1) | _BV(MUX2));
  ADMUX |= _BV(MUX0);
  ADCSRB = _BV(ACME);
  ADCSRA &= ~_BV(ADEN);
  ACSR &= ~_BV(ACIS0);
  ACSR |= _BV(ACIS1);
}

void BEMF_B_RISING()
{
  ADMUX &= ~(_BV(MUX1) | _BV(MUX2));
  ADMUX |= _BV(MUX0);
  ADCSRB = _BV(ACME);
  ADCSRA &= ~_BV(ADEN);
  ACSR |= _BV(ACIS1) | _BV(ACIS0);
}
void BEMF_C_FALLING()
{
  ADMUX &= ~(_BV(MUX0) | _BV(MUX2));
  ADMUX |= _BV(MUX1);
  ADCSRB = _BV(ACME);
  ADCSRA &= ~_BV(ADEN);
  ACSR &= ~_BV(ACIS0);
  ACSR |= _BV(ACIS1);
}

void BEMF_C_RISING()
{
  ADMUX &= ~(_BV(MUX0) | _BV(MUX2));
  ADMUX |= _BV(MUX1);
  ADCSRB = _BV(ACME);
  ADCSRA &= ~_BV(ADEN);
  ACSR |= _BV(ACIS1) | _BV(ACIS0);
}

/**
 * @brief UART initialization function
 * 
 */
void UART_init()
{
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);	/* Turn on transmission and reception */
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);/* Use 8-bit char size */
	UBRR0L = BAUD_PRESCALE;			/* Load lower 8-bits of the baud rate */
	UBRR0H = (BAUD_PRESCALE >> 8);		/* Load upper 8-bits*/
}

/**
 * @brief UART function for receiving characters
 * 
 * @return unsigned char 
 */
unsigned char UART_RxChar()
{
	while ((UCSR0A & (1 << RXC0)) == 0);/* Wait till data is received */
	return(UDR0);		/* Return the byte */
}

/**
 * @brief UART function for transmitting characters
 * 
 * @param ch 
 */
void UART_TxChar(char ch)
{
	while (! (UCSR0A & (1<<UDRE0)));  /* Wait for empty transmit buffer */
	UDR0 = ch ;
}

/**
 * @brief UART function for transmitting strings
 * 
 * @param str 
 */
void UART_SendString(char *str)
{
	unsigned char j=0;
	
	while (str[j]!=0)		/* Send string till null */
	{
		UART_TxChar(str[j]);	
		j++;
	}
}

/***------------------------***/

#endif /* end of normal operation */

#ifdef DEBUG

#define F_CPU 8000000UL		/* Define CPU Frequency e.g. here its 8MHz */
#include <avr/io.h>			/* Include AVR std. library file */
#include <stdio.h>			/* Include std. library file */
#include <util/delay.h>		/* Include Delay header file */

int main(void)
{
	DDRB |= (1<<PB1);	/* Make OC1A pin as output */ 
	TCNT1 = 0;			/* Set timer1 count zero */
	ICR1 = 2499;		/* Set TOP count for timer1 in ICR1 register */

	/* Set Fast PWM, TOP in ICR1, Clear OC1A on compare match, clk/64 */
	TCCR1A = (1<<WGM11)|(1<<COM1A1);
	TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS10)|(1<<CS11);
	while(1)
	{
		OCR1A = 135;		/* Set servo shaft at -90° position 127 */
		// _delay_ms(1500);
		// OCR1A = 175;	/* Set servo shaft at 0° position */
		// _delay_ms(1500);
		// OCR1A = 300;	/* Set servo at +90° position */
		// _delay_ms(1500);
	}
}

#endif