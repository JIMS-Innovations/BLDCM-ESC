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
#include "uart.h"

/* Variables */
extern volatile uint8_t sequence = 0; 
volatile int RX_buf_idx = 0;
uint8_t motor_speed;
char display[16];

/* Receiver Buffer */
uint8_t RX_buffer[32] = {0};

/* ISR function for Analog Comparator */
ISR(ANALOG_COMP_vect)
{
    for(int i = 0; i < 10; i++) {          
    if(sequence & 1)             
    {
      if(!(ACSR & _BV(ACO))) 
        i -= 1; 
    }
    else                              
    {
      if((ACSR & _BV(ACO)))  
        i -= 1; 
    }
  }
    commutate();
    sequence++;
    sequence %= 6;
    LED_PORT ^= _BV(WORKING_LED);
}

/* ISR function for serial RX */
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

  /* Set global interrupt */
  sei();

  /* UART Intro */
  UART_SendString("BLDC ESC Open loop start up test\n\r");
  
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
  UART_SendString("BLDC ESC Start up test\n\r");

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

    }

    return 0;
}
