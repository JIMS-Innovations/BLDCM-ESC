
/* Include header file */
#include "commutation.h"

extern volatile uint8_t sequence; 

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
