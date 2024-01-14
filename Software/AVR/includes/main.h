/**
 * @file main.h
 * @author Jesutofunmi Kupoluyi (jimsufficiency@gmail.com)
 * @brief This is firmware header for my final year project - BLDC ESC
 * @version 0.1
 * @date 2023-10-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef     __MAIN_H__
#define     __MAIN_H__

/* Definitions */
#define F_CPU 8000000UL /* CPU Clock frequency */
#define __AVR_ATmega328__
#define __DELAY_BACKWARD_COMPATIBLE__

/* Include required libraries */
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* Configuration definitions */
#define MAX_SPEED 255
#define MIN_SPEED 35
#define START_SPEED 75

/* Motor Phase Commutation sequence
    Step - 1 2 3 4 5 6
    High - A A B B C C 
    Low  - B C C A A B 
 */

/*** Bridge Pin definitions ***/
/* High Side */

#define A_H     PB1
#define B_H     PB2
#define C_H     PB3 /* * */
#define DRV_HS_DDR  DDRB
#define DRV_HS_PORT PORTB

/* Low Side */

#define A_L     PD4 /* * */
#define B_L     PD3
#define C_L     PD2 /* * */
#define DRV_LS_DDR  DDRD
#define DRV_LS_PORT PORTD

/***-----------------------***/

/* BEMF Sense Pins */
#define PH_A_BEMF   PC0
#define PH_B_BEMF   PC1
#define PH_C_BEMF   PC2
#define V_GND       PD6
#define BEMF_DDR    DDRC
#define BEMF_PORT   PORTC

/* PWM Input pin */
#define PWM_IN      PB0
#define PWN_DDR     DDRB
#define PWM_PORT    PORTB

/* LED Indicators */
#define WORKING_LED PB4
#define ERROR_LED   PB5
#define LED_DDR     DDRB
#define LED_PORT    PORTB

/* Mode Selection */
#define PWM_SEL     PB6
#define DIR_SEL     PB7
#define SEL_DDR     DDRB
#define SEL_PORT    PORTB

/* USART Port */
#define UART_RX     PD0
#define UART_TX     PD1

/* Speed Control Buttons */
#define INC_BTN     PD5
#define DEC_BTN     PD7
#define BTN_DDR     DDRD
#define BTN_PORT    PORTD
#define BTN_PIN     PIND

/* UART defines */
#define USART_BAUDRATE 9600 /* Set baud rate here */
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)	

/* Function protypes */
void commutate();
void inv_commutate();
void stop();
void set_speed(uint8_t);

/* Function prototypes */
void UART_init();
unsigned char UART_RxChar();
void UART_TxChar(char);
void UART_SendString(char *str);

void AH_BL();
void AH_CL();
void BH_CL();
void BH_AL();
void CH_AL();
void CH_BL();

void BEMF_A_FALLING();
void BEMF_A_RISING();
void BEMF_B_FALLING();
void BEMF_B_RISING();
void BEMF_C_FALLING();
void BEMF_C_RISING();

#endif    /*__MAIN_H__*/