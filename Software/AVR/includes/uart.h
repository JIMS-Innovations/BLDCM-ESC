/**
 * @file uart.h
 * @author Jesutofunmi Kupoluyi
 * @brief  UART driver header file
 * @version 0.1
 * @date 2024-01-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

/* Include guard */
#ifndef __UART_H__
#define __UART_H__

/* Include main definition header */
#include "main.h"

/* Function prototypes */
void UART_init();
unsigned char UART_RxChar();
void UART_TxChar(char);
void UART_SendString(char *str);

#endif/*__UART_H__*/