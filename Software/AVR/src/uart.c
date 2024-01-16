/**
 * @file uart.h
 * @author Jesutofunmi Kupoluyi
 * @brief  UART driver source file
 * @version 0.1
 * @date 2024-01-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

/* Including header file */
#include "uart.h"

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
