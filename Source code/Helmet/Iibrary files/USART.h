#ifndef USART
#define USART

#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 8000000UL
#define BAUD 9600
#define MYUBRR  F_CPU/16UL/BAUD-1


void usart_init();
void usart_string_transmit(char *string);
void usart_data_transmit(unsigned char data );
unsigned char usart_data_receive( void );


void usart_init()
{	
	UCSRB|= (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);       // Enable USART transmitter and receiver 
	UCSRC |=(1 << URSEL)|(1<<UCSZ1)|(1<<UCSZ0);   // 8 bit data      or UCSRC |= (1 << URSEL)| (1 << UCSZ0) | (1 << UCSZ1);	 Write USCRC for 8 bit data and 1 stop bit    
	UBRRH = 0;			                         // Load UBRRH with upper 8 bit of prescale value 
	UBRRL =MYUBRR;                               // Load UBRRL with lower 8 bit of prescale value 	
}

void usart_data_transmit(unsigned char data )
{
	while ( !( UCSRA & (1<<UDRE)))       // Wait until data transmit and buffer get empty 
	;
	UDR = data;                           //Write data to be transmitting in UDR 
	
}

void usart_string_transmit(char *string)
{
	while(*string)
	{
		usart_data_transmit(*string++);
	}
}

unsigned char usart_data_receive( void )
{
	while ( !(UCSRA & (1<<RXC)) )        // Wait until new data receive 
	;
	return UDR;                          // Get and return received data 
}





#endif