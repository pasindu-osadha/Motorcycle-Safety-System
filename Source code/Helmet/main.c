/*
 * helmert side.c
 *
 * Created: 12/14/2019 12:20:49 PM
 * Author : PasinduOsadha
 */ 

#define F_CPU 8000000UL



#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include "i2cmaster.c"
#include "i2c_lcd.c"
#include "USART.h"


void adc_init(void);
int read_adc_channel(unsigned char channel);

int limit = 350;   // alcohol limit 
int push1,value,sample;
char str[5],samp[8];

int main(void)
{
	/*PIN SETUP */
	
	/*
	
	PINA0 - Alcohol sensor
	PINB0 - Helmet push button
	
	*/
	
	
	/*PIN declaration*/
	DDRA = 0b00000000;   // PINA0 is INPUT for Alcohol
	DDRB = 0b11111110;   // PINB0 is push button

	adc_init();  
	usart_init();
	
	
    /* Replace with your application code */
    while (1) 
    {
			//PORTB =0b00000001;
			PORTB |= 1 << PINB0;  // set PINB0 to high reading
		
	
		if(bit_is_clear(PINB,0))
		{
			push1 = 1;			
		}
		else
		{
			push1 = 0;			
		}
	
		sample = 0;
		value = 0;
		
		 for (int i=0;i<10;i++)
		 {
			 sample += read_adc_channel(0);   // get read from A0
		 }
		  
		  value = sample/10;
		  itoa(value,str,10);            //convert value to string
		  itoa(sample,samp,10);            //convert value to string
		
	
		if (value>limit && push1==1 )
		{
			usart_string_transmit("A");          // alcohol detected and pushbutton pressed
			_delay_ms(300);
		}
		
		
		 if (value<limit && push1== 0)
		{
			usart_string_transmit("H");		// alcohol not detected push button is not pressed
			_delay_ms(300);							// helmet is not worn 
		}
		
		if (value<limit && push1==1 )
		{
			usart_string_transmit("S");       // normal safe condition
			_delay_ms(300);
		}	
		
    }
}






void adc_init(void)
{
	ADCSRA=(1<<ADEN)|(1<<ADSC)|(1<<ADATE)|(1<<ADPS2);
	SFIOR=0x00;
}

int read_adc_channel(unsigned char channel)
{
	int adc_value;
	unsigned char temp;
	ADMUX=(1<<REFS0)|channel;
	_delay_ms(1);
	temp=ADCL;
	adc_value=ADCH;
	adc_value=(adc_value<<8)|temp;
	return adc_value;
}
