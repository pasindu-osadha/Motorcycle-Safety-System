/*PIN WITH MODULES*/
/*
	   PINB0 - Seat pushbutton
	   PINB1 - Stand push button
	   PINB2 - SPEED INDICATOR
	   PINB3 - VEHECLE INDICATOR
	   PINB4 - RED INDICATOR, BUZZER
	   PINB5 - RELAY MODULE
	   PINB6 - GSM GPS RESET
	   
	   PIND0 - GSM/GPS RECIVER  & BLUETOOTH RECIVER              
	   PIND1 - GSM/GPS TRANSMITTER & BLUETOOTH TRANSMITTER 
	   PIND2 - ULTRASONIC ECHO
	   PIND6 - MUX CONTROLLER 
	   PIND7 - ULTRASONIC TRIGGER 
	              
	   
	   PINC0 - LCD SCL
	   PINC1 - LCD SDA
	   
	   PINA0 - ACCELEROMETER X AXIS
	   PINA1 - ACCELEROMETER Y AXIS
	   PINA2 - ACCELEROMETER Z AXIS
	   PINA7 - RAIN SENSOR  
	    
	*/

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include "i2c_lcd.c"
#include "i2cmaster.c"
#include "USART.h"

/*Indicators defines*/
#define SPEED_ON PORTB |= (1 << PINB2)
#define SPEED_OFF PORTB &= ~(1 << PINB2)
#define VEHICLE_ON PORTB |= (1 << PINB3)
#define VEHICLE_OFF PORTB &= ~(1 << PINB3)
#define RED_ON PORTB |= (1 << PINB4)
#define RED_OFF PORTB &= ~(1 << PINB4)
#define RELAY_ON PORTB &= ~(1 << PINB5)
#define RELAY_OFF PORTB |= (1 << PINB5)
#define GSM_RESET_ON  PORTB |= (1 << PINB6)
#define GSM_RESET_OFF  PORTB &= ~(1 << PINB6)

/*Multiplexer control*/
#define BLUETOOTH_ON   PORTD &=~(1 << PIND6)
#define GSM_GPS_ON     PORTD |=(1 << PIND6)

#define Buffer_size  380

/*GSM COMMANDS, Functions & variables*/
char At[] = {"AT\n"};						// AT to check GSM  -- or "AT\n"
char Meg_Mode[] = {"AT+CMGF=1\n"};			// Active message mode
char Meg_cmd1[] = {"AT+CMGS="};				// message command
char Re_num1[] = {"\"+94702019316\""};		// receiver number 01
char Re_num2[] = {"\"+94716342666\""};		// receiver number 02
char Gps_on[] = {"AT+CGNSPWR=1\n"};			// GPS power on
char Get_Gps_location[] = {"AT+CGNSINF\n"}; // get location CGNSINF
char Dis_Call[] = {"ATH\n"};				// Disconnect the call

void get_location_and_store();
void Send_meg_with_location(char *receiver_no,char *emergency);

volatile char gps_data_recive[160], Gmap_link[] = {"http://maps.google.com/maps?q=loc:"};
char temp[30], temp1[20], date_time[20], latitude[15], longitude[15], altitude[10], Speed_Over_ground[8];
volatile char buffer[Buffer_size];
volatile int buffer_index = 0, call_flag = 0;
int speed,speed_inde_flag, gsm_reset_flag = 0;

/*Accelerometer functions & variables declaration*/

void adc_init(void);
int read_adc_channel(unsigned char channel);
void accelerometer_start_value(void);
void accelerometer_read_values(void);
 void Identify_accident (void);

volatile int x_axis, y_axis, z_axis;
volatile int x_intial, y_intial, z_intial;
volatile int x_final, y_final, z_final;
const int accident_value = 70;

/*ultrasonic functions and variables */

static volatile int pulse = 0; // integer to access all though the program
static volatile int ultra_i = 0;
int distance, distance_limit = 200, vehical_dete_flag;

// Rain sensor variables
int rain_value, rain_limit = 500;

int stand_flag, bluetooth_signal;

int main(void)
{
	/*PN DECLARATION */
	DDRA = 0b01111000;
	DDRB = 0b11111100;
	DDRC = 0b11111100;
	DDRD = 0b11000000;

	/*Initialization */
	adc_init();
	usart_init();


	sei(); // Global interrupt enable
	
	// check GSM & GPS available or not
	GSM_GPS_ON;
	memset(buffer,0,Buffer_size);
	buffer_index = 0;
	
	while (gsm_reset_flag<1)
	{
	
	usart_string_transmit(At);
	_delay_ms(2000);
	if (strstr(buffer,"OK"))
	{
		GSM_RESET_OFF;
		gsm_reset_flag =1;	
	}
	else
	{
		usart_string_transmit(At);
		GSM_RESET_ON;
		_delay_ms(3000);
		usart_string_transmit(At);
		GSM_RESET_OFF;
		_delay_ms(2000);
		usart_string_transmit(At);
		gsm_reset_flag =0;
	}
	}
	
	
	usart_string_transmit(Gps_on); // power up the GPS module and wait 2 sec
	_delay_ms(2000);

	PORTB |= (1 << PINB0); // seat push button high reading
	PORTB |= (1 << PINB1); // stand push button high reading

	RELAY_ON;
	lcd_init(LCD_BACKLIGHT_ON);
	lcd_clear();
	lcd_write_a_sample_string(0, 0, "-WELCOME KOSMO-");
	_delay_ms(2000);
	
	accelerometer_start_value(); // this function is use for identify the stating values in accelerometer
	
	lcd_clear();
	lcd_write_a_sample_string(0,0," checking Stand ");
	_delay_ms(1000);
	while (1)
	{

		
		stand_flag = 0;
		bluetooth_signal = 0;
		
		GSM_GPS_ON;	
		while (stand_flag < 1)
		{
			

			if (bit_is_clear(PINB, 0) && bit_is_clear(PINB, 1)) //  stand is released when the seat push button is pressed
			{
				
				stand_flag = 1; // stand release (good condition)
				RED_OFF;
				RELAY_OFF;
			}

			if (bit_is_clear(PINB, 0) &&  (bit_is_clear(PINB, 1)==0))
			{
				stand_flag = 0; // stand not release (bad condition)
				
				lcd_clear();
				lcd_write_two_sample_strings(0, 0, "PLEASE RELEASE ", 0, 1, "YOUR STAND");
				_delay_ms(100);
				RED_ON;
				_delay_ms(5000);                             //  5 sec delay
				RELAY_ON;
				
			}
			if (((bit_is_clear(PINB, 0)==0) &&  (bit_is_clear(PINB, 1)==0)) ) // stand not release but vehicle in stop condition and also identify  stand based accident  
			{
				
				//memset(buffer, 0, 380);
				//lcd_init(LCD_BACKLIGHT_OFF);
				lcd_clear();
				RELAY_ON;
				RED_OFF;
				if (call_flag == 1)
				{
					get_location_and_store();
					Send_meg_with_location(Re_num1,"Found your vehicle ");
					Send_meg_with_location(Re_num2,"Found your vehicle ");
					call_flag = 0;
				}
				
					Identify_accident();
			}
			
			
			
		}
		lcd_clear();
		lcd_write_a_sample_string(0, 0, " Checking helmet ");
		//check helmet is worn and  alcohol
		BLUETOOTH_ON;			// active multiplexer to bluetooth side
		memset(buffer, 0, Buffer_size); // clear previous memory
		buffer_index =0;
		
		while (bluetooth_signal < 3)
		{

			//(ALCOHOL DITECTED & WORN HELMET)            ALCOHOL DETECTED
			if (strstr(buffer, "A"))
			{

				bluetooth_signal = 1;
				
			}

			//(ALCOHOL DO NOT DETECTED & Is not WORN HELMET)   helmet is removed
			if ( strstr(buffer, "H"))
			{
				bluetooth_signal = 2;
				
			}

			//(ALCOHOL DO NOT DETECTED & WORN HELMET)     SAFE CONDITION
			if (strstr(buffer, "S"))
			{
				bluetooth_signal = 3;
				
			}
			
			switch(bluetooth_signal)
			{
				case  1 :	
							RED_ON;
							lcd_clear();
							lcd_write_a_sample_string(0, 0, "ALCOHOL DETECTED");
							_delay_ms(5000);
							RELAY_ON;
							break;
							
				case  2 :   
						    RED_ON;
							lcd_clear();
							lcd_write_two_sample_strings(0, 0, "PLEASE WEAR", 0, 1, "YOUR HELMET");
							_delay_ms(5000);
							RELAY_ON;
							break;
			
				case  3 :	
							RED_OFF;
							RELAY_OFF;
							break;
				
				default:    bluetooth_signal = 0;		
			}
		}
			
		GSM_GPS_ON;
		memset(buffer, 0, Buffer_size);
		buffer_index = 0;

		//right side closed vehicle detection
		
		GICR |= 1 << INT0;   // enabling interrupt 0 - ultrasonic (Global Interupt Control Registor)  GICR = 0b01000000;
		MCUCR |= 1 << ISC00; // setting interrupt triggering logic change
		
		distance = 0;
		vehical_dete_flag = 0;
		
	while(vehical_dete_flag < 1)
	{
			
		
		for (int u = 0; u < 5; u++)
		{
			PORTD |= 1 << PIND7; // Trigger PIN D7
			_delay_us(15);		 // triggering time

			PORTD &= ~(1 << PIND7);   // trigger pin off
			distance += (pulse / 58); // calculate the distance in CM
			// if use fcpu 8 000 000 --> count_a = (pulse/58)/8     
		}
		
	    distance /= 5;

		if (distance < distance_limit && distance > 0)
		{
			vehical_dete_flag = 0;
			VEHICLE_ON;
			lcd_clear();
			lcd_write_a_sample_string(0, 0, "   Check your   ");
			lcd_write_a_sample_string(0, 1, "  Side mirror  ");
			_delay_ms(500);
		}
		else
		{
			vehical_dete_flag = 1;
			VEHICLE_OFF;
			
		}
		
		lcd_clear();
		lcd_write_number(0,0,distance,10);
		_delay_ms(500);
		
	}
	
	


		// get Speed 70 KMPH and 40 KMPH in rainy condition
		speed_inde_flag = 0;
while (speed_inde_flag < 1)
{

		for (int r = 0; r < 5; r++) // get rain information using rain sensor
		{
			rain_value += read_adc_channel(7);
		}
		rain_value /= 5;

		get_location_and_store();			// this function is used for identify the speed
		if (rain_value < 600 && speed > 40) // 40 KMPH during the rainy condition
		{
		
			speed_inde_flag = 0;
		}
		else if (rain_value > 600 && speed > 70) // 70 KMPH during the normal condition
		{
			
			speed_inde_flag = 0;
		}
		else
		{
		
			speed_inde_flag =1;
		}
		
		switch(speed_inde_flag)
		{
			case 0 :SPEED_ON;
					lcd_clear();
					lcd_write_a_sample_string(0,0,"HIGHSPEED DETECT");
					lcd_write_a_sample_string(0,1,"Speed : ");
					lcd_write_number(8,1,speed,10);
					_delay_ms(500);
					break;
					
			case 1 :SPEED_OFF;
					break;
					
		}
		
	
		
}

// use accelerometer to identify the accident and angle 

		Identify_accident ();
		lcd_clear();
		lcd_write_a_sample_string(0, 0, "X:");
		lcd_write_number(3,0,x_final,10);
		lcd_write_a_sample_string(11, 0, "Y:");
		lcd_write_number(13,0,y_final,10);
		lcd_write_a_sample_string(0, 1, "Z:");
		lcd_write_number(3,1,z_final,10);
		_delay_ms(1000);
		
	}
}

/*OTHER FUNCTIONS*/

/* ADC accelerometer functions*/

void adc_init(void)
{
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADPS2);
	// ADEN -- Analog to digital enable 
	//ADSC -- ADC Start conversion in single conversion mode 
	// ADATE -- 
	//ADPS2 -- 16 prescale    
	SFIOR = 0x00;
}

int read_adc_channel(unsigned char channel)
{
	int adc_value;
	unsigned char temp;
	ADMUX = (1 << REFS0) | channel; // helps to select the analog pin 
	_delay_ms(1);
	temp = ADCL;  
	adc_value = ADCH;
	adc_value = (adc_value << 8) | temp;
	return adc_value;
}

void accelerometer_start_value(void)
{
	for (int i = 0; i < 20; i++)
	{
		x_intial += read_adc_channel(0);
		y_intial += read_adc_channel(1);
		z_intial += read_adc_channel(2);
	}
	x_intial /= 20;
	y_intial /= 20;
	z_intial /= 20;
}

void accelerometer_read_values(void)
{
	x_axis = 0;
	y_axis = 0;
	z_axis = 0;
	
	for (int r = 0; r < 10 ;r++)
	{
			x_axis += read_adc_channel(0);
			y_axis += read_adc_channel(1);
			z_axis += read_adc_channel(2);
	}

	x_axis /= 10;
	y_axis /= 10;
	z_axis /= 10;
	
	x_final = x_intial - x_axis;
	y_final = y_intial - y_axis;
	z_final = z_intial - z_axis;
	
	x_final = abs(x_final);
	y_final = abs(y_final);
	z_final = abs(z_final);

}

    void Identify_accident (void)
    {
	    
	    accelerometer_read_values();

	    if (accident_value < x_final || accident_value < y_final || accident_value < z_final)
	    {
		    lcd_clear();
		    lcd_write_a_sample_string(0, 0, " ACCIDENT DETECT ");
		    lcd_write_a_sample_string(0, 1, " SEND MSG OWNER ");
		    get_location_and_store();
		    Send_meg_with_location(Re_num1,"ACCIDENT... Please call to the GSM ");
		    Send_meg_with_location(Re_num2,"ACCIDENT... Please call to the GSM ");
	    }
    }

/*Ultrasonic functions*/

ISR(INT0_vect) //interrupt service routine when there is a change in logic level
{
	if (ultra_i == 1) //when logic from HIGH to LOW
	{
		TCCR1B = 0;	//disabling counter
		pulse = TCNT1; //count memory is updated to integer
		TCNT1 = 0;	 //resetting the counter memory
		ultra_i = 0;
	}

	if (ultra_i == 0) //when logic change from LOW to HIGH

	{
		TCCR1B |= 1 << CS11; //enabling counter  CS11 --> prescaler 8    CS10 --> no prescaler
		ultra_i = 1;
	}
}

/*GSM/GPS functions*/

void get_location_and_store()
{
	memset(buffer, 0, Buffer_size);
	buffer_index = 0;

	usart_string_transmit(Get_Gps_location);
	_delay_ms(400);

	for (int a = 0; a < 150; a++)
	{
		gps_data_recive[a] = buffer[a];
	}

	/*categorize GPS data */

	int c;

	/*Store  */
	for (c = 0; gps_data_recive[c] != ','; c++)
	{
		temp[c] = gps_data_recive[c];
	}

	int index = 0;
	for (c = c + 1; gps_data_recive[c] != ','; c++)
	{
		temp1[index] = gps_data_recive[c];
		index++;
	}

	index = 0;
	for (c = c + 1; gps_data_recive[c] != ','; c++)
	{
		date_time[index] = gps_data_recive[c];
		index++;
	}

	index = 0;
	for (c = c + 1; gps_data_recive[c] != ','; c++)
	{
		latitude[index] = gps_data_recive[c];
		index++;
	}

	index = 0;
	for (c = c + 1; gps_data_recive[c] != ','; c++)
	{
		longitude[index] = gps_data_recive[c];
		index++;
	}

	index = 0;
	for (c = c + 1; gps_data_recive[c] != ','; c++)
	{
		altitude[index] = gps_data_recive[c];
		index++;
	}

	index = 0;
	for (c = c + 1; gps_data_recive[c] != ','; c++)
	{
		Speed_Over_ground[index] = gps_data_recive[c];
		index++;
	}

	speed = atoi(Speed_Over_ground); // convert string to Integer
}

void Send_meg_with_location(char *receiver_no,char *emergency)
{

	usart_string_transmit(At); // check AT
	_delay_ms(10);
	usart_string_transmit(Meg_Mode); // enter mesage mode
	_delay_ms(10);
	usart_string_transmit(Meg_cmd1); //
	_delay_ms(10);
	usart_string_transmit(receiver_no); // receivers number
	usart_data_transmit(13);			//UDR=(13);    //  <CR> character
	_delay_ms(10);
	usart_string_transmit(emergency);
	usart_string_transmit(Gmap_link); // google link
	usart_string_transmit(latitude);
	usart_data_transmit(',');
	usart_string_transmit(longitude);
	usart_data_transmit(26);
	_delay_ms(5000);
	memset(buffer, 0, Buffer_size);
	buffer_index = 0;
}

ISR(USART_RXC_vect)
{
	buffer[buffer_index] = UDR;
	buffer_index++;

	if (strstr(buffer, "RING"))
	{
		usart_string_transmit(Dis_Call);
		call_flag = 1;
		_delay_ms(4000);
		memset(buffer, 0, Buffer_size);
		buffer_index =0;
	}
	
	if (buffer_index >= Buffer_size)
	{
		buffer_index =0;
	}
}
