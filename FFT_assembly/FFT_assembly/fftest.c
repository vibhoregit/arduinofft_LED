/*------------------------------------------------*/
/* FFTEST : A test program for FFT module         */

//#define	SYSCLK		16000000L
#define F_CPU		16000000L
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "interrupt.h"
#include "delay.h"
#include "iom168pa.h"
//#include "suart.h"		/* Defs for using Software UART module (Debugging via AVRSP-COM) */
#include "ffft.h"		/* Defs for using Fixed-point FFT module */
//#define pcvisual
#define ledvisual




/*------------------------------------------------*/
/* Global variables                               */

//char pool[16];	/* Console input buffer */

int16_t capture[FFT_N];			/* Wave captureing buffer */
complex_t bfly_buff[FFT_N];		/* FFT buffer */
uint16_t spektrum[FFT_N/2];		/* Spectrum output buffer */
uint16_t red_last, green_last, blue_last;
uint16_t red,blue,green;
volatile uint8_t j = 0;


/*------------------------------------------------*/
/* Capture waveform                               */
void init_adc()
{
	DIDR0 |= (1<<ADC0D);									// Disable input buffer
	ADMUX |= (1<<REFS0|1<<REFS1|1<<ADLAR);					// Ref is Internal ref 1.1V and channel is A0, results left adjusted
	ADCSRA |= (1<<ADEN|1<<ADATE|1<<ADIE|1<<ADPS2); // Enable ADC, Auto trigger mode
															// Interrupt enabled, scaler = 16
	ADCSRB |= (1<<ADTS2|1<<ADTS0);		         			// Trigger source timer1 compare match B 
}

void init_timer1() // This triggers ADC
{
	TCCR1B |= (1<<WGM12|1<<CS11); // Top = OCCR1A, Clock = CLKio/8 = 2MHz => Fmin = 30.52 Hz
	OCR1AH = 0x00;          
	OCR1AL = 100;                // Hopefully compare match occurs at 9 kHz approximately sets ADC frequency
	OCR1BH = 0x00;                // 222 = 9 khz
	OCR1BL = 0x00;				  // This compare match triggers the ADC
								  // Interrupts disabled Peace! :)
}

/*void capture_wave (int16_t *buffer)
{
	uint8_t count = 0;
	//UDR0 = 255;
	//_delay_ms(2);
	while(count < FFT_N)
	{
		//UDR0 = count;
	    //_delay_ms(1);
		ADCSRA |= 1<<ADSC;			  // start conversion
		while(ADCSRA&(1<<ADSC));	  // Wait for it to finish
		buffer[count] = ADCH - 127;   // Store it in the buffer
		count++;			          // Next count
	}
}
*/

/* This is an alternative function of capture_wave() and can omit captureing buffer.

void capture_wave_inplace (complex_t *buffer, uint16_t count)
{
	const prog_int16_t *window = tbl_window;
	int16_t v;

	ADMUX = _BV(REFS0)|_BV(ADLAR)|_BV(MUX2)|_BV(MUX1)|_BV(MUX0);	// channel

	do {
		ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADFR)|_BV(ADIF)|_BV(ADPS2)|_BV(ADPS1);
		while(bit_is_clear(ADCSRA, ADIF));
		v = fmuls_f(ADC - 32768, pgm_read_word_near(window));
		buffer->r = v;
		buffer->i = v;
		buffer++; window++;
	} while(--count);

	ADCSRA = 0;
}
*/
void init_timer0() // This is used for PWM
{
	DDD |= (1<<DDD5|1<<DDD6); // Set pin 5,6 port D to output
	TCCR0A |= (1<<COM0A1|1<<COM0B1|1<<WGM01|1<<WGM00); // Set Timer 0 for non inverting fast PWM
	TCCR0B |= (1<<CS01|1<<CS01); // Tim0 clock = CLKio/256
	OCR0A = 0; // Default PWM close to 50%
	OCR0B = 0; // No interrupts are enabled :) peace!
}


void init_timer2() // Again used for PWM
{
	DDD |= (1<<DDD3); // Set arduino pin 3 as output for PWM using OC2B
	TCCR2A |= (1<<COM0B1|1<<WGM21|1<<WGM20);  // Set Timer 2 for non inverting fast PWM on OC1B only
	TCCR2B |= (1<<CS22); // Set timer 2 clock to be CLKio/64
	OCR2B = 0;         // No interrupts  Peace :) and default duty cycle = 50% ish
	
}

void init_uart()
{
	
  MCUCR |= 0b00010000;    // Disabling pull up on all ports
  UCSR0B = 0x00;          // disable while setting baud rate
  UCSR0A = 0x00;
  UCSR0C = 0x06;
  UBRR0L = 0x08;          // set baud rate lo (currently 115200) (Refer datasheet)
  UBRR0H = 0x00;          // set baud rate hi
  UCSR0B = 0x08;          // Tx/Rx complete interrupt disabled, Buffer empty interrupt disable
}


void PWM_5(uint16_t pwm)
{
	if(pwm > 200)
		pwm = 255;
	if(pwm > 8)
		{
			DDD |= (1<<DDD5);
			OCR0A = pwm;
		}			
	else
		DDD &= ~(1<<DDD5);
} 
void PWM_6(uint16_t pwm)
{
	//if(pwm > 200)
		//pwm = 255;
	if(pwm > 8)
		{
			DDD |= (1<<DDD6);
			OCR0B = pwm;
		}			
	else
		DDD &= ~(1<<DDD6);
}
void PWM_3(uint16_t pwm)
{
	if(pwm > 200)
		pwm = 255;
	if(pwm > 8)
		{
			DDD |= (1<<DDD3);
			OCR2B = pwm;
		}			
	else
		DDD &= ~(1<<DDD3);
}

int main (void)
{
	uint8_t i = 0;
	cli();
	init_adc();
	init_uart();
	DDRB |= 1<<DDD5;    // For debugging
	init_timer1();
	#ifdef ledvisual
	init_timer0();
	init_timer2();
	#endif
	sei();
				while(1){
				while(j<127);          // Populate the capture buffer			
				fft_input(capture, bfly_buff);     // Send capture buffer to FFT buffer
				j = 0;
				ADCSRA |= (1<<ADEN|1<<ADATE|1<<ADSC);
				fft_execute(bfly_buff);            // Compute FFT
				fft_output(bfly_buff, spektrum);   // Get the magnitude spectrum
#ifdef pcvisual
				for(i = 0; i< FFT_N/2;i++)         // Print N/2 components of the magnitude spectrum to UART
					{
						UDR0 = (spektrum[i]);
						while(!(UCSR0A&(1<<UDRE0)));
					}
					UDR0 = 255;
					//PORTB ^=0x20;
			                         // Stop byte for synchronization
								
#endif

#ifdef ledvisual
				red = 0;
				green = 0;
				blue = 0;
				for(i = 1; i <4; i++)
					if(spektrum[i]>8)
						red += (uint8_t)spektrum[i];
				for(i = 5; i <15; i++)
					if(spektrum[i]>5)
						green += (uint8_t)spektrum[i]<<1;
			    for(i = 16; i <64; i++)
					if(spektrum[i]>5)
						blue += (uint8_t)spektrum[i]<<1;
				
				red_last = (red_last + red)>>1;
				green_last = (green_last+green)>>1;
				blue_last = (blue_last+blue)>>1;
				/*UDR0 =red_last;
						while(!(UCSR0A&(1<<UDRE0)));
				UDR0 =green_last;
						while(!(UCSR0A&(1<<UDRE0)));
				UDR0 = blue_last;
						while(!(UCSR0A&(1<<UDRE0)));
				UDR0 = 255;
						while(!(UCSR0A&(1<<UDRE0)));*/	
				PWM_3(red_last);
				PWM_5(green_last);
				PWM_6(blue_last);	

#endif		
				
}


}

ISR(ADC_vect)
{ 
    if(j < 128)
    {
      capture[j] =ADCH-127;
	  j++;
    }
  else 
    ADCSRA &= ~(1<<ADEN); // Buffer full stop adc auto trigger
  PORTB ^=0x20;
  TIFR1 |= (~(1<<OCF1B|1<<OCF1A)); // Clear interrupt flag for compare match B timer 1 
}