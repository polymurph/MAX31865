/*
 * max31865_lib_settup.c
 *
 * Created: 22-Dec-18 17:28:51
 * Author : Edwin
 */ 

#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include "max31865.h"
#include "hal_usart.h"
#include "hal_spi.h"
#include "hal_timer0.h"
#include <stdio.h>


void usartSettup()
{
	cli();
	hal_USART_enableModule();
	hal_USART_TxEnable();
	hal_USART_setCharSize(charSize_8BIT);
	hal_USART_setBaudRate(9600);
	sei();
}

void chipSelect_setup()
{
	DDRB |=(1 << PINB0);
	PORTB |= (1 << PINB0);
}

void chipSelect()
{
	PORTB &= ~(1 << PINB0);
}

void chipUnselect()
{
	PORTB |= (1 << PINB0);
}

void spi_settup()
{
	cli();
	hal_SPI_enableModule();
	hal_SPI_dissableInterrupt();
	hal_SPI_setMode(MASTER);
	hal_SPI_setClockRate(clk_CLK_DIV_128);
	hal_SPI_dataDirectionOrder(MSB_FIRST);
	hal_SPI_setClockPolarity(NONINVERTED);
	hal_SPI_setClockPhase(SAMPLE_ON_FALLING_EDGE);
	hal_SPI_enable_TRX();
	sei();
}

void delay_chargetime()
{
	_delay_ms(RTD_CAPACITOR_CHARGETIME_ms);
}

void delay_conversiontime()
{
	_delay_ms(63);
}

void timer0_CTC_PWM()
{	
	hal_timer0_enableModule();
	// this code sets up counter0 for an 8kHz Fast PWM wave @ 16Mhz Clock
	// based on https://sites.google.com/site/qeewiki/books/avr-guide/pwm-on-the-atmega328
	
	// PD6 is now output A
	//DDRD |= (1 << DDD6);
	
	// PD5 is now output B
	//DDRD |= (1 << DDD5);

	/*
		OCR0A is the leading register. if the counter reaches the OCR0A value the timer is reset.
		OCR0B just determines the counter compare value for output B. A match will not reset the timer.
		If the OCR0B value is higher than OCR0A the output B will will never be toggled hence the timer is reset 
		after a compare match with the OCR0A
	*/
	//OCR0A = 255;
	//OCR0B = 50;
	hal_timer0_setCompareValueA(100);
	hal_timer0_setCompareValueB(25);

	// set non-inverting mode (can only run like this in CTC mode) for output B 
	//TCCR0A |= (1 << COM0B0);
	hal_timer0_setOutputB(t0_outB_TOGGLE_ON_COMPARE);
	
	// set inverting mode for output A
	//TCCR0A |= (1 << COM0A0);
	hal_timer0_setOutputA(t0_outA_TOGGLE_ON_COMPARE);
	
	//TCCR0A |= (1 << WGM01);
	hal_timer0_setMode(t0_mode_CTC);

	//TCCR0B |= (1 << CS01);
	hal_timer0_setClockSource(t0_clk_CLK_DIV_8);
	// set prescaler to 8 and starts PWM
}

void timer0_Fast_PWM()
{
	hal_timer0_enableModule();
	// this code sets up counter0 for an 8kHz Fast PWM wave @ 16Mhz Clock
	// based on https://sites.google.com/site/qeewiki/books/avr-guide/pwm-on-the-atmega328
	
	// PD6 is now output A
	//DDRD |= (1 << DDD6);
	
	// PD5 is now output B
	//DDRD |= (1 << DDD5);

	// set PWM duty cycle 50% for output A
	//OCR0A = 128;
	hal_timer0_setCompareValueA(128);
	
	// set PWM duty cycle 25% for output B
	//OCR0B = 64;
	hal_timer0_setCompareValueB(64);

	// set none-inverting mode for output B
	//TCCR0A |= (1 << COM0B1);
	hal_timer0_setOutputB(t0_outB_CLEAR_ON_COMPARE);
	
	// set none-inverting mode for output A
	//TCCR0A |= (1 << COM0A1);
	hal_timer0_setOutputA(t0_outA_CLEAR_ON_COMPARE);
	

	//TCCR0A |= (1 << WGM01) | (1 << WGM00);
	hal_timer0_setMode(t0_mode_FAST_PWM_1);
	// set fast PWM Mode
	
	
	// set prescaler to 8 and starts PWM
	
	hal_timer0_setClockSource(t0_clk_CLK_DIV_256);
	
}

void timer0_Normal_PWM()
{
	hal_timer0_enableModule();
	hal_timer0_setCompareValueA(64);
	hal_timer0_setCompareValueB(128);
	hal_timer0_setOutputA(t0_outA_TOGGLE_ON_COMPARE);
	hal_timer0_setOutputB(t0_outB_TOGGLE_ON_COMPARE);
	hal_timer0_setMode(t0_mode_NORMAL);
	hal_timer0_setClockSource(t0_clk_CLK_DIV_8);
}

/*
	prject settup for sprintf with uint16_t values
	https://startingelectronics.org/articles/atmel-AVR-8-bit/print-float-atmel-studio-7/
*/

int main(void)
{
	max31865_t max;
	float rtdbuff = 0;
	uint16_t adcbuff = 0;
	int8_t thfault = 0;
	float tempK = 0;
	float tempC = 0;
	char buff[100];
	
	chipSelect_setup();
	
	// max31865 lib settup
	max31865_register_spi_trx(hal_SPI_trx);
	max31865_register_chargetime_delay(delay_chargetime);
	max31865_register_conversiontime_delay(delay_conversiontime);
	
	
	
	
	// device setup
	max.rtd = 100;
	max.rref = 430;
	max.lowFaultThreshold = 0;
	max.highFaultThreshold = 0xFFFF;
	// 3-Wire + 50Hz filter
	max.configReg = 0x11;
	max.selectChip = chipSelect;
	max.unselectChip = chipUnselect;
	
	
	usartSettup();
	
	
	//timer0_Normal_PWM();
	timer0_Fast_PWM();
	//timer0_CTC_PWM();
	
	/*
		for putty: 
		https://superuser.com/questions/555554/putty-clear-scrollback-from-commandline
	*/
	
	sprintf(buff,"\033[2J\r");
	hal_USART_puts(buff);
	
	sprintf(buff,"Program: max31865_lib_settup\n\r");
	hal_USART_puts(buff);
	
	//max31865_SPIsetup();
	spi_settup();
	
	max31865_configDevice(max);	
	
	
    while (1) 
    {
		/*
		sprintf(buff,"\n\r-----------------------------------");
		hal_USART_puts(buff);
		*/
		
		
		adcbuff = max31865_readADC(max);
		sprintf(buff,"\n\r$%d;",adcbuff);
		hal_USART_puts(buff);
		
		/*
		
		
		rtdbuff = max31865_readRTD(max);
		sprintf(buff,"\n\rrtd: %f",rtdbuff);
		hal_USART_puts(buff);
		*/
		
		/*
		tempC = max31865_readCelsius(max);
		sprintf(buff,"\n\rCelsius: %f",tempC);
		hal_USART_puts(buff);
		
		
		
		tempK = max31865_readKelvin(max);
		sprintf(buff,"\n\rKelvin: %f",tempK);
		hal_USART_puts(buff);
		
		
		
		tempC = max31865_readCelsius(max);
		sprintf(buff,"\n\r%f",tempC);
		hal_USART_puts(buff);
		
		
		
		thfault = max31865_checkThresholdFault(max);
		sprintf(buff,"\n\rThreshold Fault:%d",thfault);
		hal_USART_puts(buff);
		*/
		
		_delay_ms(500);
    }
}

