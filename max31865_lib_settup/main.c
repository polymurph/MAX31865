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

