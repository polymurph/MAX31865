/*
 * test_3.c
 *
 * Created: 16-Dec-18 10:26:06
 *  Author: Edwin
 */ 
#include "settup.c"
#ifdef _test_3

#define F_CPU 16000000UL
#include <stdint.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <stdio.h>

#include "HAL/hal_spi.h"
#include "HAL/hal_usart.h"
#include "max31865.h"


void usartSettup()
{
	cli();
	hal_USART_enableModule();
	//hal_USART_RxEnable();
	hal_USART_TxEnable();
	
	hal_USART_setCharSize(charSize_8BIT);
	hal_USART_setBaudRate(9600);
	//hal_USART_registerRxCallback(rxCallback);
	
	// enable global interrupts
	sei();
}

void max_settup()
{
	// SPI settup
	hal_SPI_enableModule();
	hal_SPI_dissableInterrupt();
	hal_SPI_setMode(MASTER);
	hal_SPI_setClockRate(clk_CLK_DIV_128);
	hal_SPI_dataDirectionOrder(MSB_FIRST);
	hal_SPI_setClockPolarity(NONINVERTED);
	hal_SPI_setClockPhase(SAMPLE_ON_FALLING_EDGE);
	hal_SPI_enable_TRX();
}


void writeConfigReg(uint8_t data)
{
	PORTB &= ~(1 << PINB1);
	hal_SPI_trx(max31865_reg_CONFIGURATION_W);
	hal_SPI_trx(data);
	// chip unselect
	PORTB |= (1 << PINB1);
}

uint8_t readConfigReg()
{
	uint8_t buff;
	PORTB &= ~(1 << PINB1);
	//read config reg
	hal_SPI_trx(0);
	buff = hal_SPI_trx(0xFF);
	// chip unselect
	PORTB |= (1 << PINB1);
	return buff;
}

uint16_t readRTD()
{
	uint16_t buff;
	PORTB &= ~(1 << PINB1);
	//read config reg
	hal_SPI_trx(0x01);
	buff = hal_SPI_trx(0xAA) << 8;
	buff |= hal_SPI_trx(0xAA);
	
	// chip unselect
	PORTB |= (1 << PINB1);
	return buff;
	
}



int main()
{
	uint16_t buff = 0;
	uint8_t cnt = 0;
	char header[] = "\n\rMAX31865 test program\n\r";
	char stringBuff[20];
	
	DDRB |=(1 << PINB1);
	PORTB |= (1 << PINB1);
	
	usartSettup();
	hal_USART_puts(header);
	
	
	
	max_settup();
	
	while(1)
	{
		writeConfigReg(0xB2);
		
		_delay_ms(10);
		
		buff = readRTD();
		
		//hal_SPI_trx((uint8_t)(buff & 0x00FF));
		//hal_SPI_trx((uint8_t)(buff >> 8));
		
		sprintf(stringBuff,"\n\rRTD value #%3d: %u",cnt++,buff);
		hal_USART_puts(stringBuff);
		
		
		
		
		//readConfigReg();
		
		
		
		_delay_ms(500);
		
	}
}
#endif // _test_3