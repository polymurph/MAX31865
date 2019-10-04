/*
 * test_1.c
 *
 * Created: 30-Nov-18 15:31:09
 *  Author: Edwin
 */ 

#include "settup.c"
#ifdef _test_1

#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/io.h>
//#include "max31865.h"
#include "HAL/hal_usart.h"
#include "HAL/hal_spi.h"
#include <stdio.h>

// temperature curve polynom approximation coefficients
static const float a1 = 2.55865721669;
static const float a2 = 0.000967360412;
static const float a3 = 0.000000731467;
static const float a4 = 0.000000000691;
static const float a5 = 7.31888555389e-13;


uint8_t configRegbuff = 0;


float rtd_to_celsius(uint16_t x)
{
	float h = (float)(100 - (x / 32768 * 400));
	float h2 = h*h;
	float h3 = h2*h;
	float h4 = h2*h2;
	float h5 = h4*h;
	return ((a1*h+a2*h2+a3*h3+a4*h4+a5*h5)-237.15);
}

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

//////////////////////////////////////////////////////////////////////////
void chipSelect()
{
	PORTB &= ~0x02;
	
}
void chipUnselect()
{
	PORTB |= 0x02;
}

void cipSelectSettup()
{
	DDRB |= 0x02;
	PORTB |= 0x02;
}

//////////////////////////////////////////////////////////////////////////

uint8_t read_1Byte(uint8_t address)
{
	uint8_t buff = 0;
	address &= ~0x80;
	
	chipSelect();
	
	hal_SPI_trx(address);
	buff = hal_SPI_trx(0);
	hal_SPI_trx(0);
	
	chipUnselect();
}

void write_1Byte(uint8_t address, uint8_t data)
{
	chipSelect();
	hal_SPI_trx(address | 0x80);
	hal_SPI_trx(data);
	chipUnselect();
}

void turnOnVBias()
{
	configRegbuff |= 0x80;
	write_1Byte(0x80,configRegbuff);
}
void turnOffVBias()
{
	configRegbuff &= ~0x80;
	write_1Byte(0x80,configRegbuff);
}

uint16_t readRTD_1Shot()
{
	uint16_t rtdVal = 0;
	chipSelect();
	
	hal_SPI_trx(0x80);
	// v bias + 1 shot + 50Hzs
	hal_SPI_trx(0xA1);
	_delay_ms(60);
	chipUnselect();
	
	_delay_ms(60);
	
	
	chipSelect();
	hal_SPI_trx(0x01);
	rtdVal = (hal_SPI_trx(0x02) << 8);
	rtdVal |= hal_SPI_trx(0);
	chipUnselect();
	
	return rtdVal;
}

//////////////////////////////////////////////////////////////////////////

int main(void)
{
	uint8_t a, b;
	uint16_t c = 0;
	float temp = 0;
	char buff[100];
	
	DDRB |= 0x01;
	PORTB &= ~0x01;
	
	cipSelectSettup();
	usartSettup();
	max_settup();
	
	configRegbuff = read_1Byte(0x00);
	
	turnOnVBias();
	
	
	_delay_ms(10);
	
	while(1)
	{
		
		/*
		a = read_1Byte(0x01);
		b = read_1Byte(0x02);
		sprintf(buff,"\n\rMSB: %d\n\rLSB: %d\n\r",a, b);
		*/
		c = readRTD_1Shot();
		//temp = rtd_to_celsius(c);
		sprintf(buff,"\n\rRTD: %u\n\r°C: %-.2f\n\r",c,temp);
		//sprintf(buff,"\n\rMSB: %d\n\rLSB: %d\n\r",(uint8_t)(c >> 8),(uint8_t)(c));
		
		if(c > 500)
		{
			PORTB |= 0x01;
		}
		else
		{
			PORTB &= ~0x01;
		}
		
		
		hal_USART_puts(buff);
		_delay_ms(100);
	}
}

#endif // _test_1
