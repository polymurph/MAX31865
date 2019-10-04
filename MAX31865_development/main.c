/*
 * MAX31865_development.c
 *
 * Created: 13-Nov-18 17:10:41
 * Author : Edwin
 */ 
#include "settup.c"

#ifdef main_file

#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/io.h>
//#include "max31865.h"
#include "HAL/hal_usart.h"
#include "HAL/hal_spi.h"
#include <stdio.h>

uint8_t configReg = 0;


void chipSelect()
{
	PORTB &= ~0x02;
	
}
void chipUnselect()
{
	PORTB |= 0x02;
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

//////////////////////////////////////////////////////////////////////////
void max_settup()
{
	// SPI settup
	hal_SPI_enableModule();
	hal_SPI_dissableInterrupt();
	hal_SPI_setMode(MASTER);
	hal_SPI_setClockRate(clk_CLK_DIV_128);
	hal_SPI_dataDirectionOrder(MSB_FIRST);
	hal_SPI_setClockPolarity(INVERTED);
	hal_SPI_setClockPhase(SAMPLE_ON_FALLING_EDGE);
	hal_SPI_enable_TRX();
}

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

uint16_t readRTDRegs()
{
	uint16_t buff = 0;
	chipSelect();
	
	hal_SPI_trx(0x03);
	buff = hal_SPI_trx(0) << 7;
	_delay_ms(1);
	buff |= hal_SPI_trx(0);
	hal_SPI_trx(0);
	
	chipUnselect();
	return buff;
}


void write_1Byte(uint8_t address, uint8_t data)
{
	chipSelect();
	hal_SPI_trx(address | 0x80);
	hal_SPI_trx(data);
	chipUnselect();
}

void turnOnVBIAS()
{
	configReg |= 0x80;
	write_1Byte(0x00,configReg);
}

void turnOffVBIAS()
{
	configReg &= ~0x80;
	write_1Byte(0x00,configReg);
}

void set3WireMode()
{
	configReg |= 0x10;
	write_1Byte(0x00,configReg);
}

void set2or4WireMode()
{;
	configReg &= ~0x10;
	write_1Byte(0x00,configReg);
}

void clearFault()
{
	configReg |= 0x02;
	write_1Byte(0x00,configReg);
}

uint8_t readFault()
{
	return read_1Byte(0x07) & ~0x03;
	
}

uint16_t oneShotReadRTD()
{
	uint16_t temp = 0;
	uint8_t buff = 0;
	
	//turnOnVBIAS();
	//turnOffVBIAS();
	
	//buff = read_1Byte(0x80);
	
	// set oneshot bit
	buff |= 0x20;
	
	// turn on vbias
	buff |= 0x80;
	
	// erase auto conversion mode
	buff &= ~0x40;
	
	// fault  state clear
	buff |= 0x02;
	
	write_1Byte(0x00,buff);
	
	_delay_ms(40);
	//temp = (uint16_t)(readRTDRegs() / 32768 * 400);
	temp = readRTDRegs();
	turnOffVBIAS();
	
	return temp;
}

//////////////////////////////////////////////////////////////////////////
//#define main_test
#ifdef main_test
int main(void)
{
	float temp = 0;
	uint16_t a = 1050;
	uint8_t b = 0;
	uint8_t cnt = 0;
	char buff[100];
	
	DDRB |= 0x02;
	PORTB |= 0x02;
	
	usartSettup();
	
	max_settup();
	set3WireMode();
	clearFault();
	
	
	
	
    while (1) 
    {
		#ifdef float_print
		// https://startingelectronics.org/articles/atmel-AVR-8-bit/print-float-atmel-studio-7/
		sprintf(buff,"%-.2f\n\r",temp);
		#endif
		//sprintf(buff,"%d\n\r",a);
		
		#ifdef int_print
		sprintf(buff,"%d\n\r",max31865_readRTD(dev));
		
		
		hal_USART_puts(buff);
		_delay_ms(1000);
		#endif
		a = oneShotReadRTD();
		b = readFault();
		sprintf(buff,"Measurement %d: %d\n\rFaultStatus: %d\n\r\n\r",cnt++,a,b);
		hal_USART_puts(buff);
		_delay_ms(1000);
    }
}
#endif



#ifdef bla
int main()
{
	float temp = 0;
	uint16_t a = 1050;
	uint8_t b = 0;
	uint8_t cnt = 0;
	char buff[100];
	
	DDRB |= 0x02;
	PORTB |= 0x02;

	usartSettup();
	
	max_settup();
	
	while (1)
	{
		a = read_1Byte()

		sprintf(buff,"Read Data\n\r",cnt++,a,b);
		hal_USART_puts(buff);
		
	}
}
#endif
#endif