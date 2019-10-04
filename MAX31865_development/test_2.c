/*
 * test_2.c
 *
 * Created: 09-Dec-18 09:37:25
 *  Author: Edwin
 */ 


#include "settup.c"
#ifdef _test_2

#define F_CPU 16000000UL
#include <stdint.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <stdio.h>

#include "HAL/hal_spi.h"
#include "HAL/hal_usart.h"
#include "max31865.h"


//////////////////////////////////////////////////////////////////////////

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
	hal_SPI_setClockPolarity(NONINVERTED);
	hal_SPI_setClockPhase(SAMPLE_ON_FALLING_EDGE);
	hal_SPI_enable_TRX();
}

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

void readNReg(max31865_t device,uint8_t addr, uint8_t *buff, uint8_t n)
{
	uint8_t index = 0;
	hal_SPI_setCLK_LOW();
	device.selectChip();
	
	hal_SPI_trx(addr);
	
	do
	{
		buff[index++] = hal_SPI_trx(0xFF);
	} while (n--);
	
	device.unselectChip();
}


void writeNReg(max31865_t device, uint8_t addr,const uint8_t *buff, uint8_t n)
{
	uint8_t index = 0;
	
	hal_SPI_setCLK_LOW();
	device.selectChip();
	
	hal_SPI_trx(addr);
	
	do
	{
		hal_SPI_trx(buff[index++]);
	} while (n--);
	
	device.unselectChip();
}

uint8_t read_reg(max31865_t dev ,max31865_regAddr_t reg)
{
	uint8_t buff = 0;
	
	readNReg(dev,reg,&buff,1);
	return buff;
}

void write_reg(max31865_t dev, max31865_regAddr_t reg, uint8_t data)
{
	writeNReg(dev,reg,&data,1);
}

//////////////////////////////////////////////////////////////////////////

void max_turnOnVBias(max31865_t dev)
{
	uint8_t buff = read_reg(dev,max31865_reg_CONFIGURATION_R);
	buff |= 0x80;
	write_reg(dev,max31865_reg_CONFIGURATION_W,buff);
}

void max_turnOffVBias(max31865_t dev)
{
	uint8_t buff = read_reg(dev,max31865_reg_CONFIGURATION_R);
	buff &= ~0x80;
	write_reg(dev,max31865_reg_CONFIGURATION_W,buff);
}

void max_turnOnAutoConv(max31865_t dev)
{
	uint8_t buff = read_reg(dev,max31865_reg_CONFIGURATION_R);
	buff |= 0x40;
	write_reg(dev,max31865_reg_CONFIGURATION_W,buff);
}

void max_turnOffAutoConv(max31865_t dev)
{
	uint8_t buff = read_reg(dev,max31865_reg_CONFIGURATION_R);
	buff &= ~0x40;
	write_reg(dev,max31865_reg_CONFIGURATION_W,buff);
}

void max_1Shot(max31865_t dev)
{
	uint8_t buff = read_reg(dev,max31865_reg_CONFIGURATION_R);
	buff |= 0x20;
	write_reg(dev,max31865_reg_CONFIGURATION_W,buff);
}

void max_init(max31865_t device)
{
	// optimize this 
	max_turnOffVBias(device);
	max_turnOffAutoConv(device);
	
}

//////////////////////////////////////////////////////////////////////////

int main()
{
	uint16_t temp = 0;
	char buff[100];
	max31865_t A;
	
	A.selectChip = chipSelect;
	A.unselectChip = chipUnselect;
	cipSelectSettup();
	usartSettup();
	max_settup();
	_delay_ms(5000);
	
		
	while(1)
	{
		max_turnOnVBias(A);
		_delay_ms(100);
		max_1Shot(A);
		_delay_ms(100);
		
		temp = (uint16_t)(read_reg(A,max31865_reg_RTD_MSB_R) << 8);
		temp |= read_reg(A,max31865_reg_RTD_LSB_R);
		
		max_turnOffVBias(A);
		
		sprintf(buff,"\n\rRTD: %u",temp);
		hal_USART_puts(buff);
		_delay_ms(100);
	}
}

#endif //_test_2