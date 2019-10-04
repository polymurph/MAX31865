/*
 * test_3_imp.c
 *
 * Created: 16-Dec-18 11:29:28
 *  Author: Edwin
 */ 

#include "settup.c"
#ifdef _test_3_imp


#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define BLU   "\x1B[34m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"
#define WHT   "\x1B[37m"
#define RESET "\x1B[0m"


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

void chipSelect()
{
	PORTB &= ~(1 << PINB1);
}

void chipUnselect()
{
	PORTB |= (1 << PINB1);
}

//////////////////////////////////////////////////////////////////////////
#if 0
void readNReg(max31865_t device,uint8_t addr, uint8_t *buff, uint8_t n)
{
	uint8_t index = 0;
	
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
	
	device.selectChip();
	
	hal_SPI_trx(addr);
	
	do
	{
		hal_SPI_trx(buff[index++]);
	} while (n--);
	
	device.unselectChip();
}

void writeReg(max31865_t device, uint8_t addr,uint8_t data)
{
	writeNReg(device,addr,&data,1);
}
#endif

uint16_t readADC(max31865_t device)
{
	uint8_t buff[2];
	uint16_t ADC_val = 0;
	device.selectChip();
	
	readNReg(device,0x01,buff,2);
	// chip unselect
	device.unselectChip();
	ADC_val = (uint16_t)((buff[0]<<8) | buff[1]);
	ADC_val >>= 1;
	
	return ADC_val;
}

uint16_t readRTD(max31865_t device)
{
	uint8_t buff[2];
	uint16_t ADC_val = 0;
	device.selectChip();
	
	readNReg(device,0x01,buff,2);
	// chip unselect
	device.unselectChip();
	ADC_val = (uint16_t)((buff[1]<<8) | buff[0]);
	
	return (uint16_t)(((uint32_t)(ADC_val) * (uint32_t)(device.Rref))/(uint32_t)(32768));
	
}

//////////////////////////////////////////////////////////////////////////

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

/*

Terminal color https://stackoverflow.com/questions/3585846/color-text-in-terminal-applications-in-unix

*/
int main()
{
	uint16_t buff = 0;
	uint16_t RTDres = 0;
	float frtd = 0;
	uint8_t regBuff[2];
	uint8_t cnt = 0;
	char header[] = "\n\rMAX31865 test program\n\rmain file: test_3_imp.c\n\r";
	char stringBuff[100];
	
	
	// device object setup
	max31865_t max;
	max.selectChip = chipSelect;
	max.unselectChip = chipUnselect;
	max.rtd = 100;
	max.Rref = 430;
	max.configurationReg = 0x11;
	
	
	DDRB |=(1 << PINB1);
	PORTB |= (1 << PINB1);
	
	usartSettup();
	hal_USART_puts(header);
	
	
	
	max_settup();
	
	while(1)
	{
		//writeConfigReg(0xB2);
		/*
		regBuff[0] = 0xB2;
		writeNReg(max,0x80,regBuff,1);
		*/
		
		// configuration register setup
		writeReg(max,0x80,0xB3);
		
		//_delay_ms(10);
		_delay_ms(1000);
		
		//buff = readRTD();
		buff = readADC(max);
		
		
		
		//RTDres = (uint16_t) (((uint32_t)(buff) * (uint32_t)(max.Rref)) / (uint32_t)(32768));
		
		//RTDres = (uint16_t)(((uint32_t)(buff) * (uint32_t)(max.Rref))  / (uint32_t)(32768));
		
		frtd = (((float)(buff) * (float)(max.Rref))  / (float)(32768));
	
		#define float_RTD
		#ifdef float_RTD
		sprintf(stringBuff,RESET"\n\r\n\rmeasurement #%3d:\n\rADC value: %u\n\rRTD resistance: %f",cnt++,buff,frtd);
		#else
		#ifdef _coloredTerminal
		sprintf(stringBuff,GRN"\n\r\n\rmeasurement #"RED"%3d"GRN":\n\rADC value: "RED"%u"GRN"\n\rRTD resistance: "RED"%u"RESET,cnt++,buff,RTDres);
		#else
		sprintf(stringBuff,RESET"\n\r\n\rmeasurement #%3d:\n\rADC value: %u\n\rRTD resistance: %u",cnt++,buff,RTDres);
		#endif
		#endif
		//sprintf(stringBuff,"\n\rRTD ADC value #%3d: %u\n\rRTD resistance: %u",cnt++,buff,RTDres);
		hal_USART_puts(stringBuff);
		
		
		/*
		// turn off v bias
		writeReg(max,0x80,0x13);
		_delay_ms(100);
		*/
		
		
		
		//readConfigReg();
		
		
		
		_delay_ms(500);
		
	}
}
#endif // _test_3_imp
