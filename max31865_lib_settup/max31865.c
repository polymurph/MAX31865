/*
 * max31865.c
 *
 * Created: 22-Dec-18 17:31:07
 *  Author: Edwin
 */ 
#include "max31865.h"
#include "hal_spi.h"
//#include "dummy.h"
#define F_CPU 16000000UL
#include <util/delay.h>

//////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////

static fptr_ret_t spi_trx = NULL_PTR;

//////////////////////////////////////////////////////////////////////////

// temperature curve polynomial approximation coefficients
static const float a1 = 2.55865721669;
static const float a2 = 0.000967360412;
static const float a3 = 0.000000731467;
static const float a4 = 0.000000000691;
static const float a5 = 7.31888555389e-13;

//////////////////////////////////////////////////////////////////////////
// local functions

void readNReg(max31865_t device,uint8_t addr, uint8_t *buff, uint8_t n)
{
	uint8_t index = 0;
	
	if(n == 0) return;
	
	device.selectChip();
	
	spi_trx(addr);
	
	do
	{
		buff[index++] = spi_trx(0xFF);
	}while(index < n);
	
	device.unselectChip();
}

uint8_t readReg(max31865_t device, uint8_t addr)
{
	uint8_t buff = 0;
	readNReg(device,addr,&buff,1);
	return buff;
}

void writeNReg(max31865_t device, uint8_t addr,const uint8_t *buff, uint8_t n)
{
	uint8_t index = 0;
	
	device.selectChip();
	
	spi_trx(addr);
	
	do
	{
		spi_trx(buff[index++]);
		n--;
	} while (n > 0);
	
	
	device.unselectChip();
}

void writeReg(max31865_t device, uint8_t addr,uint8_t data)
{
	writeNReg(device,addr,&data,1);
}

//////////////////////////////////////////////////////////////////////////
// interface functions

void max31865_register_spi_trx(fptr_ret_t cb)
{
	spi_trx = cb;
}

void max31865_unregister_spi_trx()
{
	spi_trx = NULL_PTR;
}

void max31865_SPIsetup()
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

void max31865_configDevice(max31865_t device)
{
	uint8_t buff[4];
	
	// turn off vbias, deactivate auto conversion mode and erase fault status clear bit
	writeReg(device,0x80,device.configReg);
	
	// low and high fault threshold setup
	buff[0] = (uint8_t)(device.highFaultThreshold >> 8);
	buff[1] = (uint8_t)(device.highFaultThreshold);
	buff[2] = (uint8_t)(device.lowFaultThreshold >> 8);
	buff[3] = (uint8_t)(device.lowFaultThreshold);
	
	writeNReg(device,0x83,buff,4);
}

uint16_t max31865_readADC(max31865_t device)
{
	uint8_t buff[2];
	uint16_t ADC_val = 0;
	
	// turn on vbias
	writeReg(device,0x80,(device.configReg | 0x80)); 
	
	// wait until rtd filter capacitor is charged and
	_delay_ms(RTD_CAPACITOR_CHARGETIME_ms);
	
	// initiate 1-shot conversion
	writeReg(device,0x80,(device.configReg | 0xA0));
	
	// wait until conversion is complete
	_delay_ms(63);
	
	readNReg(device,0x01,buff,2);
	
	// turn off vbias
	writeReg(device,0x80,device.configReg);
	
	ADC_val = (uint16_t)((buff[0]<<8) | buff[1]);
	ADC_val >>= 1;
	
	return ADC_val;
}

float max31865_readRTD(max31865_t device)
{
	return (((float)(max31865_readADC(device)) * (float)(device.rref))  / (float)(32768));
}

float max31865_readCelsius(max31865_t device)
{
	volatile float x = max31865_readRTD(device);
	
	//volatile float h = (float)(device.rtd) - x;
	volatile float h = (float)(device.rtd) - x;
	volatile float h2 = h*h;
	volatile float h3 = h2*h;
	volatile float h4 = h2*h2;
	volatile float h5 = h4*h;
	return - (a1*h+a2*h2+a3*h3+a4*h4+a5*h5);
}

float max31865_readKelvin(max31865_t device)
{
	return max31865_readCelsius(device) + 273.15; 
}

void max31865_setHighFaultThreshold(max31865_t device, uint16_t threshold)
{
	uint8_t buff[2];
	
	device.highFaultThreshold = threshold;
	
	buff[0] = (uint8_t)(threshold >> 8);
	buff[1] = (uint8_t)(threshold);
	
	writeNReg(device,0x83,buff,2);	
}

void max31865_setLowFaultThreshold(max31865_t device, uint16_t threshold)
{
	uint8_t buff[2];
	
	device.lowFaultThreshold = threshold;
	
	buff[0] = (uint8_t)(threshold >> 8);
	buff[1] = (uint8_t)(threshold);
	
	writeNReg(device,0x85,buff,2);
}

int8_t max31865_checkThresholdFault(max31865_t device)
{
	uint8_t buff = readReg(device,0x07) & ~0x3F;
	
	if(buff)
	{
		if(buff & ~0x7F)
		{
			// high fault
			return 1;
		}
		else
		{
			// low fault
			return -1;
		}
	}
	return 0;
}

void max31865_clearFault(max31865_t device)
{
	writeReg(device,0x01,device.configReg | 0x02);
}