/*
 * max31865.c
 *
 * Created: 13-Nov-18 17:14:52
 *  Author: Edwin
 */ 

#include "max31865.h"
#include "HAL/hal_spi.h"
#include <util/delay.h>

static fptr_t spi_start = 0;
static fptr_t spi_stop = 0;
static fptr_t spi_trx = 0;


// temperature curve polynom approximation coefficients
static const float a1 = 2.55865721669;
static const float a2 = 0.000967360412;
static const float a3 = 0.000000731467;
static const float a4 = 0.000000000691;
static const float a5 = 7.31888555389e-13;

//////////////////////////////////////////////////////////////////////////

void max31865_register_spi_start(fptr_t cb)
{
	spi_start = cb;
}

void max31865_unregister_spi_start()
{
	spi_start = 0;
}

void max31865_register_spi_stop(fptr_t cb)
{
	spi_stop = cb;
}

void max31865_unregister_spi_stop()
{
	spi_stop = 0;
}

void max31865_register_spi_trx(fptr_t cb)
{
	spi_trx = cb;
}

void max31865_unregister_spi_trx()
{
	spi_trx = 0;
}

//////////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////////////


void max31865_initDevice(max31865_t device)
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
	
	// settup device
	device.selectChip();
	// select register
	hal_SPI_trx(max31865_reg_CONFIGURATION_W);
	hal_SPI_trx(device.configurationReg);
	device.unselectChip();
	
	device.selectChip();
	// making use of the auto register increment
	// select address
	hal_SPI_trx(max31865_reg_HIGH_FAULT_THRESHHOLD_MSB_W);
	hal_SPI_trx((uint8_t)(device.highFaultTresh >> 7));
	hal_SPI_trx((uint8_t)(device.highFaultTresh));
	hal_SPI_trx((uint8_t)(device.lowFaulTresh >> 7));
	hal_SPI_trx((uint8_t)(device.lowFaulTresh));
	device.unselectChip();
}

uint8_t max31865_checkFaultStat(max31865_t device)
{
	// TODO:check Fault State function
	uint8_t status;
	device.selectChip();
	hal_SPI_trx(max31865_reg_FAULT_STATUS_R);
	status = hal_SPI_trx(0);
	device.unselectChip();
	return status;
}

#define in_bearbeitung
#ifdef in_bearbeitung

uint16_t max31865_readADC(max31865_t device)
{
	
	/*
	uint8_t buff[2];
	uint16_t ADC_val = 0;
	device.selectChip();
	
	readNReg(device,0x01,buff,2);
	// chip unselect
	device.unselectChip();
	ADC_val = (uint16_t)((buff[0]<<8) | buff[1]);
	ADC_val >>= 1;
	
	return ADC_val;
	*/
	uint8_t buff[2];
	uint16_t ADC_val = 0;
	
	//initiate 1 shot
	
	// delete auto conversion mode
	device.configurationReg &= ~0x40;
	
	// turn on Vbias + initiate 1-shot
	writeReg(device,max31865_reg_CONFIGURATION_W,device.configurationReg | 0xA0);
	// wait until RTDIN capacitor is charged
	_delay_ms(65);
	
	// read ADC
	readNReg(device,max31865_reg_RTD_LSB_R,buff,2);
	
	ADC_val = (uint16_t)((buff[0]<<8) | buff[1]);
	ADC_val >>= 1;
	
	return ADC_val;
}

float max31865_readRTD(max31865_t device)
{
	return (((float)(max31865_readADC(device)) * (float)(device.Rref))  / (float)(32768));
}
#endif


float max31865_readCelsius(max31865_t device)
{
	// TODO: read Clesius function
	uint16_t x = max31865_readRTD(device);
	
	float h = (float)(device.rtd - x);
	float h2 = h*h;
	float h3 = h2*h;
	float h4 = h2*h2;
	float h5 = h4*h;
	return a1*h+a2*h2+a3*h3+a4*h4+a5*h5;
}

float max31865_readKelvin(max31865_t device)
{
	return max31865_readCelsius(device) - 273.15;
}


//////////////////////////////////////////////////////////////////////////