/*
 * max31865.h
 *
 * Created: 13-Nov-18 17:15:07
 *  Author: Edwin
 */ 


#ifndef MAX31865_H_
#define MAX31865_H_
//#include <stdbool.h>
#include <stdint.h>

typedef void (*fptr_t) ();

typedef enum
{
	max31865_reg_CONFIGURATION_R,
	max31865_reg_RTD_MSB_R,
	max31865_reg_RTD_LSB_R,
	max31865_reg_HIGH_FAULT_THRESHHOLD_MSB_R,
	max31865_reg_HIGH_FAULT_THRESHHOLD_LSB_R,
	max31865_reg_LOW_FAULT_THRESHHOLD_MSB_R,
	max31865_reg_LOW_FAULT_THRESHHOLD_LSB_R,
	max31865_reg_FAULT_STATUS_R,
	max31865_reg_CONFIGURATION_W = 0x80,
	max31865_reg_HIGH_FAULT_THRESHHOLD_MSB_W = 0x83,
	max31865_reg_HIGH_FAULT_THRESHHOLD_LSB_W,
	max31865_reg_LOW_FAULT_THRESHHOLD_MSB_W,
	max31865_reg_LOW_FAULT_THRESHHOLD_LSB_W
}max31865_regAddr_t;

typedef enum
{
	max31865_bit_50_60_HZ_FILTER,
	max31865_bit_FAULT_STATUS,
	max31865_bit_FAULT_DETECTION_0,
	max31865_bit_FAULT_DETECTION_1,
	max31865_bit_WIRE_2_3_WIRE,
	max31865_bit_1_SHOT,
	max31865_bit_CONVERSION_MODE,
	max31865_bit_VBIAS 
}max31865_bitShift_t;

typedef struct 
{
	uint16_t highFaultTresh;
	uint16_t lowFaulTresh;
	uint16_t rtd;
	uint16_t Rref;
	uint8_t configurationReg;
	fptr_t unselectChip;
	fptr_t selectChip;
}max31865_t;

// SPI functions

// TODO: implement spi functions

void max31865_register_spi_start(fptr_t cb);

void max31865_unregister_spi_start();

void max31865_register_spi_stop(fptr_t cb);

void max31865_unregister_spi_stop();

void max31865_register_spi_trx(fptr_t cb);

void max31865_unregister_spi_trx();

// max31865 functionality functions

// TODO: settup correct spi functions
void max31865_initDevice(max31865_t device);

uint8_t max31865_checkFaultStat(max31865_t device);

uint16_t max31865_readADC(max31865_t device);

float max31865_readRTD(max31865_t device);

float max31865_readCelsius(max31865_t device);

float max31865_readKelvin(max31865_t device);

#endif /* MAX31865_H_ */