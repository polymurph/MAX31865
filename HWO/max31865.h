/*
 * max31865.h
 *
 * Created: 22-Dec-18 17:30:56
 *  Author: Edwin Koch
 */ 

/*
 *  chipselect: lowactive
 *  MSB first:  true
 *
 */

#ifndef MAX31865_H_
#define MAX31865_H_
#include <stdint.h>
#include <stdbool.h>

#define MAX31865_RTD_CAPACITOR_CHARGETIME_ms (2)
#define MAX31865_DELAY_63_ms (63)

typedef void (*fptr_t)(void);
typedef void (*fptr_b_t)(bool);
typedef uint8_t (*u8_fptr_u8_t)(uint8_t);

typedef enum{
    max31865_err_VOLTAGE_FAULT = 0x04,
    max31865_err_VRTDIN_TO_LOW = 0x08,
    max31865_err_VREFIN_TO_LOW = 0x10,
    max31865_err_VREFIN_TO_HIGH = 0x20,
    max31865_err_RTD_LOW_THRESHOLD = 0x40,
    max31865_err_RTD_HIGH_THRESHOLD = 0x80
}max31865_err_t;

typedef struct{
    fptr_b_t chipselect;
    u8_fptr_u8_t spi_trx;
    fptr_t charged_time_delay;
    fptr_t conversion_timer_deay;
    fptr_t highFaultThreshold_cb;
    fptr_t lowFaultThreshold_cb;
    uint16_t rtd;
    uint16_t rref;
    uint16_t lowFaultThreshold;
    uint16_t highFaultThreshold;
    uint8_t configReg;
}max31865_t;


void max31865_init(max31865_t*  device,
                   fptr_b_t     chipselect_cb,
                   u8_fptr_u8_t spi_trx_cb,
                   fptr_t       charged_time_delay_cb,
                   fptr_t       conversion_timer_deay_cb,
                   fptr_t       highFaultThreshold_callback,
                   fptr_t       lowFaultThreshold_callback,
                   uint16_t     rtd_ohm,
                   uint16_t     rref_ohm,
                   uint16_t     lowerFaulThreshold,
                   uint16_t     higherFaultThreshold,
                   bool         wire_3,
                   bool         filter_50Hz);

uint16_t max31865_readADC(const max31865_t* device);

float max31865_readRTD_ohm(const max31865_t* device);

float max31865_readCelsius(const max31865_t* device);

float max31865_readKelvin(const max31865_t* device);

void max31865_setHighFaultThreshold(max31865_t* device,
                                    uint16_t    threshold);

void max31865_setLowFaultThreshold(max31865_t*  device,
                                   uint16_t     threshold);

int8_t max31865_checkThresholdFault(const max31865_t* device);

uint8_t max31865_readFault(const max31865_t* device);

void max31865_clearFault(const max31865_t* device);

#endif /* MAX31865_H_ */
