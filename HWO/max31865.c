/*
 * max31865.c
 *
 * Created: 22-Dec-18 17:31:07
 *  Author: Edwin Koch
 */ 

#include "max31865.h"
#include <stdint.h>
#include <stdbool.h>

enum REG{
    REG_READ_CONFIGURATION = 0x00,
    REG_READ_RTD_MSB,
    REG_READ_RTD_LSB,
    REG_READ_HIGH_FAULT_TH_MSB,
    REG_READ_HIGH_FAULT_TH_LSB,
    REG_READ_LOW_FAULT_TH_MSB,
    REG_READ_LOW_FAULT_TH_LSB,
    REG_READ_FAULT_STATUS,
    REG_WRITE_CONFIGURATION = 0x80,
    REG_WRITE_HIGH_FAULT_TH_MSB = 0x83,
    REG_WRITE_HIGH_FAULT_TH_LSB,
    REG_WRITE_LOW_FAULT_TH_MSB,
    REG_WRITE_LOW_FAULT_TH_LSB
};

enum {
    D0 = 0x01,
    D1 = 0x02,
    D2 = 0x04,
    D3 = 0x08,
    D4 = 0x10,
    D5 = 0x20,
    D6 = 0x40,
    D7 = 0x80
};

// temperature curve polynomial approximation coefficients
static const float a1 = 2.55865721669; /*!< 1. polynomial coeff. for
                                            temperature curve*/
static const float a2 = 0.000967360412; /*!< 2. polynomial coeff. for
                                            temperature curve*/
static const float a3 = 0.000000731467; /*!< 3. polynomial coeff. for
                                            temperature curve*/
static const float a4 = 0.000000000691; /*!< 4. polynomial coeff. for
                                            temperature curve*/
static const float a5 = 7.31888555389e-13; /*!< 5. polynomial coeff. for
                                            temperature curve*/

static void _write_n_reg(const max31865_t*  device,
                         uint8_t            start_reg_address,
                         const uint8_t*     data,
                         uint8_t            len)
{
    uint8_t index = 0;

    if(len == 0) return;

    device->chipselect(true);

    device->spi_trx(start_reg_address);

    do{
        device->spi_trx(data[index++]);
    } while(index < len);

    device->chipselect(false);
}

static void _read_n_reg(const max31865_t*   device,
                        uint8_t             start_reg_address,
                        uint8_t*            data,
                        uint8_t             len)
{
    uint8_t index = 0;

    if(len == 0) return;

    device->chipselect(true);
    device->spi_trx(start_reg_address);

    do {
        data[index++] = device->spi_trx(0xFF);
    } while(index < len);

    device->chipselect(false);
}


void max31865_init(max31865_t*  device,
                   fptr_b_t     chipselect_cb,
                   u8_fptr_u8_t spi_trx_cb,
                   fptr_t       charged_time_delay_cb,
                   fptr_t       conversion_timer_deay_cb,
                   uint16_t     rtd_ohm,
                   uint16_t     rref_ohm,
                   uint16_t     lowerFaulThreshold,
                   uint16_t     higherFaultThreshold,
                   bool         wire_3,
                   bool         filter_50Hz)
{
    uint8_t buff[4];
    uint8_t temp = 0;
    uint16_t temp_1 = 0;

    // object setup
    device->chipselect = chipselect_cb;
    device->spi_trx = spi_trx_cb;
    device->charged_time_delay = charged_time_delay_cb;
    device->conversion_timer_deay = conversion_timer_deay_cb;
    device->rtd = rtd_ohm;
    device->rref = rref_ohm;
    device->lowFaultThreshold = lowerFaulThreshold << 1;
    device->highFaultThreshold = higherFaultThreshold << 1;
    // settup configurations + set a fault status clear (bit auto clear)
    device->configReg = (uint8_t)((wire_3 << 4) | (filter_50Hz) | (1 << 1));

#if 0
    // activate fault detection with automatic delay
    device->configReg &= ~D3;
    device->configReg |= D2;
#endif

    // low and high fault threshold setup

    temp_1 = device->highFaultThreshold;
    buff[0] = (uint8_t)(temp_1);
    buff[1] = (uint8_t)(temp_1 >> 8);
    temp_1 = device->lowFaultThreshold;
    buff[2] = (uint8_t)(temp_1);
    buff[3] = (uint8_t)(temp_1 >> 8);

    temp = device->configReg;
    _write_n_reg(device, REG_WRITE_CONFIGURATION, &temp, 1);
    _write_n_reg(device, REG_WRITE_HIGH_FAULT_TH_MSB, buff, 4);
}


uint16_t max31865_readADC(const max31865_t* device)
{
    uint8_t buff[2] = {0,0};
    uint8_t temp = 0;
    // turn on vbias
    temp = device->configReg | 0x80;
    _write_n_reg(device, 0x80, &temp, 1);

    device->charged_time_delay();

    // initiate 1-shot conversion + vbias
    temp = device->configReg | 0xA0;
    _write_n_reg(device, 0x80, &temp, 1);

    device->conversion_timer_deay();

    _read_n_reg(device, REG_READ_RTD_MSB, buff, 2);

    // turn off vbias
    _write_n_reg(device, REG_WRITE_CONFIGURATION, &(device->configReg), 1);

    //TODO: handle fault bit D0 here! (with callback or other!)


    while(buff[1] & 0x01);
    {
        device->conversion_timer_deay();
    }

    return (((uint16_t)((buff[0]<<8) | buff[1])) >> 1);
}


float max31865_readRTD_ohm(const max31865_t* device)
{
    return (((float)(max31865_readADC(device)) * (float)(device->rref))  / 32768.0);
}


float max31865_readCelsius(const max31865_t* device)
{
    float x = (float)(device->rtd) - max31865_readRTD_ohm(device);
    // return celsius calculated with the help of the horners method
    // reduces needed multiplications and additions
    return -(x * (a1 + x * (a2 + x * (a3 + x * (a4 + x * a5)))));
}


float max31865_readKelvin(const max31865_t* device)
{
    return max31865_readCelsius(device) + 273.15;
}


// TODO: test
void max31865_setHighFaultThreshold(max31865_t* device,
                                    uint16_t    threshold)
{
    uint8_t buff[2];

    device->highFaultThreshold = threshold;
    threshold = threshold << 1;
    buff[0] = (uint8_t)(threshold >> 8);
    buff[1] = (uint8_t)(threshold);

    _write_n_reg(device, REG_WRITE_HIGH_FAULT_TH_MSB, buff, 2);
}

// TODO: test
void max31865_setLowFaultThreshold(max31865_t*  device,
                                   uint16_t     threshold)
{
    uint8_t buff[2];

    device->lowFaultThreshold = threshold;
    threshold = threshold << 1;
    buff[0] = (uint8_t)(threshold >> 8);
    buff[1] = (uint8_t)(threshold);
    _write_n_reg(device, 0x85, buff, 2);
}

int8_t max31865_checkThresholdFault(const max31865_t* device)
{
    //uint8_t buff = readReg(device,0x07) & ~0x3F;
    uint8_t buff = 0;
    _read_n_reg(device,REG_READ_FAULT_STATUS,&buff,1);

    if(buff & max31865_err_RTD_HIGH_THRESHOLD) return 1;
    if(buff & max31865_err_RTD_LOW_THRESHOLD) return -1;

    // no fault
    return 0;
}

uint8_t max31865_readFault(const max31865_t* device)
{
    uint8_t buff = 0x84;

#if as_docu_says

    uint8_t temp = 0;

    // manual fault detection
    // 1. VBIAS on for >= 5 time constants of the capacitor charge time
    // 2. write 0b100x100x to REG_WRITE_CONFIGURATION
    // 3. wait for >= 5 time constants of the capacitor charge time
    // 4. write 0b100x110x to REG_WRITE_CONFIGURATION
    // 5. read fault REG_READ_FAULT_STATUS

    // 1.
    temp = device->configReg | 0x80;
    _write_n_reg(device, 0x80, &temp, 1);
    device->charged_time_delay();
    device->charged_time_delay();
    // 2.
    temp = 0b10001000;
    _write_n_reg(device, REG_WRITE_CONFIGURATION, &temp, 1);
    // 3.
    device->charged_time_delay();
    // 4.
    temp = 0b10001100;
    _write_n_reg(device, REG_WRITE_CONFIGURATION, &temp, 1);

    device->charged_time_delay();
    device->charged_time_delay();
    device->charged_time_delay();

    _read_n_reg(device, REG_READ_FAULT_STATUS, &buff,1);

    // reset to old settings
    //temp = device->configReg;
    //_write_n_reg(device, REG_WRITE_CONFIGURATION, &temp, 1);
#endif

    _write_n_reg(device, REG_WRITE_CONFIGURATION, &buff, 1);
    device->charged_time_delay();

    _read_n_reg(device, REG_READ_FAULT_STATUS, &buff, 1);

    return buff;
}

void max31865_clearFault(const max31865_t* device)
{
    uint8_t temp = (device->configReg | 0x02);
    _write_n_reg(device,REG_WRITE_CONFIGURATION, &temp, 1);
}



