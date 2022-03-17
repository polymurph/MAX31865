/*
MIT License

Copyright (c) 2019 Edwin Koch

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
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
static const float a1 = 2.55865721669;
static const float a2 = 0.000967360412;
static const float a3 = 0.000000731467;
static const float a4 = 0.000000000691; 
static const float a5 = 7.31888555389e-13;

static void _write_n_reg(const max31865_t*  device,
                         uint8_t            start_reg_address,
                         const uint8_t*     data,
                         uint8_t            len);
static void _read_n_reg(const max31865_t*   device,
                        uint8_t             start_reg_address,
                        uint8_t*            data,
                        uint8_t             len);
void _handle_threshold_fault(const max31865_t* device);


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
    device->highFaultThreshold_cb =  highFaultThreshold_callback;
    device->lowFaultThreshold_cb = lowFaultThreshold_callback;
    device->rtd = rtd_ohm;
    device->rref = rref_ohm;
    device->lowFaultThreshold = lowerFaulThreshold << 1;
    device->highFaultThreshold = higherFaultThreshold << 1;
    // settup configurations + set a fault status
    device->configReg = (uint8_t)((wire_3 << 4) | (filter_50Hz));

    // low and high fault threshold setup
    temp_1 = device->highFaultThreshold;
    buff[0] = (uint8_t)(temp_1 >> 8);
    buff[1] = (uint8_t)(temp_1);
    temp_1 = device->lowFaultThreshold;
    buff[2] = (uint8_t)(temp_1 >> 8);
    buff[3] = (uint8_t)(temp_1);

    temp = device->configReg;
    _write_n_reg(device, REG_WRITE_CONFIGURATION, &temp, 1);
    _write_n_reg(device, REG_WRITE_HIGH_FAULT_TH_MSB, buff, 4);
}

uint16_t max31865_readADC(const max31865_t* device)
{
    uint8_t buff[2] = {0,0};
    uint8_t temp = 0;
    // turn on vbias
    temp = device->configReg | D7;
    _write_n_reg(device, REG_WRITE_CONFIGURATION, &temp, 1);

    device->charged_time_delay();

    // initiate 1-shot conversion + vbias
    temp = device->configReg | 0xA0;
    _write_n_reg(device, REG_WRITE_CONFIGURATION, &temp, 1);

    device->conversion_timer_deay();

    _read_n_reg(device, REG_READ_RTD_MSB, &buff, 2);

    // turn off vbias
    _write_n_reg(device, REG_WRITE_CONFIGURATION, &(device->configReg), 1);

    if(buff[1] & 0x01)  {
        _handle_threshold_fault(device);
    }

    return (uint16_t)buff[0] << 7 | (uint16_t)buff[1] >> 1;
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

void max31865_setLowFaultThreshold(max31865_t*  device,
                                   uint16_t     threshold)
{
    uint8_t buff[2];

    device->lowFaultThreshold = threshold;
    threshold = threshold << 1;
    buff[0] = (uint8_t)(threshold >> 8);
    buff[1] = (uint8_t)(threshold);
    _write_n_reg(device, REG_WRITE_LOW_FAULT_TH_MSB, buff, 2);
}

int8_t max31865_checkThresholdFault(const max31865_t* device)
{
    uint8_t buff;
    _read_n_reg(device,REG_READ_FAULT_STATUS, &buff, 1);

    if(buff & max31865_err_RTD_HIGH_THRESHOLD) return 1;
    if(buff & max31865_err_RTD_LOW_THRESHOLD) return -1;

    // no fault
    return 0;
}

uint8_t max31865_readFault(const max31865_t* device)
{
    uint8_t buff;
    _read_n_reg(device, REG_READ_FAULT_STATUS, &buff, 1);
    return buff;
}

void max31865_clearFault(const max31865_t* device)
{
    uint8_t temp = (device->configReg | D1);
    _write_n_reg(device,REG_WRITE_CONFIGURATION, &temp, 1);
}


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

void _handle_threshold_fault(const max31865_t* device)
{
    switch(max31865_readFault(device))
    {
    case max31865_err_RTD_HIGH_THRESHOLD:
        device->highFaultThreshold_cb();
        break;
    case max31865_err_RTD_LOW_THRESHOLD:
        device->lowFaultThreshold_cb();
        break;
    default:
        break;
    }
    max31865_clearFault(device);
}
