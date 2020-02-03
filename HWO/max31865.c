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
    device->lowFaultThreshold = lowerFaulThreshold;
    device->highFaultThreshold = higherFaultThreshold;
    // settup configurations + set a fault status clear (bit auto clear)
    device->configReg = (uint8_t)((wire_3 << 4) | (filter_50Hz) | (1 << 1));

#if 0
    // activate fault detection with automatic delay
    device->configReg &= ~D3;
    device->configReg |= D2;
#endif

    // low and high fault threshold setup

    temp_1 = device->highFaultThreshold << 1;
    buff[0] = (uint8_t)(temp_1);
    buff[1] = (uint8_t)(temp_1 >> 8);
    temp_1 = device->lowFaultThreshold << 1;
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
    uint16_t temp = 0;

    device->highFaultThreshold = threshold;
    temp = threshold << 1;
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
    buff[0] = (uint8_t)(threshold >> 7);
    buff[1] = (uint8_t)(threshold << 1);
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


#if 0

static fptr_ret_t spi_trx = NULL_PTR; /*!< spi_trx callback function*/
static fptr_t chargetime = NULL_PTR; /*!< chargetimer delay callback function*/
static fptr_t conversiontime = NULL_PTR; /*!< conversiontime delay callback function*/

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

/************************************************
 *	@brief Read multiple registers
 *	@param device
 *	@param addr register start address
 *	@param *buff pointer to buffer
 *	@param 	n buffer size
 *	@return
 *		This function reads multiple regsiters\n
 *		starting at the designated register set\¨n
 *      addr. The contetn of each register is\n
 *		stored in the buffer. The ammount of\n
 *		registers to be read is defined by n.\n
 *		After each read the address is automatically\n
 *		incremented to the next address. See datasheet\n
 *      for more information regarding register\n
 *		manipulation.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
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

/************************************************
 *	@brief Read Register
 *	@param	device
 *  @param	addr_t_t
 *	@return uint8_t
 *		This function reads the content of a designated\n
 *		register by addr and returns the content.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
uint8_t readReg(max31865_t device, uint8_t addr)
{
	uint8_t buff = 0;
	readNReg(device,addr,&buff,1);
	return buff;
}

/************************************************
 *	@brief Write n registers
 *	@param device	
 *	@param addr register start address
 *	@param *buff pointer of buffer
 *	@param n buffer size (number of elements)
 *	@return
 *		this function writes multiple registers\n
 *      with the content in buff. the addr is\n
 *      automatically incremented from the set
 *      address (addr) onwards. See datasheet\n
 *      for more information regarding register\n
 *		manipulation.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
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

/************************************************
 *	@brief write one register
 *	@param device
 *	@param addr register address
 *	@param data 
 *		This function writes data to one register\n
 *		selected by addr.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
void writeReg(max31865_t device, uint8_t addr,uint8_t data)
{
	writeNReg(device,addr,&data,1);
}

/************************************************
 *	@brief Register SPI transmit receive function
 *	@param	function pointer
 *		This function sets the SPI transmit receive\n
 *		function pointer to the passed over function\n
 *		pointer.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
void max31865_register_spi_trx(fptr_ret_t cb)
{
	spi_trx = cb;
}

/************************************************
 *	@brief unregister SPI transmit receive function\n
 *		This function sets the SPI transmit receive\n
 *		(trx) function pointer to a zero\n
 *      pointer defined by NULL_PTR in the\n
 *		@file max31865.h header file.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
void max31865_unregister_spi_trx()
{
	spi_trx = NULL_PTR;
}

/************************************************
 *	@brief Register chargetime delay function
 *	@param	function pointer
 *		This function sets the chargetime delay\n
 *		function pointer to the passed over function\n
 *		pointer.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
void max31865_register_chargetime_delay(fptr_t cb)
{
	chargetime = cb;
}

/************************************************
 *	@brief unregister chargetime delay function\n
 *		This function sets the chargetime\n
 *		function pointer to a zero\n
 *      pointer defined by NULL_PTR in the\n
 *		@file max31865.h header file.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
void max31865_unregister_chargetime_delay()
{
	chargetime = NULL_PTR;
}

/************************************************
 *	@brief Register conversiontime delay function
 *	@param	function pointer
 *		This function sets the conversiontime delay\n
 *		function pointer to the passed over function\n
 *		pointer.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
void max31865_register_conversiontime_delay(fptr_t cb)
{
	conversiontime = cb;
}

/************************************************
 *	@brief unregister conversiontime delay function\n
 *		This function sets the conversiontime\n
 *		function pointer to a zero\n
 *      pointer defined by NULL_PTR in the\n
 *		@file max31865.h header file.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
void max31865_unregister_conversiontime_delay()
{
	conversiontime = NULL_PTR;
}

/************************************************
 *	@brief device configurator
 *	@param	device max31865_t device typedef struct
 *	@return
 *		This function configures the device with all\n
 *		the parameters of the device parameter
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
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

/************************************************
 *	@brief short description
 *	@param	device is a max31865_t struct type
 *	@return uint16_t raw 15-bit ADC value
 *
 *	@todo make delay independent of target device\n
 *		  -> implement register unregister functions
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
uint16_t max31865_readADC(max31865_t device)
{
	uint8_t buff[2];
	uint16_t ADC_val = 0;
	
	// turn on vbias
	writeReg(device,0x80,(device.configReg | 0x80)); 
	
	// wait until rtd filter capacitor is charged and
	//delay_ms(RTD_CAPACITOR_CHARGETIME_ms);
	chargetime();
	
	// initiate 1-shot conversion
	writeReg(device,0x80,(device.configReg | 0xA0));
	
	// wait until conversion is complete
	//delay_ms(63);
	conversiontime();
	
	readNReg(device,0x01,buff,2);
	
	// turn off vbias
	writeReg(device,0x80,device.configReg);
	
	ADC_val = (uint16_t)((buff[0]<<8) | buff[1]);
	ADC_val >>= 1;
	
	return ADC_val;
}

/************************************************
 *	@brief short description
 *	@param	
 *	@return
 *		detailed description
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
float max31865_readRTD(max31865_t device)
{
	return (((float)(max31865_readADC(device)) * (float)(device.rref))  / (float)(32768));
}

/************************************************
 *	@brief Read Temperature in Celsius
 *	@param	device max31865_t
 *	@return float °C value
 *		this function reads the raw ADC value and\n
 *		and calculates the °C value with the help\n
 *		of the polynomial approximation of the\n
 *		RTD curve.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
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

/************************************************
 *	@brief Read Temperature in Kelvin
 *	@param	device
 *	@return float K Kelvin
 *		This function reads the °C value and\n
 *		and converts it to Kelvin by adding\n
 *		the constant 273.15 to the °C value.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
float max31865_readKelvin(max31865_t device)
{
	return max31865_readCelsius(device) + 273.15; 
}

/************************************************
 *	@brief Set high fault threshold
 *	@param	device
 *	@param  threshold
 *		This function sets the high fault threshold.\n
 *		If the raw ADC value goes over the set value\n
 *		a Fault is generated.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
void max31865_setHighFaultThreshold(max31865_t device, uint16_t threshold)
{
	uint8_t buff[2];
	
	device.highFaultThreshold = threshold;
	
	buff[0] = (uint8_t)(threshold >> 8);
	buff[1] = (uint8_t)(threshold);
	
	writeNReg(device,0x83,buff,2);	
}

/************************************************
 *	@brief set low fault threshold
 *	@param device
 *	@param threshold
 *		This function sets the low fault threshold.\n
 *		If the raw ADC value goes under the set value\n
 *		a Fault is generated.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
void max31865_setLowFaultThreshold(max31865_t device, uint16_t threshold)
{
	uint8_t buff[2];
	
	device.lowFaultThreshold = threshold;
	
	buff[0] = (uint8_t)(threshold >> 8);
	buff[1] = (uint8_t)(threshold);
	
	writeNReg(device,0x85,buff,2);
}

/************************************************
 *	@brief check for threshold fault
 *	@param	device
 *	@return int8_t
 *		This function checks for threshold fault.\n
 *		In case of a high fault 1 is returned \n
 *		and for low fault a -1 and for no fault 0.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
int8_t max31865_checkThresholdFault(max31865_t device)
{
	uint8_t buff = readReg(device,0x07) & ~0x3F;
	
	if(buff) {
		if(buff & ~0x7F) {
			// high fault
			return 1;
		} else {
			// low fault
			return -1;
		}
	}
	return 0;
}

/************************************************
 *	@brief clear fault
 *	@param	device
 *		This function clears the thresold fault.
 *
 *	@todo
 *	@test
 *	@bug
 *
 *	@version 1.0
 ***********************************************/
void max31865_clearFault(max31865_t device)
{
	writeReg(device,0x01,device.configReg | 0x02);
}
#endif
