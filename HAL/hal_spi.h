/*
 * hal_spi.h
 *
 *  Created on: 11.11.2019
 *      Author: timon
 */

#ifndef HAL_HAL_SPI_H_
#define HAL_HAL_SPI_H_

#include <stdint.h>
#include <stdbool.h>
#include "msp430fr6989.h"

typedef enum{
    spi_mode_MASTER = UCMST,
    spi_mode_SLAVE = 0
}spi_mode_t;

typedef enum{
    spi_mode_3PIN = 0,
    spi_mode_4PIN_AH = UCMODE0,//UCxSTE active high: Slave enabled when UCxSTE = 1
    spi_mode_4PIN_AL = UCMODE1 //UCxSTE active low: Slave enabled when UCxSTE = 0
}spi_pin_mode_t;

typedef enum{
    spi_clk_source_UC0CLK = 0,
    spi_clk_source_ACLK = UCSSEL0,
    spi_clk_source_SMCLK = UCSSEL1
}spi_clk_source_t;

typedef enum{
    spi_data_dir_MSB_FIRST = UCMSB,
    spi_data_dir_LSB_FIRST = 0
}spi_data_dir_t;

// TODO: choose better names
typedef enum{
    spi_clk_mode_0 = 0, //Rising Edge Tx, Falling Edge Rx, Polarity Low
    spi_clk_mode_1= UCCKPL, //Falling Edge Tx, Rising Edge Rx, Polarity High
    spi_clk_mode_2 = UCCKPH, //Falling Edge Tx, Rising Edge Rx, Polarity Low
    spi_clk_mode_3 = UCCKPL | UCCKPH //Rising Edge Tx, Falling Edge Rx, Polarity High
}spi_clk_mode_t;

bool hal_spi_init(spi_mode_t        mode,
                  spi_clk_source_t  clk_source,
                  spi_clk_mode_t    clk_mode,
                  uint16_t          prescaler,
                  bool              MSB_first);

uint8_t hal_spi_trx_byte(uint8_t data);

void hal_spi_tx_byte(uint8_t data);

uint8_t hal_spi_rx_byte(void);

void hal_spi_trx(const uint8_t*   txblock,
                 uint8_t*         rxblock,
                 uint8_t          len);

void hal_spi_tx(const uint8_t*   txblock,
                uint8_t          len);

void hal_spi_rx(uint8_t*   rxblock,
                uint8_t    len);

#endif /* HAL_HAL_SPI_H_ */
