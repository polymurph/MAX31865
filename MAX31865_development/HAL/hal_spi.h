/*
 * hal_spi.h
 *
 * Created: 31-Oct-18 20:08:07
 *  Author: Edwin
 */ 
/*
	https://embedds.com/serial-peripheral-interface-in-avr-microcontrollers/
*/

#ifndef HAL_SPI_H_
#define HAL_SPI_H_

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

typedef void (*fptr_t) ();

typedef enum
{
	MSB_FIRST,
	LSB_FIST
	
}dataDir_t;

typedef enum
{
	SLAVE,
	MASTER
}mode_t;

typedef enum
{
	NONINVERTED,
	INVERTED
}SPIclkPol_t;

typedef enum
{
	SAMPLE_ON_RISING_EDGE,
	SAMPLE_ON_FALLING_EDGE
}clkPhase_t;

typedef enum
{
	clk_CLK_DIV_4,
	clk_CLK_DIV_16,
	clk_CLK_DIV_64,
	clk_CLK_DIV_128,
	clk_CLK_DIV_2,
	clk_CLK_DIV_8,
	clk_CLK_DIV_32,
	clk_CLK_DIV_64_
}clk_presc_t;

void hal_SPI_enableModule();

void hal_SPI_dissableModule();
//////////////////////////////////////////////////////////////////////////

void hal_SPI_setCLK_LOW();

void hal_SPI_setCLK_HIGH();

//////////////////////////////////////////////////////////////////////////

void hal_SPI_enable_TRX();

void hal_SPI_dataDirectionOrder(dataDir_t direction);

void hal_SPI_setMode(mode_t mode);

void hal_SPI_setClockPolarity(SPIclkPol_t polarity);

void hal_SPI_setClockPhase(clkPhase_t phase);

void hal_SPI_setClockRate(clk_presc_t prescaler);

//////////////////////////////////////////////////////////////////////////

uint8_t hal_SPI_trx(uint8_t data);

uint8_t hal_SPI_readDataRegister();
//////////////////////////////////////////////////////////////////////////
void hal_SPI_enableInterrupt();

void hal_SPI_dissableInterrupt();

void hal_SPI_registerTransferCompletionCallback(fptr_t routine);

void hal_SPI_unregisterTransferCompletionCallback();

#endif /* HAL_SPI_H_ */