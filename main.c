#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "HAL/hal_spi.h"
#include "HWO/max31865.h"

void _chip_select_init(void)
{
    // TODO: init chip select pin
}

void _chip_select(bool select)
{
    // TODO: choose chip select pin
}

void _charge_time_delay()
{
    volatile uint16_t i = 0;
    for(i = 0; i<0xFFF;i++);
}

void _conversion_time_delay()
{
    volatile uint16_t i = 0;
    for(i = 0; i<0xFFF;i++);
}


int main(void)
{
    uint16_t temp = 0;
    max31865_t Temperature;

    _chip_select_init();

    hal_spi_init(spi_mode_MASTER,
                 spi_clk_source_ACLK,
                 spi_clk_mode_2,
                 0,
                 true);

    max31865_init(&Temperature,
                  _chip_select, hal_spi_trx,
                  _charge_time_delay,
                  _conversion_time_delay,
                  1000,
                  400,
                  0,
                  0xFFFF);

    temp = max31865_readADC(&Temperature);


	while(1)
	{

	}
	
	return 0;
}
