#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "HAL/hal_spi.h"
#include "HWO/max31865.h"

void _threshold_fault_cb_init()
{
    PM5CTL0 &= ~LOCKLPM5;

    // LED
    P1DIR |= 0x01;
    P1OUT &= ~0x01;

    // button
    P1DIR &= ~0x02;
    P1REN |= 0x02;
    P1OUT |= 0x02;

    PM5CTL0 |= LOCKLPM5;
}

void _threshold_fault_cb()
{
    while(P1IN & 0x02)
    //while(1)
    {
        __delay_cycles(100000);
        P1OUT ^= 0x01;
        //if(!(P1IN & 0x01)) break;
    }
    P1OUT &= ~0x01;
}

void _chip_select_init(void)
{
    // Pin 2.1
    PM5CTL0 &= ~LOCKLPM5;

    P2DIR |= 0x02;
    P2OUT |= 0x02;

    PM5CTL0 |= LOCKLPM5;
}

void _chip_select(bool select)
{
    if(select){
        P2OUT &= ~0x02;
    } else {
        P2OUT |= 0x02;
    }
}

void _charge_time_delay()
{
    volatile uint16_t i = 0;
    for(i = 0; i<0x1FFF;i++);

    //__delay_cycles(1000);
}

void _conversion_time_delay()
{
    volatile uint16_t i = 0;
    for(i = 0; i<0x1FFF;i++);

    //__delay_cycles(1000);
    // TODO: check if plausible: wait until DRDY goes low (active -> conversion completed)
}

int main(void)
{
    volatile uint16_t temp = 0;
    volatile int8_t status = 0;
    volatile uint8_t fault = 0;
    volatile float rtd_ohm = 0.0;
    volatile float celsius = 0.0;
    volatile float kelvin = 0.0;

    max31865_t Temperature;

    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    _threshold_fault_cb_init();
    _chip_select_init();

    hal_spi_init(spi_mode_MASTER,
                 spi_clk_source_ACLK,
                 spi_clk_mode_1,
                 0,
                 true);

    // init Temperature object

    // void hal_spi_trx(const uint8_t* txblock, uint8_t* rxblock, uint8_t len)

    max31865_init(&Temperature,
                  _chip_select,
                  hal_spi_trx_byte,
                  _charge_time_delay,
                  _conversion_time_delay,
                  _threshold_fault_cb,
                  100,
                  430,  // Rref on breakout board
                  0,
                  0x0FFF,
                  true,    // 3 wire mode
                  false);

    temp = max31865_readADC(&Temperature);

    max31865_setHighFaultThreshold(&Temperature,9000/*7000*/);
    max31865_setLowFaultThreshold(&Temperature, 4000);


	while(1)
	{
	    //temp = max31865_readADC(&Temperature);

	    //rtd_ohm = max31865_readRTD_ohm(&Temperature);
	    celsius = max31865_readCelsius(&Temperature);
	    //kelvin = max31865_readKelvin(&Temperature);

	    //status = max31865_checkThresholdFault(&Temperature);
	    fault = max31865_readFault(&Temperature);

	    //max31865_clearFault(&Temperature);
	    //_charge_time_delay();
	}
	
	return 0;
}
