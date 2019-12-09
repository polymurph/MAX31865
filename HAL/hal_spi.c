/*
 * hal_spi.c
 *
 *  Created on: 11.11.2019
 *      Author: timon
 */
#include "hal_spi.h"
#include <stdint.h>
#include <stdbool.h>
#include "msp430fr6989.h"


static inline void _gpio_setup(void)
{
    PM5CTL0 &= ~LOCKLPM5;
    // MOSI
    P1DIR |= 0x40;
    P1SEL0 |= 0x40;
    P1SEL1 &= ~0x40;

    // MISO
    P1DIR &= ~0x80;
    P1SEL0 |= 0x80;
    P1SEL1 &= ~0x80;

    // CLK
    P1DIR |= 0x10;
    P1SEL0 |= 0x10;
    P1SEL1 &= ~0x10;

    PM5CTL0 |= LOCKLPM5;
}

bool hal_spi_init(spi_mode_t        mode,
                  spi_clk_source_t  clk_source,
                  spi_clk_mode_t    clk_mode,
                  uint16_t          prescaler,
                  bool              MSB_first)
{
    // enable register modification
    UCB0CTLW0 |= UCSWRST;

    // reset entire register
    UCB0CTLW0 &= ~0xFFFE;

    // 3 pin spi mode
    UCB0CTLW0 &= ~(UCMODE0 | UCMODE1);

    // set synchronous mode
    UCB0CTLW0 |= UCSYNC;

    // set master or slave mode
    UCB0CTLW0 |= mode;

    //MSB or LSB first
    UCB0CTLW0 |= (MSB_first) ? (UCMSB) : (0);

    // set clock prescaler
    UCB0BRW = prescaler;

    // set clock phase and polarity
    UCB0CTLW0 |= clk_mode;

    // set 8 bit mode
    UCB0CTLW0 &= ~UC7BIT;

    //set clock source
    if(mode){
        // master mode
        if(clk_source == spi_clk_source_UC0CLK){
            UCB0CTLW0 &= ~UCSWRST;
            return true;
        }
         UCB0CTLW0 |= clk_source;
    } else {
         // slave mode
         if(clk_source != spi_clk_source_UC0CLK){
             UCB0CTLW0 &= ~UCSWRST;
             return true;
         }
         UCB0CTLW0 |= clk_source;
    }

    _gpio_setup();

    // disable register modification
    UCB0CTLW0 &= ~UCSWRST;
    return false;
}

uint8_t hal_spi_trx_byte(uint8_t data)
{
      UCB0IFG = 0;
      UCB0TXBUF = data;
      // poll for receive completion
      while(UCB0IFG & UCRXIFG);
      return UCB0RXBUF;
}

//chip select procedure must be done by user
void hal_spi_tx_byte(uint8_t data)
{
    UCB0IFG = 0;
    // transmit
    UCB0TXBUF = data;
    while(UCB0STATW & UCBUSY);
}

uint8_t hal_spi_rx_byte(void)
{
    UCB0IFG = 0;
    UCB0TXBUF = 0x00;
    while(UCB0STATW & UCBUSY);
    return UCB0RXBUF;
}

void hal_spi_trx(const uint8_t* txblock, uint8_t* rxblock, uint8_t len)
{
    uint8_t i = 0;

    for(i = 0; i < len; i++){
        rxblock[i] = hal_spi_trx_byte(txblock[i]);
    }
}

void hal_spi_tx(const uint8_t*   txblock,
                uint8_t          len)
{
    uint8_t i = 0;

    for(i = 0; i < len; i++){
        hal_spi_trx_byte(txblock[i]);
    }
}

void hal_spi_rx(uint8_t*   rxblock,
                uint8_t    len)
{
    uint8_t i = 0;

    for(i = 0; i < len; i++){
        rxblock[i] = hal_spi_trx_byte(0x00);
    }
}
