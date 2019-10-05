/*
 * hal_spi.c
 *
 * Created: 31-Oct-18 20:07:57
 *  Author: Edwin
 */ 
#include "hal_spi.h"
#include "dummy.h"

static fptr_t callback = dummy;

void hal_SPI_enableModule()
{
	PRR &= ~(1 << PRSPI);
}

void hal_SPI_dissableModule()
{
	PRR |= (1 << PRSPI);
}

//////////////////////////////////////////////////////////////////////////

void hal_SPI_setCLK_LOW()
{
	DDRB &= ~(1 << PINB5);
}

void hal_SPI_setCLK_HIGH()
{
	DDRB |= (1 << PINB5);
}

//////////////////////////////////////////////////////////////////////////
void hal_SPI_enable_TRX()
{
	SPCR |= (1<<SPE);
}

void hal_SPI_dissable_trx()
{
	SPCR &= ~(1<<SPE);
}

void hal_SPI_dataDirectionOrder(dataDir_t direction)
{
	SPCR &= ~(1 << DORD);
	SPCR |= (direction << DORD);
}

void hal_SPI_setMode(mode_t mode)
{
	SPCR &= ~(1 << MSTR);
	SPCR |= (mode << MSTR);
	
	if(mode == MASTER)
	{
		DDRB |= (1<<PINB3)|(1<<PINB5)|(1<<PINB2);
		DDRB &= ~(1 << PINB4);
		
		// Latch Disable (RCK Low)
		PORTB &= ~(1<<PINB2);
		
	}
	if(mode == SLAVE)
	{
		DDRB &= ~((1<<PINB3)|(1<<PINB5)|(1<<PINB2));
		DDRB |= (1 << PINB4);
	}
	
}

void hal_SPI_setClockPolarity(SPIclkPol_t polarity)
{
	SPCR &= ~(1 << CPOL);
	SPCR |= (polarity << CPOL);
}

void hal_SPI_setClockPhase(clkPhase_t phase)
{
	SPCR &= ~(1 << CPHA);
	SPCR |= (phase << CPHA);
}

void hal_SPI_setClockRate(clk_presc_t prescaler)
{
	SPCR &= ~0x03;
	SPCR |= prescaler & 0x03;
	SPSR |= (prescaler >> 2);
}

//////////////////////////////////////////////////////////////////////////

uint8_t hal_SPI_trx(uint8_t data)
{
	SPDR = data;
	
	while(!(SPSR & (1 << SPIF)));

	return (SPDR);
}

uint8_t hal_SPI_readDataRegister()
{
	return SPDR;
}

//////////////////////////////////////////////////////////////////////////

void hal_SPI_enableInterrupt()
{
	SPCR |= (1 << SPIE);
}

void hal_SPI_dissableInterrupt()
{
	SPCR &= ~(1 << SPIE);
}

void hal_SPI_registerTransferCompletionCallback(fptr_t routine)
{
	callback = routine;
	SPCR |= (1 << SPIE);
}

void hal_SPI_unregisterTransferCompletionCallback()
{
	callback = dummy;
	SPCR &= ~(1 << SPIE);
}

ISR(SPI_STC_vect)
{
	callback();
}