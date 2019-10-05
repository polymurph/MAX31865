/*
 * hal_usart.c
 *
 * Created: 08-Nov-18 17:40:43
 *  Author: Edwin
 */ 


// http://ee-classes.usc.edu/ee459/library/documents/avr_intr_vectors/

#define BAUD_PRESCALE(baude_val) (((F_CPU / (baude_val * 16UL))) - 1)

//////////////////////////////////////////////////////////////////////////

#include "hal_usart.h"
#include "dummy.h"

//////////////////////////////////////////////////////////////////////////
static charSize_t charakterSize = charSize_8BIT;

static fptr_t rxCallback = dummy;
static fptr_t dataRegEmptyCallback = dummy;
static fptr_t txCallback = dummy;


//////////////////////////////////////////////////////////////////////////
void hal_USART_enableModule()
{
	PRR &= ~(1 <<PRUSART0);
}

void hal_USART_dissableModule()
{
	PRR |= (1 <<PRUSART0);
}

uint8_t hal_USART_readDataReg()
{
	return UDR0;
}

uint16_t hal_USART_readDataReg_9Bit()
{
	return (((UCSR0B & 0x01) << 8) | UDR0); 
}

void hal_USART_writeDataReg(uint8_t data)
{
	UDR0 = data;
}

void hal_USART_writeDataReg_9Bit(uint16_t data)
{
	UCSR0B |= (data >> 8);
	UDR0 = data;
}

// not sure yet
uint8_t hal_USART_checkRxFrameErr()
{
	return ((UCSR0A & (1 << FE0)) >> FE0);
}
// not sure yet
uint8_t hal_USART_checkRxDataOverRun()
{
	return ((UCSR0A & (1 << DOR0)) >> DOR0);
}

uint8_t hal_USART_checkRxParityErr()
{
	return ((UCSR0A & (1 << UPE0)) >> UPE0);
}

void hal_USART_enableDoubleTxSpeed()
{
	UCSR0A = (UCSR0A & (1 << MPCM0)) | (1 << U2X0);
}

void hal_USART_dissableDoubleTxSpeed()
{
	UCSR0A = (UCSR0A & (1 << MPCM0));
}

void hal_USART_enableMultiProcessorComMode()
{
	UCSR0A = (UCSR0A & (1 << U2X0)) | (1 << MPCM0);
}

void hal_USART_dissableMultiProcessorComMode()
{
	UCSR0A = (UCSR0A & (1 << U2X0));
}

void hal_USART_RxEnable()
{
	UCSR0B |= (1 << RXEN0);
}

void hal_USART_RxDissable()
{
	UCSR0B &= ~(1 << RXEN0);
}

void hal_USART_TxEnable()
{
	UCSR0B |= (1 << TXEN0);
}

void hal_USART_TxDissable()
{
	UCSR0B &= ~(1 << TXEN0);
}

void hal_USART_setMode(usart_mode_t mode)
{
	UCSR0C &= ~((1 << UMSEL00) | (1 << UMSEL01));
	UCSR0C |= (mode << UMSEL00);
}

void hal_USART_setParityMode(parityMode_t mode)
{
	UCSR0C &= ~((1 << UPM00) | (1 << UPM01));
	UCSR0C |= (mode << UPM00);
}

void hal_USART_setStopBits(stopBit_t bit)
{
	UCSR0C &= ~(1 << USBS0);
	UCSR0C |= (bit << USBS0);
}

void hal_USART_setCharSize(charSize_t size)
{
	/*
	UCSR0C &= ~((1 << UCSZ00) | (1 << UCSZ01));
	//UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);
	UCSR0C |= (size & 0x03) << 1;
	*/
	charakterSize = size;
	size &= 0x07;
	
	UCSR0B &= ~(1 << UCSZ02);
	UCSR0C &= ~((1 << UCSZ00) | (1 << UCSZ01));
	
	//UCSR0B |= size & 0x04;
	UCSR0C |= (size & 0x03) << 1;
	
}

void hal_USART_setClockPolarity(clkPol_t polarity)
{
	UCSR0C &= ~(1 << UCPOL0);
	UCSR0C |= (polarity << UCPOL0);
}

void hal_USART_setBaudRate(uint32_t baudRate)
{
	//baudRate &= 0x0FFF;	
	UBRR0L = ((((F_CPU / (baudRate * 16UL))) - 1));
	UBRR0H = (((((F_CPU / (baudRate * 16UL))) - 1)) >> 8);
}

//////////////////////////////////////////////////////////////////////////

void hal_USART_registerRxCallback(fptr_t callback)
{
	rxCallback = callback;
	UCSR0B |= (1 << RXCIE0);
}

void hal_USART_unregisterRXCallback()
{
	UCSR0B &= ~(1 << RXCIE0);
	rxCallback = dummy;
}

void hal_USART_registerDataRegEmptyCallback(fptr_t callback)
{
	UCSR0B |= (1 << UDRIE0);
	dataRegEmptyCallback = callback;
}

void hal_USART_unregisterDataRegEmptCallback()
{
	UCSR0B &= ~(1 << UDRIE0);
	dataRegEmptyCallback = dummy;
}

void hal_USART_registerTxCallback(fptr_t callback)
{
	UCSR0B |= (1 << TXCIE0);
	txCallback = callback;
}

void hal_USART_unregisterTXCallback()
{
	UCSR0B &= ~(1 << TXCIE0);
	txCallback = dummy;
}

//////////////////////////////////////////////////////////////////////////

void hal_USART_putc(const char send)
{
	// Do nothing for a bit if there is already
	// data waiting in the hardware to be sent
	while ((UCSR0A & (1 << UDRE0)) == 0);
	UDR0 = send;
}

void hal_USART_puts(const char *send)
{
	// Cycle through each character individually
	while (*send != '\0') 
	{
		hal_USART_putc(*send++);
	}
}

//////////////////////////////////////////////////////////////////////////

ISR (USART_RX_vect)
{
	rxCallback();
}

ISR(USART_UDRE_vect)
{
	dataRegEmptyCallback();
}

ISR(USART_TX_vect)
{
	txCallback();
}