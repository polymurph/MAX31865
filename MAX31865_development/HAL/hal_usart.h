/*
 * hal_usart.h
 *
 * Created: 08-Nov-18 17:40:59
 *  Author: Edwin
 */ 

#ifndef HAL_USART_H_
#define HAL_USART_H_

//////////////////////////////////////////////////////////////////////////

#include <avr/interrupt.h>
#include <stdint.h>
#include <avr/io.h>

//////////////////////////////////////////////////////////////////////////
#define F_CPU 16000000UL
//////////////////////////////////////////////////////////////////////////

//#define BAUD_PRESCALER(BAUD,CLK) (((CLK / (BAUD * 16UL))) - 1)


//////////////////////////////////////////////////////////////////////////

typedef void (*fptr_t) (); /*<*/

typedef enum
{
	mode_ASYNCHRON_USART,
	mode_SYNCHRONOUS_USART
	//mode_MASTER_SPI = 3
	
}usart_mode_t;

typedef enum
{
	parityMode_DISSABELD,
	parityMode_EVEN_PARITY = 2,
	parityMode_ODD_PARITY
}parityMode_t;


typedef enum
{
	stopBit_1_BIT,
	stopBit_2_BIT
}stopBit_t;

typedef enum
{
	charSize_5BIT = 0,
	charSize_6BIT = 1,
	charSize_7BIT = 2,
	charSize_8BIT = 3,
	charSize_9BIT = 7
}charSize_t;

typedef enum
{
	clkPol_NONINVERTED,
	clkPol_INVERTED	
}clkPol_t;

//////////////////////////////////////////////////////////////////////////

void hal_USART_enableModule();

void hal_USART_dissableModule();

uint8_t hal_USART_readDataReg();

uint16_t hal_USART_readDataReg_9Bit();

void hal_USART_writeDataReg(uint8_t data);

void hal_USART_writeDataReg_9Bit(uint16_t data);

uint8_t hal_USART_checkRxFrameErr();

uint8_t hal_USART_checkRxDataOverRun();

uint8_t hal_USART_checkRxParityErr();

void hal_USART_enableDoubleTxSpeed();

void hal_USART_dissableDoubleTxSpeed();

void hal_USART_enableMultiProcessorComMode();

void hal_USART_dissableMultiProcessorComMode();

void hal_USART_RxEnable();

void hal_USART_RxDissable();

void hal_USART_TxEnable();

void hal_USART_TxDissable();

void hal_USART_setMode(usart_mode_t mode);

void hal_USART_setParityMode(parityMode_t mode);

void hal_USART_setStopBits(stopBit_t bit);

void hal_USART_setCharSize(charSize_t size);

void hal_USART_setClockPolarity(clkPol_t polarity);

void hal_USART_setBaudRate(uint32_t baudRate);

//////////////////////////////////////////////////////////////////////////

void hal_USART_registerRxCallback(fptr_t callback);

void hal_USART_unregisterRXCallback();

void hal_USART_registerDataRegEmptyCallback(fptr_t callback);

void hal_USART_unregisterDataRegEmptCallback();

void hal_USART_registerTxCallback(fptr_t callback);

void hal_USART_unregisterTXCallback();

//////////////////////////////////////////////////////////////////////////

void hal_USART_putc (const char send);

void hal_USART_puts (const char *send);

#endif /* HAL_USART_H_ */