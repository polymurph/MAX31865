/*
 * hal_timer0.c
 *
 * Created: 28-Oct-18 14:25:44
 *  Author: Edwin
 */ 

#include <stdint.h>
#include <avr/interrupt.h>
#include "hal_timer0.h"
#include "dummy.h"
//////////////////////////////////////////////////////////////////////////
// flags
volatile uint8_t t0_overflowEnbaleRoutine_FLG = 0; /**< Overflow callback routine enable flag */
volatile uint8_t t0_compareMatchEnableRoutineA_FLG = 0; /**< Compare match callback routine A enable flag. */
volatile uint8_t t0_compareMatchEnableRoutineB_FLG = 0; /**< Compare match callback routine B enable flag. */

// callbacks are initialized by default to a dummy function. This is safer than setting them to 0 which in some cases can have unexpected results
// overflow callback routine
volatile fptr_t t0_overflowCallBack = dummy; /**< Overflow callback routine*/
// compare match callback routines
volatile fptr_t t0_compareMatchACallBack = dummy; /**< Compare match callback routine A*/
volatile fptr_t t0_compareMatchBCallBack = dummy; /**< Compare match callback routine B*/

//////////////////////////////////////////////////////////////////////////
void hal_timer0_enableModule()
{
	PRR &= ~(1 << PRTIM0);
}

void hal_timer0_dissableModule()
{
	PRR |= (1 << PRTIM0);
}

void hal_timer0_setMode(t0_mode_t mode)
{
	TCCR0A &= ~0x03;
	TCCR0B &= ~0x08;
	
	TCCR0A |= mode & 0x03;
	TCCR0B |= mode & 0x08;
}

void hal_timer0_setCompareValueA(uint8_t value)
{
	OCR0A = value;
}

void hal_timer0_setCompareValueB(uint8_t value)
{
	OCR0B = value;
}

void hal_timer0_setOutputA(t0_outA_Mode_t mode)
{
	TCCR0A &= 0x3F;
	TCCR0A |= mode;
	if(mode != t0_outA_DISCONECTED) DDRD |= (1 << DDD6); // PD6 is now output A
}

void hal_timer0_setOutputB(t0_outB_Mode_t mode)
{
	TCCR0A &= 0xCF;
	TCCR0A |= mode;
	if(mode != t0_outB_DISCONECTED) DDRD |= (1 << DDD5); // PD5 is now output B
}

void hal_timer0_setClockSource(t0_clk_src_t source)
{
	TCCR0B &= 0x8F;
	TCCR0B |= source;
	if(source == (t0_clk_CLK_EXT_PIN_T0_FALLING_EDEGE | t0_clk_CLK_EXT_PIN_T0_FALLING_EDEGE))
	{
		// set pin T0 as input (PIND4 high impedance) for external clock source
		DDRD &= ~(1 << PIND4);
		PORTD &= ~(1 << PIND4);
	}
}
//////////////////////////////////////////////////////////////////////////
// interrupt related functions
// overflow callback setup functions
void hal_timer0_overflow_unregisterCB()
{
	t0_overflowEnbaleRoutine_FLG = 0;
	t0_overflowCallBack = dummy;
	TIMSK0 &= ~(1 << TOIE0);
}

void hal_timer0_overflow_registerCB(fptr_t CBroutine)
{
	t0_overflowCallBack = CBroutine;
	TIMSK0 |= (1 << TOIE0);
}

void hal_timer0_overflow_enableCBroutine()
{
	t0_overflowEnbaleRoutine_FLG = 1;
}

void hal_timer0_overflow_dissableCBroutine()
{
	t0_overflowEnbaleRoutine_FLG = 0;
}

// compare match with A callback setup functions
void hal_timer0_compareMatch_unregisterCBA()
{
	t0_compareMatchEnableRoutineA_FLG = 0;
	t0_compareMatchACallBack = dummy;
	TIMSK0 &= ~(1 << OCIE0A);
}

void hal_timer0_compareMatch_registerCBA(fptr_t CBroutineA)
{
	t0_compareMatchACallBack = CBroutineA;
	TIMSK0 |= (1 << OCIE0A);
}

void hal_timer0_compreMatch_enableCBroutineA()
{
	t0_compareMatchEnableRoutineA_FLG = 1;
}

void hal_timer0_compreMatch_dissableCBroutineA()
{
	t0_compareMatchEnableRoutineA_FLG = 0;
}

// compare match with B callback setup functions
void hal_timer0_compareMatch_unregisterCBB()
{
	t0_compareMatchEnableRoutineB_FLG = 0;
	t0_compareMatchBCallBack = dummy;
	TIMSK0 &= ~(1 << OCIE0B);
}

void hal_timer0_compareMatch_registerCBB(fptr_t CBroutineB)
{
	t0_compareMatchBCallBack = CBroutineB;
	TIMSK0 |= (1 << OCIE0B);
}

void hal_timer0_compreMatch_enableCBroutineB()
{
	t0_compareMatchEnableRoutineB_FLG = 1;
}

void hal_timer0_compreMatch_dissableCBroutineB()
{
	t0_compareMatchEnableRoutineB_FLG = 0;
}

// compare match ISR for COMPA
ISR(TIMER0_COMPA_vect)
{
	if(t0_compareMatchEnableRoutineA_FLG) t0_compareMatchACallBack();
}

// compare match ISR for COMPB
ISR(TIMER0_COMPB_vect)
{
	if(t0_compareMatchEnableRoutineB_FLG) t0_compareMatchBCallBack();
}

// overflow match
ISR(TIMER0_OVF_vect)
{
	if(t0_overflowEnbaleRoutine_FLG) t0_overflowCallBack();
}

