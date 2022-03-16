// ----------------------------------------------------------------------------
/*
 * Copyright (c) 2007 Fabian Greif, Roboterclub Aachen e.V.
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Id: mcp2515_private.h 6910 2008-11-30 21:13:14Z fabian $
 */
// ----------------------------------------------------------------------------

#ifndef	MCP2515_PRIVATE_H
#define	MCP2515_PRIVATE_H

// ----------------------------------------------------------------------------
/**
 * \brief	defines only used inside the library
 *
 * \author	Fabian Greif <fabian.greif@rwth-aachen.de>
 * \version	$Id: mcp2515_private.h 6910 2008-11-30 21:13:14Z fabian $
 *
 * \todo	MCP2515_TXRTSn Pins nutzbar machen.
 */
// ----------------------------------------------------------------------------

#include <inttypes.h>
#include <stdbool.h>
#include "inc/spi/spi.h"

#include "can.h"
#include "utils.h"
#include "can_private.h"


#define MCP2515_CS			bitband_t m_BITBAND_PERIPH(&GPIOB->ODR, 12)

// ----------------------------------------------------------------------------

#ifdef  SUPPORT_FOR_MCP2515__

// ----------------------------------------------------------------------------
// load some default values

#ifndef	MCP2515_CLKOUT_PRESCALER
	#define	MCP2515_CLKOUT_PRESCALER	0
#endif
#ifndef	MCP2515_INTERRUPTS
	#define	MCP2515_INTERRUPTS			(1<<RX1IE)|(1<<RX0IE)
#endif

// ----------------------------------------------------------------------------
// TODO: this file is imcompatible with the at90can
#include "mcp2515_defs.h"

#ifndef	MCP2515_CS
	#error	MCP2515_CS ist nicht definiert!
#endif

#if defined(MCP2515_RX0BF) && !defined(MCP2515_RX1BF)
	#warning	only MCP2515_RX0BF but not MCP2515_RX1BF defined!
#elif !defined(MCP2515_RX0BF) && defined(MCP2515_RX1BF)
	#warning	only MCP2515_RX1BF but not MCP2515_RX0BF defined!
#elif defined(MCP2515_RX0BF) && defined(MCP2515_RX1BF)
	#define	RXnBF_FUNKTION
#endif

// -------------------------------------------------------------------------
/**
 * \brief	Beschreiben von internen Registern
 */
extern void mcp2515_write_register( uint8_t adress, uint8_t data );

// -------------------------------------------------------------------------
/**
 * \brief	Lesen der internen Register
 */
extern uint8_t mcp2515_read_register(uint8_t adress);

extern uint8_t mcp2515_read_status(uint8_t type);

// -------------------------------------------------------------------------
/**
 * \brief	Setzten/loeschen einzelner Bits
 *
 * Diese Funktionen laesst sich nur auf die Register BFPCTRL, 
 * TXRTSCTRL, CANCTRL, CNF1, CNF2, CNF3, CANINTE, CANINTF, EFLG,
 * TXB0CTRL, TXB1CTRL, TXB2CTRL, RXB0CTRL und RXB1CTRL anwenden.
 *
 * \see		Datenblatt des MCP2515, Registerbersichtstabelle
 */
extern void mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data);

// -------------------------------------------------------------------------
extern __attribute__ ((gnu_inline)) inline void mcp2515_change_operation_mode(uint8_t mode)
{
	mcp2515_bit_modify(CANCTRL, 0xe0, mode);
	while ((mcp2515_read_register(CANSTAT) & 0xe0) != (mode & 0xe0))
		;
}

// -------------------------------------------------------------------------
/**
 * \brief	Liest bzw. schreibt eine CAN-Id zum MCP2515
 */
#if	SUPPORT_EXTENDED_CANID

extern void mcp2515_write_id( const uint32_t *id, uint8_t extended );

extern uint8_t mcp2515_read_id( uint32_t *id );

#else

extern void mcp2515_write_id( const uint16_t *id );

extern uint8_t mcp2515_read_id( uint16_t *id );

#endif	// USE_EXTENDED_CANID

#endif  // SUPPORT_FOR_MCP2515__

#endif	// MCP2515_PRIVATE_H
