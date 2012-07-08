/*************************************************************************
Title:    MRBee Access Point - Bridge to MRBus
Authors:  Michael Petersen <railfan@drgw.net>
File:     $Id: $
License:  GNU General Public License

LICENSE:
    Copyright (C) 2011 Michael Petersen

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "mrbus.h"
#include "mrbee.h"

void init(void);

extern uint8_t mrbus_rx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_tx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_state;
extern uint8_t mrbus_loneliness;

extern uint8_t mrbee_rx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbee_tx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbee_state;


int main(void)
{
	uint8_t count, i;

	DDRA |= _BV(PA0);
	PORTA &= ~_BV(PA0);

	init();
	mrbusInit();
	mrbeeInit();

	sei();

	while (1)
	{
		mrbeePoll();

		if(mrbus_state & MRBUS_RX_PKT_READY)
		{
			// MRBUS packet is available
			if( !(mrbee_state & MRBEE_TX_BUF_ACTIVE) )
			{
				if (mrbus_rx_buffer[MRBUS_PKT_LEN] > MRBEE_BUFFER_SIZE) mrbus_rx_buffer[MRBUS_PKT_LEN] = MRBEE_BUFFER_SIZE;

			    for(i = 0; i < mrbus_rx_buffer[MRBUS_PKT_LEN]; i++)
			    {
			        mrbee_tx_buffer[i] = mrbus_rx_buffer[i];
			    }

			    mrbeePacketTransmit();
			    mrbus_state &= ~MRBUS_RX_PKT_READY;
            }
		}
		
		if(mrbee_state & MRBEE_RX_PKT_READY)
		{
		    // MRBEE packet is available
		    if( !(mrbus_state & (MRBUS_TX_PKT_READY | MRBUS_TX_BUF_ACTIVE)) )
            {
				if (mrbee_rx_buffer[MRBEE_PKT_LEN] > MRBUS_BUFFER_SIZE) mrbee_rx_buffer[MRBEE_PKT_LEN] = MRBUS_BUFFER_SIZE;

			    for(i = 0; i < mrbee_rx_buffer[MRBEE_PKT_LEN]; i++)
			    {
			        mrbus_tx_buffer[i] = mrbee_rx_buffer[i];
			    }

                mrbus_state |= MRBUS_TX_PKT_READY;
                mrbee_state &= ~MRBEE_RX_PKT_READY;
            }
		}

		if(mrbus_state & MRBUS_TX_PKT_READY)
		{
			if (mrbusPacketTransmit())
			{
				//  Packet transmit failed; Wait prescribed 10ms before trying again
				for (count = 0; count < 10; count++)
				{
					_delay_ms(1);
				}
			}
		}
	}
}



void init(void)
{
	// prescaler is 8, timer counts 1/2 microseconds
	TCCR0B = _BV(CS02) | _BV(CS00);

	/* prescaler is 1024 for 16bit timer */
	TCCR1A = 0x00;
	TCCR1B = 0x00 | _BV(CS12) | _BV(CS10);
	//    TIMSK1 = 0x00 | _BV(TOIE1);

	//    GIMSK=0x00;
	ACSR = _BV(ACD);
}
