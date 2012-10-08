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
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "mrbus.h"
#include "mrbee.h"

extern uint8_t mrbus_rx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_tx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_state;
extern uint8_t mrbus_loneliness;

extern uint8_t mrbee_rx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbee_tx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbee_state;

uint8_t dev_addr = 0;
uint16_t pkt_period = 10;

uint8_t powerOnReset = 0;

extern uint8_t mrbee_rssi;
uint8_t rssi_table[256];

void pktHandler(uint8_t *rx_buffer, uint8_t *tx_buffer, uint8_t busy, uint8_t *state, uint8_t tx_mask)
{
	//*************** PACKET HANDLER - PROCESS HERE ***************

	if ('A' == rx_buffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		tx_buffer[MRBUS_PKT_DEST] = rx_buffer[MRBUS_PKT_SRC];
		tx_buffer[MRBUS_PKT_SRC] = dev_addr;
		tx_buffer[MRBUS_PKT_LEN] = 6;
		tx_buffer[MRBUS_PKT_TYPE] = 'a';
		*state |= tx_mask;
	} 
	else if ('W' == rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		tx_buffer[MRBUS_PKT_DEST] = rx_buffer[MRBUS_PKT_SRC];
		tx_buffer[MRBUS_PKT_LEN] = 8;			
		tx_buffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)rx_buffer[6], rx_buffer[7]);
		tx_buffer[6] = rx_buffer[6];
		tx_buffer[7] = rx_buffer[7];
		if (MRBUS_EE_DEVICE_ADDR == rx_buffer[6])
			dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		if ( (MRBUS_EE_DEVICE_UPDATE_L == rx_buffer[6]) || (MRBUS_EE_DEVICE_UPDATE_H == rx_buffer[6]) )
		    pkt_period = ((eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H) << 8) & 0xFF00) | (eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) & 0x00FF);
		tx_buffer[MRBUS_PKT_SRC] = dev_addr;
		*state |= tx_mask;
	}
	else if ('R' == rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		tx_buffer[MRBUS_PKT_DEST] = rx_buffer[MRBUS_PKT_SRC];
		tx_buffer[MRBUS_PKT_SRC] = dev_addr;
		tx_buffer[MRBUS_PKT_LEN] = 8;			
		tx_buffer[MRBUS_PKT_TYPE] = 'r';
		tx_buffer[6] = rx_buffer[6];
		tx_buffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)rx_buffer[6]);			
		*state |= tx_mask;
	}
	else if ( ('V' == rx_buffer[MRBUS_PKT_TYPE]) || (powerOnReset) )
	{
		// Version
		tx_buffer[MRBUS_PKT_DEST] = rx_buffer[MRBUS_PKT_SRC];
		tx_buffer[MRBUS_PKT_SRC] = dev_addr;
		tx_buffer[MRBUS_PKT_LEN] = 14;
		tx_buffer[MRBUS_PKT_TYPE] = 'v';
		tx_buffer[6]  = MRBUS_VERSION_WIRELESS;
		tx_buffer[7]  = 0; // Software Revision
		tx_buffer[8]  = 0; // Software Revision
		tx_buffer[9]  = 0; // Software Revision
		tx_buffer[10]  = 0; // Hardware Major Revision
		tx_buffer[11]  = 0; // Hardware Minor Revision
		tx_buffer[12] = 'A';
		tx_buffer[13] = 'P';
		*state |= tx_mask;
	}
	else if ('X' == rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// Reset
		cli();
		wdt_reset();
		wdt_enable(WDTO_15MS);
		while(1);  // Force a watchdog reset
		sei();
	}
	else if ('C' == rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// Command Packet
		if( ('R' == rx_buffer[6]) && (rx_buffer[MRBUS_PKT_LEN] > 7) )
		{
		    // RSSI Request
    		tx_buffer[MRBUS_PKT_DEST] = rx_buffer[MRBUS_PKT_SRC];
    		tx_buffer[MRBUS_PKT_SRC] = dev_addr;
    		tx_buffer[MRBUS_PKT_LEN] = 9;
    		tx_buffer[MRBUS_PKT_TYPE] = 'c';
    		tx_buffer[6] = 'r';
    		tx_buffer[7] = rx_buffer[7];
    		tx_buffer[8] = rssi_table[rx_buffer[7]];
    		*state |= tx_mask;
		}
	}

	//*************** END PACKET HANDLER  ***************

}


void pktRelay(void)
{
    uint16_t crc = 0;
	uint8_t i;

    // MRBus --> MRBee
	if( (mrbus_state & MRBUS_RX_PKT_READY) && (mrbus_rx_buffer[MRBUS_PKT_SRC] != dev_addr) )
	{
		// MRBUS packet is available and not one we originated
    	if (dev_addr == mrbus_rx_buffer[MRBUS_PKT_DEST])
    	{
    	    // The packet is for us
        	// CRC16 Test - is the packet intact?
        	for(i=0; i<mrbus_rx_buffer[MRBUS_PKT_LEN]; i++)
        	{
        		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
        			crc = mrbusCRC16Update(crc, mrbus_rx_buffer[i]);
        	}
        	if ((UINT16_HIGH_BYTE(crc) == mrbus_rx_buffer[MRBUS_PKT_CRC_H]) && (UINT16_LOW_BYTE(crc) == mrbus_rx_buffer[MRBUS_PKT_CRC_L]))
        	{
                pktHandler(mrbus_rx_buffer, mrbus_tx_buffer, (mrbus_state & (MRBUS_TX_PKT_READY | MRBUS_TX_BUF_ACTIVE)), &mrbus_state, MRBUS_TX_PKT_READY );
            }
    	}
		else if( !(mrbee_state & (MRBEE_TX_PKT_READY | MRBEE_TX_BUF_ACTIVE)) )
		{
			if (mrbus_rx_buffer[MRBUS_PKT_LEN] > MRBEE_BUFFER_SIZE) mrbus_rx_buffer[MRBUS_PKT_LEN] = MRBEE_BUFFER_SIZE;

		    for(i = 0; i < mrbus_rx_buffer[MRBUS_PKT_LEN]; i++)
		    {
		        mrbee_tx_buffer[i] = mrbus_rx_buffer[i];
		    }

            mrbee_state |= MRBEE_TX_PKT_READY;
        }

        // Clear receive flag
        mrbus_state &= (~MRBUS_RX_PKT_READY);
	}
	else
	{
	    // Clear flag in the case of loopback packet
	    mrbus_state &= (~MRBUS_RX_PKT_READY);
	}
	
	// MRBee --> MRBus
	if( (mrbee_state & MRBEE_RX_PKT_READY) && (mrbee_rx_buffer[MRBEE_PKT_SRC] != dev_addr) )
	{
	    // MRBEE packet is available and not one we originated
    	if (dev_addr == mrbee_rx_buffer[MRBEE_PKT_DEST])
    	{
    	    // The packet is for us
        	// CRC16 Test - is the packet intact?
        	for(i=0; i<mrbee_rx_buffer[MRBEE_PKT_LEN]; i++)
        	{
        		if ((i != MRBEE_PKT_CRC_H) && (i != MRBEE_PKT_CRC_L)) 
        			crc = mrbusCRC16Update(crc, mrbee_rx_buffer[i]);
        	}
        	if ((UINT16_HIGH_BYTE(crc) == mrbee_rx_buffer[MRBEE_PKT_CRC_H]) && (UINT16_LOW_BYTE(crc) == mrbee_rx_buffer[MRBEE_PKT_CRC_L]))
        	{
                pktHandler(mrbee_rx_buffer, mrbee_tx_buffer, (mrbee_state & (MRBEE_TX_PKT_READY | MRBEE_TX_BUF_ACTIVE)), &mrbee_state, MRBEE_TX_PKT_READY );
            }
    	}
	    else if( !(mrbus_state & (MRBUS_TX_PKT_READY | MRBUS_TX_BUF_ACTIVE)) )
        {
			if (mrbee_rx_buffer[MRBEE_PKT_LEN] > MRBUS_BUFFER_SIZE) mrbee_rx_buffer[MRBEE_PKT_LEN] = MRBUS_BUFFER_SIZE;

		    for(i = 0; i < mrbee_rx_buffer[MRBEE_PKT_LEN]; i++)
		    {
		        mrbus_tx_buffer[i] = mrbee_rx_buffer[i];
		    }

            mrbus_state |= MRBUS_TX_PKT_READY;
        }

        // Update RSSI table
        rssi_table[mrbee_rx_buffer[MRBEE_PKT_SRC]] = mrbee_rssi;
        
        // Clear receive flag
        mrbee_state &= (~MRBEE_RX_PKT_READY);
	}
	else
	{
	    // Clear flag in the case of loopback packet
	    mrbee_state &= (~MRBEE_RX_PKT_READY);
	}
	
	//*************** RECEIVE CLEANUP ***************
	return;	
}



void init(void)
{
    uint16_t i;
    
    // Clear watchdog (in the case of an 'X' packet reset)
    MCUSR = 0;
	wdt_reset();
	wdt_disable();

	// prescaler is 8, timer counts 1/2 microseconds
//	TCCR0B = _BV(CS02) | _BV(CS00);

	/* prescaler is 1024 for 16bit timer */
//	TCCR1A = 0x00;
//	TCCR1B = 0x00 | _BV(CS12) | _BV(CS10);

    for(i=0; i<256; i++)
    {
        rssi_table[i] = 0xFF;  // Set all values to max -dBm
    }

	// Disable comparator to save power
	ACSR = _BV(ACD);

	// Initialize MRBus address from EEPROM address 1
	dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);

	// Initialize MRBus packet update interval from EEPROM
	pkt_period = ((eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H) << 8) & 0xFF00) | (eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) & 0x00FF);
}




int main(void)
{
	DDRA |= _BV(PA0);
	PORTA &= ~_BV(PA0);

	init();
	mrbusInit();
	mrbeeInit();

	powerOnReset = 1;
	mrbus_rx_buffer[MRBUS_PKT_SRC] = 0xFF;  // Fake the handler into sending a broadcast
    pktHandler(mrbus_rx_buffer, mrbus_tx_buffer, 0, &mrbus_state, MRBUS_TX_PKT_READY );
	mrbee_rx_buffer[MRBEE_PKT_SRC] = 0xFF;  // Fake the handler into sending a broadcast
    pktHandler(mrbee_rx_buffer, mrbee_tx_buffer, 0, &mrbee_state, MRBEE_TX_PKT_READY );
	powerOnReset = 0;

	sei();
	
	while (1)
	{
		mrbeePoll();

		if( (mrbus_state & MRBUS_RX_PKT_READY) || (mrbee_state & MRBEE_RX_PKT_READY) )
			pktRelay();

		// If we have an MRBus packet to be transmitted, try to send it here
		if(mrbee_state & MRBEE_TX_PKT_READY)
		{
		    mrbeePacketTransmit();
            mrbee_state &= ~(MRBEE_TX_PKT_READY);
		}
		
		while(mrbus_state & MRBUS_TX_PKT_READY)
		{
			uint8_t bus_countdown;

			// Even while we're sitting here trying to transmit, keep handling
			// any packets we're receiving so that we keep up with the current state of the
			// bus.  Obviously things that request a response cannot go, since the transmit
			// buffer is full.
			if( (mrbus_state & MRBUS_RX_PKT_READY) || (mrbee_state & MRBEE_RX_PKT_READY) )
				pktRelay();


			if (0 == mrbusPacketTransmit())
			{
				mrbus_state &= ~(MRBUS_TX_PKT_READY);
				break;
			}

			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			bus_countdown = 20;
			while (bus_countdown-- > 0 && MRBUS_ACTIVITY_RX_COMPLETE != mrbus_activity)
			{
				//clrwdt();
				_delay_ms(1);
    			if( (mrbus_state & MRBUS_RX_PKT_READY) || (mrbee_state & MRBEE_RX_PKT_READY) )
        		    pktRelay();
			}
		}

/*
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
*/
	}
}


