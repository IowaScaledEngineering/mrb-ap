/*************************************************************************
Title:    MRBee Access Point - Bridge to MRBus (Version 2)
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License

LICENSE:
    Copyright (C) 2013 Michael Petersen & Nathan Holmes

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
#include <string.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>


#include "mrbus.h"
#include "mrbee.h"

#define QUEUE_DEPTH 8

uint8_t rssi_table[256];

uint32_t mrbusPktCount = 0;
uint32_t mrbeePktCount = 0;
uint32_t mrbusPktLoss  = 0;
uint32_t mrbeePktLoss  = 0;
uint8_t mrbusPktDepthRX = 0;
uint8_t mrbeePktDepthRX = 0;
uint8_t mrbusPktDepthTX = 0;
uint8_t mrbeePktDepthTX = 0;

uint8_t mrbus_countdown;

uint8_t tx_buffer[MRBUS_BUFFER_SIZE];
uint8_t rx_buffer[MRBUS_BUFFER_SIZE];

typedef enum
{
	AP_MRBUS_QUEUE = 0x01,
	AP_MRBEE_QUEUE = 0x02
} APQueue;

typedef struct 
{
	uint8_t pkt[MRBUS_BUFFER_SIZE];
} MRBusPacket;

typedef struct
{
	volatile uint8_t headIdx;
	volatile uint8_t tailIdx;
	volatile uint8_t full;
	volatile MRBusPacket pktData[QUEUE_DEPTH];
} PacketBuffer;

static PacketBuffer mrbus_rxQueue;
static PacketBuffer mrbus_txQueue;
static PacketBuffer mrbee_rxQueue;
static PacketBuffer mrbee_txQueue;

void packetBufferInitialize(PacketBuffer* r)
{
	r->headIdx = r->tailIdx = 0;
	r->full = 0;
}

uint8_t packetBufferDepth(PacketBuffer* r)
{
	if(r->full)
		return(QUEUE_DEPTH);
	return((uint8_t)(r->headIdx - r->tailIdx) % QUEUE_DEPTH);
}

uint8_t packetBufferPush(PacketBuffer* r, uint8_t* data, uint8_t dataLen)
{
	uint8_t* pktPtr;
	// If full, bail with a false
	if (r->full)
		return(0);

	dataLen = min(MRBUS_BUFFER_SIZE, dataLen);
	pktPtr = (uint8_t*)r->pktData[r->headIdx].pkt;
	memcpy(pktPtr, data, dataLen);
	memset(pktPtr+dataLen, 0, MRBUS_BUFFER_SIZE - dataLen);

	if( ++r->headIdx >= QUEUE_DEPTH )
		r->headIdx = 0;
	if (r->headIdx == r->tailIdx)
		r->full = 1;
	return(1);
}

uint8_t packetBufferPop(PacketBuffer* r, uint8_t* data, uint8_t dataLen, uint8_t snoop)
{
	memset(data, 0, dataLen);
	if (0 == packetBufferDepth(r))
		return(0);

	memcpy(data, (uint8_t*)&(r->pktData[r->tailIdx].pkt), min(dataLen, r->pktData[r->tailIdx].pkt[MRBUS_PKT_LEN]));
	if (0 == snoop)
	{
		if( ++r->tailIdx >= QUEUE_DEPTH )
			r->tailIdx = 0;
		r->full = 0;
	}
	return(1);
}

uint8_t packetBufferPopOnly(PacketBuffer* r)
{
	if (0 == packetBufferDepth(r))
		return(0);
	if( ++r->tailIdx >= QUEUE_DEPTH )
		r->tailIdx = 0;
	r->full = 0;
	return(1);
}


#ifdef PKT_HANDLER
uint8_t dev_addr = 0;
uint16_t pkt_period = 10;

void createVersionPacket(uint8_t srcAddr)
{
	tx_buffer[MRBUS_PKT_DEST] = srcAddr;
	tx_buffer[MRBUS_PKT_SRC] = dev_addr;
	tx_buffer[MRBUS_PKT_LEN] = 14;
	tx_buffer[MRBUS_PKT_TYPE] = 'v';
	tx_buffer[6]  = MRBUS_VERSION_WIRELESS;
	// Software Revision
	tx_buffer[7]  = ((uint32_t)SWREV >> 16) & 0xFF;
	tx_buffer[8]  = ((uint32_t)SWREV >> 8) & 0xFF;
	tx_buffer[9]  = (uint32_t)SWREV & 0xFF;
	tx_buffer[10]  = HWREV_MAJOR; // Hardware Major Revision
	tx_buffer[11]  = HWREV_MINOR; // Hardware Minor Revision
	tx_buffer[12] = 'A';
	tx_buffer[13] = 'P';
}

volatile uint16_t busVoltageAccum=0;
volatile uint16_t busVoltage=0;
volatile uint8_t busVoltageCount=0;

ISR(ADC_vect)
{
	busVoltageAccum += ADC;
	if (++busVoltageCount >= 64)
	{
		busVoltageAccum = busVoltageAccum / 64;
        //At this point, we're at (Vbus/3) / 5 * 1024
        //So multiply by 150, divide by 1024, or multiply by 75 and divide by 512
        busVoltage = ((uint32_t)busVoltageAccum * 75) / 512;
		busVoltageAccum = 0;
		busVoltageCount = 0;
	}
}
#endif  // PKT_HANDLER


uint8_t pktHandler(APQueue queue)
{
	switch(queue)
	{
		case AP_MRBUS_QUEUE:
			packetBufferPop(&mrbus_rxQueue, rx_buffer, sizeof(rx_buffer), 0);
			break;
		case AP_MRBEE_QUEUE:
			packetBufferPop(&mrbee_rxQueue, rx_buffer, sizeof(rx_buffer), 0);
			break;
		default:
			return(1);
	}

	// CRC16 Test - is the packet intact?
	uint16_t crc = 0;
	uint8_t i;

	for(i=0; i<rx_buffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, rx_buffer[i]);
	}

	if ((UINT16_HIGH_BYTE(crc) == rx_buffer[MRBUS_PKT_CRC_H]) && (UINT16_LOW_BYTE(crc) == rx_buffer[MRBUS_PKT_CRC_L]))
	{
		// Good packet.  Process.
#ifdef PKT_HANDLER
		if(rx_buffer[MRBUS_PKT_DEST] == dev_addr)
		{
			// Packet is intended for us
			if(rx_buffer[MRBUS_PKT_SRC] != dev_addr)
			{
				// Packet is not one we originated (to prevent rogue packets from creating a possible loop)
				uint8_t pktReady = 0;
			
				if ('A' == rx_buffer[MRBUS_PKT_TYPE])
				{
					// PING packet
					tx_buffer[MRBUS_PKT_DEST] = rx_buffer[MRBUS_PKT_SRC];
					tx_buffer[MRBUS_PKT_SRC] = dev_addr;
					tx_buffer[MRBUS_PKT_LEN] = 6;
					tx_buffer[MRBUS_PKT_TYPE] = 'a';
					pktReady = 1;
				} 
				else if ('W' == rx_buffer[MRBUS_PKT_TYPE]) 
				{
					// EEPROM WRITE Packet
					eeprom_write_byte((uint8_t*)(uint16_t)rx_buffer[6], rx_buffer[7]);
					if (MRBUS_EE_DEVICE_ADDR == rx_buffer[6])
						dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
					if ( (MRBUS_EE_DEVICE_UPDATE_L == rx_buffer[6]) || (MRBUS_EE_DEVICE_UPDATE_H == rx_buffer[6]) )
						pkt_period = ((eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H) << 8) & 0xFF00) | (eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) & 0x00FF);
					tx_buffer[MRBUS_PKT_DEST] = rx_buffer[MRBUS_PKT_SRC];
					tx_buffer[MRBUS_PKT_SRC] = dev_addr;
					tx_buffer[MRBUS_PKT_LEN] = 8;			
					tx_buffer[MRBUS_PKT_TYPE] = 'w';
					tx_buffer[6] = rx_buffer[6];
					tx_buffer[7] = rx_buffer[7];
					pktReady = 1;
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
					pktReady = 1;
				}
				else if ('V' == rx_buffer[MRBUS_PKT_TYPE])
				{
					// Version
					createVersionPacket(rx_buffer[MRBUS_PKT_SRC]);
					pktReady = 1;
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
						pktReady = 1;
					}
					else if( ('X' == rx_buffer[6]) && ('R' == rx_buffer[7]) && (rx_buffer[MRBUS_PKT_LEN] > 7) )
					{
						// RSSI Reset
						uint16_t i;
						for(i=0; i<256; i++)
						{
							rssi_table[i] = 0xFF;  // Set all values to max -dBm
						}
						tx_buffer[MRBUS_PKT_DEST] = rx_buffer[MRBUS_PKT_SRC];
						tx_buffer[MRBUS_PKT_SRC] = dev_addr;
						tx_buffer[MRBUS_PKT_LEN] = 8;
						tx_buffer[MRBUS_PKT_TYPE] = 'c';
						tx_buffer[6] = 'x';
						tx_buffer[7] = 'r';
						pktReady = 1;
					}
					else if( ('X' == rx_buffer[6]) && ('P' == rx_buffer[7]) && (rx_buffer[MRBUS_PKT_LEN] > 7) )
					{
						// Packet Counter Reset
						mrbusPktCount = 0;
						mrbeePktCount = 0;
						mrbusPktLoss  = 0;
						mrbeePktLoss  = 0;
						mrbusPktDepthRX = 0;
						mrbeePktDepthRX = 0;
						mrbusPktDepthTX = 0;
						mrbeePktDepthTX = 0;
						tx_buffer[MRBUS_PKT_DEST] = rx_buffer[MRBUS_PKT_SRC];
						tx_buffer[MRBUS_PKT_SRC] = dev_addr;
						tx_buffer[MRBUS_PKT_LEN] = 8;
						tx_buffer[MRBUS_PKT_TYPE] = 'c';
						tx_buffer[6] = 'x';
						tx_buffer[7] = 'p';
						pktReady = 1;
					}
				}
				if(pktReady)
				{
					// Queue response packet on same interface
					switch(queue)
					{
						case AP_MRBUS_QUEUE:
							// MRBus --> MRBus
							if(!packetBufferPush(&mrbus_txQueue, tx_buffer, sizeof(tx_buffer)))
							break;
						case AP_MRBEE_QUEUE:
							// MRBee --> MRBee
							if(!packetBufferPush(&mrbee_txQueue, tx_buffer, sizeof(tx_buffer)))
							break;
						default:
							return(1);
					}
				}
			}
		}
		else
		{
#endif  // PKT_HANDLER
			// Packet is not intended for us (or no packet handler defined), relay it to the other interface
			switch(queue)
			{
				case AP_MRBUS_QUEUE:
					// MRBus --> MRBee
					packetBufferPush(&mrbee_txQueue, rx_buffer, sizeof(rx_buffer));
					break;
				case AP_MRBEE_QUEUE:
					// MRBee --> MRBus
					packetBufferPush(&mrbus_txQueue, rx_buffer, sizeof(rx_buffer));
					break;
				default:
					return(1);
			}
#ifdef PKT_HANDLER
		}
	}
#endif  // PKT_HANDLER
	return(0);
}





// ******** Start 1kHz Timer (Timer 0)
// If you can live with a slightly less accurate timer, this one only uses Timer 0, leaving Timer 1 open
// for more advanced things that actually need a 16 bit timer/counter

// Initialize a 1kHz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint8_t ticks;
volatile uint16_t decisecs;

void initialize1kHzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 78;  // 20MHz / 256 / 78 = 1.0016 kHz
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02);   // Divide by 256
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	if (++ticks >= 100)
	{
		ticks = 0;
		decisecs++;
	}
	if(mrbus_countdown)
	{
		mrbus_countdown--;
	}
}

// End of 1kHz timer




void init(void)
{
    uint16_t i;
    
    // Clear watchdog (in the case of an 'X' packet reset)
    MCUSR = 0;
	wdt_reset();
	wdt_disable();

	packetBufferInitialize(&mrbus_rxQueue);
	packetBufferInitialize(&mrbus_txQueue);
	packetBufferInitialize(&mrbee_rxQueue);
	packetBufferInitialize(&mrbee_txQueue);
	mrbusPktCount = 0;
	mrbeePktCount = 0;
	mrbusPktLoss  = 0;
	mrbeePktLoss  = 0;
	mrbusPktDepthRX = 0;
	mrbeePktDepthRX = 0;
	mrbusPktDepthTX = 0;
	mrbeePktDepthTX = 0;

    for(i=0; i<256; i++)
    {
        rssi_table[i] = 0xFF;  // Set all values to max -dBm
    }

	// Disable comparator to save power
	ACSR = _BV(ACD);

#ifdef PKT_HANDLER
	// Initialize MRBus address from EEPROM address 1
	dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);

	// Initialize MRBus packet update interval from EEPROM
	pkt_period = ((eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H) << 8) & 0xFF00) | (eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) & 0x00FF);

	// Setup ADC
	ADMUX  = 0x40;  // AVCC reference; ADC0 input
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 64 prescaler
	ADCSRB = 0x00;
	DIDR0  = _BV(ADC0D);

    busVoltage = 0;
    busVoltageAccum = 0;
    busVoltageCount = 0;
    ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
#endif  // PKT_HANDLER
}




int main(void)
{
	uint8_t pktDepth;
	
	init();

	mrbusInit();
	mrbeeInit();

	initialize1kHzTimer();
	
#ifdef PKT_HANDLER
	createVersionPacket(0xFF);
	packetBufferPush(&mrbus_txQueue, tx_buffer, sizeof(tx_buffer));
	packetBufferPush(&mrbee_txQueue, tx_buffer, sizeof(tx_buffer));
#endif  // PKT_HANDLER

	sei();
	
	while (1)
	{
#ifdef PKT_HANDLER
        if (decisecs >= pkt_period)
        {
			tx_buffer[MRBUS_PKT_SRC] = dev_addr;
			tx_buffer[MRBUS_PKT_DEST] = 0xFF;
			tx_buffer[MRBUS_PKT_LEN] = 20;
			tx_buffer[5] = 'S';

			tx_buffer[6]  = (uint8_t)((mrbusPktCount >> 24) & 0xFF);
			tx_buffer[7]  = (uint8_t)((mrbusPktCount >> 16) & 0xFF);
			tx_buffer[8]  = (uint8_t)((mrbusPktCount >>  8) & 0xFF);
			tx_buffer[9]  = (uint8_t)(mrbusPktCount & 0xFF);

			tx_buffer[10] = (uint8_t)((mrbeePktCount >> 24) & 0xFF);
			tx_buffer[11] = (uint8_t)((mrbeePktCount >> 16) & 0xFF);
			tx_buffer[12] = (uint8_t)((mrbeePktCount >>  8) & 0xFF);
			tx_buffer[13] = (uint8_t)(mrbeePktCount & 0xFF);
			
			tx_buffer[14] = (uint8_t)mrbusPktDepthRX;
			tx_buffer[15] = (uint8_t)mrbeePktDepthRX;
			tx_buffer[16] = (uint8_t)mrbusPktDepthTX;
			tx_buffer[17] = (uint8_t)mrbeePktDepthTX;

			tx_buffer[18] = 0;

			tx_buffer[19] = (uint8_t)busVoltage;

			packetBufferPush(&mrbus_txQueue, tx_buffer, sizeof(tx_buffer));
			packetBufferPush(&mrbee_txQueue, tx_buffer, sizeof(tx_buffer));

			decisecs = 0;
        }
#endif  // PKT_HANDLER

		mrbeePoll();

		// Handle any incoming packets
		if(MRBUS_RX_PKT_READY & mrbus_state)
		{
			if(!packetBufferPush(&mrbus_rxQueue, (uint8_t *)mrbus_rx_buffer, sizeof(mrbus_rx_buffer)))
				mrbusPktLoss++;
			mrbus_state &= (~MRBUS_RX_PKT_READY);
		}

		if(MRBEE_RX_PKT_READY & mrbee_state)
		{
			if(!packetBufferPush(&mrbee_rxQueue, (uint8_t *)mrbee_rx_buffer, sizeof(mrbee_rx_buffer)))
				mrbeePktLoss++;
			rssi_table[mrbee_rx_buffer[MRBEE_PKT_SRC]] = mrbee_rssi;  // Update RSSI table
			mrbee_state &= (~MRBEE_RX_PKT_READY);
		}

		// Handle any queued packets
		if((pktDepth = packetBufferDepth(&mrbus_rxQueue)))
		{
			if(pktDepth > mrbusPktDepthRX)
				mrbusPktDepthRX = pktDepth;
			pktHandler(AP_MRBUS_QUEUE);
		}

		if((pktDepth = packetBufferDepth(&mrbee_rxQueue)))
		{
			if(pktDepth > mrbeePktDepthRX)
				mrbeePktDepthRX = pktDepth;
			pktHandler(AP_MRBEE_QUEUE);
		}

		// If we have an MRBee packet to be transmitted, try to send it here
		if((pktDepth = packetBufferDepth(&mrbee_txQueue)))
		{
			if(pktDepth > mrbeePktDepthTX)
				mrbeePktDepthTX = pktDepth;

			if(!(mrbee_state & MRBEE_TX_BUF_ACTIVE))
			{
				packetBufferPop(&mrbee_txQueue, (uint8_t *)mrbee_tx_buffer, sizeof(mrbee_tx_buffer), 0);
				mrbeePacketTransmit();
				mrbeePktCount++;
			}
		}

		// If we have an MRBus packet to be transmitted, try to send it here
		if((pktDepth = packetBufferDepth(&mrbus_txQueue)))
		{
			if(pktDepth > mrbusPktDepthTX)
				mrbusPktDepthTX = pktDepth;

			// But only if not waiting for mrbus_countdown or until a pending rx is complete
			if((0 == mrbus_countdown) || (MRBUS_ACTIVITY_RX_COMPLETE == mrbus_activity))
			{
				// And then only if no transmission is already in progress
				if(!(mrbus_state & MRBUS_TX_BUF_ACTIVE))
				{
					packetBufferPop(&mrbus_txQueue, (uint8_t *)mrbus_tx_buffer, sizeof(mrbus_tx_buffer), 1);  // Get the data but don't remove it yet in case we lose arbitration
					if (0 == mrbusPacketTransmit())
					{
						// Success!
						packetBufferPopOnly(&mrbus_txQueue);
						mrbusPktCount++;
					}
					else
					{
						// If we're here, we failed to start transmission due to somebody else transmitting
						// We want to wait 20ms before we try a retransmit
						mrbus_countdown = 20;
					}
				}
			}
		}
	}
}


