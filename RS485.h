/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2019 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * -------------------------------------------------------------------------------
 *
 * Copyright (c) 2013, Majenko Technologies and S.J.Hoeksma
 * Copyright (c) 2015, LeoDesigner
 * https://github.com/leodesigner/mysensors-serial-transport
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of Majenko Technologies.
 ********************************************************************************/

#ifndef RS485_H
#define RS485_H

// Serial Transport
#include <util/delay.h>
#include <avr/io.h>
#include <util/setbaud.h>

#if defined(MY_RS485_DE_PIN)
#if !defined(MY_RS485_DE_INVERSE)
#define assertDE() MY_RS485_DE_PORT |= _BV(MY_RS485_DE_PIN)
#define deassertDE() MY_RS485_DE_PORT &= ~_BV(MY_RS485_DE_PIN)
#else
#define assertDE() MY_RS485_DE_PORT &= ~_BV(MY_RS485_DE_PIN)
#define deassertDE() MY_RS485_DE_PORT |= _BV(MY_RS485_DE_PIN)
#endif
#else
#define assertDE()
#define deassertDE()
#endif

// We only use SYS_PACK in this application
//#define MY_RS485_SOH_COUNT (1) // No need for wakeup calls
#define MY_RS485_MAX_MESSAGE_LENGTH MAX_MESSAGE_LENGTH
#define BROADCAST_ADDRESS 255

extern nodeConfig_t _eepromNodeConfig;

#define _dataAvailable() transportDataAvailable()

// Receiving header information
#define RS485_HEADER_LENGTH  3
char _header[RS485_HEADER_LENGTH];

// Reception state machine control and storage variables
unsigned char _recPhase;
unsigned char _recPos;
unsigned char _recLen;
unsigned char _recStation;
unsigned char _recSender;
unsigned char _recCS;
unsigned char _recCalcCS;

char _data[MY_RS485_MAX_MESSAGE_LENGTH];
uint8_t _packet_len;
unsigned char _packet_from;
bool _packet_received;

// Packet wrapping characters, defined in standard ASCII table
#define NEW_SOH 0x11
#define STX 2
#define ETX 3
#define EOT 4

#define uart_putc(x) putch(x)
#define uart_getc getch

//Reset the state machine and release the data pointer
void _serialReset()
{
	_recPhase = 0;
	_recPos = 0;
	_recLen = 0;
	_recCS = 0;
	_recCalcCS = 0;
}

// This is the main reception state machine.  Progress through the states
// is keyed on either special control characters, or counted number of bytes
// received.  If all the data is in the right format, and the calculated
// checksum matches the received checksum, AND the destination station is
// our station ID, then look for a registered command that matches the
// command code.  If all the above is true, execute the command's
// function.
bool _serialProcess()
{
	if (!(UCSR0A & (1 << RXC0))) // input buffer empty
	{
		return false;
	}
	while ((UCSR0A & (1 << RXC0))) // input buffer not empty
	{
		char inch;
		inch = uart_getc();
		switch (_recPhase){
		// Case 0 looks for the header.  Bytes arrive in the serial interface and get
		// shifted through a header buffer.  When the start and end characters in
		// the buffer match the SOH/STX pair, and the destination station ID matches
		// our ID, save the header information and progress to the next state.
		case 0:
			memcpy(&_header[0], &_header[1], RS485_HEADER_LENGTH-1);
			_header[2] = inch;
			if ((_header[0] == NEW_SOH) && (_header[2] == STX)) {
				_recLen = _header[1];
				_recCalcCS = 0;
				_recCalcCS += _recLen;
				_recPhase = 1;
				_recPos = 0;

				//Avoid _data[] overflow
				if (_recLen >= MY_RS485_MAX_MESSAGE_LENGTH) {
					_serialReset();
					break;
				}
				
				if (_recLen == 0) {
					_recPhase = 2;
				}
			}
			break;

		// Case 1 receives the data portion of the packet.  Read in "_recLen" number
		// of bytes and store them in the _data array.
		case 1:
			_data[_recPos++] = inch;
			_recCalcCS += inch;
			if (_recPos == _recLen)
			{
				_recPhase = 2;
			}
			break;

		// After the data comes a single ETX character.  Do we have it?  If not,
		// reset the state machine to default and start looking for a new header.
		case 2:
			// Packet properly terminated?
			if (inch == ETX)
			{
				_recPhase = 3;
			}
			else
			{
				_serialReset();
			}
			break;

		// Next comes the checksum.  We have already calculated it from the incoming
		// data, so just store the incoming checksum byte for later.
		case 3:
			_recCS = inch;
			_recPhase = 4;
			break;

		// The final state - check the last character is EOT and that the checksum matches.
		// If that test passes, then look for a valid command callback to execute.
		// Execute it if found.
		case 4:
			if (inch == EOT)
			{
				if (_recCS == _recCalcCS)
				{
					_packet_len = _recLen;
					//Check if we should process this message
					//We reject the message if we are the sender
					//Message not surpressed if node ID was not assigned to support auto id
					if ((_data[0] == _eepromNodeConfig.nodeId) && (_eepromNodeConfig.nodeId != 0xFF)) {
						_serialReset();
						break;
					}
					_packet_received = true;
				}
			}
			//Clear the data
			_serialReset();
			//Return true, we have processed one command
			return true;
			break;
		}
	}
	return true;
}

bool writeMessage(const uint8_t to, const void *data, const uint8_t len)
{
	//const char *datap = static_cast<char const *>(data);
	char *datap = (char *)data;
	unsigned char i;
	unsigned char cs = 0;

	// This is how many times to try and transmit before failing.
	unsigned char timeout = 50;

	// Let's start out by looking for a collision.  If there has been anything seen in
	// the last millisecond, then wait for a random time and check again.

	while (_serialProcess())
	{
		//	unsigned char del;
		//	del = rand() % 20;
		_delay_us((1/ BAUD_RATE) * 10 * 5 * 1000 * 1000); // wait for ~ 5 bytes
		//		for (i = 0; i < del; i++) {
		//			_delay_us(1);
		//			_serialProcess();
		//		}
		timeout--; 
		if (timeout == 0)
		{
			// Failed to transmit!!!
			return false;
		}
	}

	assertDE();

	// Start of header by writing  SOH
	uart_putc(NEW_SOH);
	uart_putc(len); // Length of text
	cs += len;
	uart_putc(STX); // Start of text
	for (i = 0; i < len; i++)
	{
		uart_putc(datap[i]); // Text bytes
		cs += datap[i];
	}
	uart_putc(ETX); // End of text
	uart_putc(cs);
	uart_putc(EOT);
	UCSR0A |= 1<<TXC0;  // clear flag!
	while((UCSR0A & _BV(TXC0)) == 0); //wait for transission complete
	deassertDE();
	return true;
}

bool initRadio(void)
{
	_serialReset();
	MY_RS485_DE_DDR |= _BV(MY_RS485_DE_PIN);
	deassertDE();
	return true;
}

bool transportDataAvailable(void)
{
	_serialProcess();
	return _packet_received;
}

uint8_t readMessage(void *data)
{
	memcpy(data, _data, _packet_len);
	_packet_received = false;
	return _packet_len;
}
#endif //RS485_H