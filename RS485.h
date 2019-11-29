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


// CAN Transceiver related stuff
// it is possible to use any digial I/O (or the USART)
#define CAN_RX_PIN PD0  // standard UART port for atmega328
#define CAN_TX_PIN PD1  // standard UART port for atmega328


#if (F_CPU / BAUD_RATE) < 256   // check maybe required prescaling
#warning "Using no prescaler for bit collision detection timer!"
#define TCNT2_VAL_PER_BIT (F_CPU / BAUD_RATE )
#else
#if ((F_CPU / BAUD_RATE / 8) < 256 ) // 8x prescaling required
#warning "Using x8 for bit collision detection timer!"
#define USE_PRESCALER_8X
#define TCNT2_VAL_PER_BIT (F_CPU / BAUD_RATE / 8)
#endif
#endif

// double check if all conditions above have been analyzed
#if !defined(TCNT2_VAL_PER_BIT)
#error " TCNT2_VAL_PER_BIT is undefined! "
#endif


uint8_t canTcnt2ValBitStart;  // TCNT2 counter value at the beginning of a bit transaction
typedef enum {
    UART_START_VAL = 0,
    UART_STOP_VAL = 1
} uart_cmd_t;

typedef enum {
    CAN_DOMINANT_LEVEL = 0,
    CAN_RECESSIVE_LEVEL = 1
} can_level_t;

bool putBitReadback(bool b)
{
// check the current state of the CAN bus
// if a dominant level is set on the CAN bus check if this node caused this in a previous transaction
    uint8_t rxVal;
    rxVal = (PIND & _BV(CAN_RX_PIN)) >> CAN_RX_PIN;

    uint8_t txVal;
    txVal = (PIND & _BV(CAN_TX_PIN)) >> CAN_TX_PIN;

    if (rxVal == CAN_DOMINANT_LEVEL && txVal != CAN_DOMINANT_LEVEL)
        return false; // some other node is sending a dominant bit


    // bus seems idle... write bit value
    if (b)
    {   // 1 is the CAN recessive state
        PORTD |= _BV(CAN_TX_PIN);
    }
    else
    {
        // 0 is CAN dominant state
        PORTD &= ~_BV(CAN_TX_PIN);
    }

	// NOTE
    // The measured delay between TX Pin and RX Pin echo (through CAN Transceiver) is about 150 ns.
    // One clock cycle at 8 MHz is 150 ns.
    // --> The rx signal should 'immediately' be stable after TX pin has been set
	// Better be on the safe side and insert one NOP
	asm("NOP");

    // ensure that the bit is set for 1/BAUD_RATE time
    while ((uint8_t)(TCNT2 - canTcnt2ValBitStart) < TCNT2_VAL_PER_BIT)
    {
        // check the output while waiting.
        // Do collisions occur, while a logical 1 (CAN recessive bit) is being transmitted?
        // Note: there is actually no need to check, if a logical zero (CAN dominant bit) is driven
        // but it does not hurt... therefore we skip the if-condition and save some bytes for the booloader
        rxVal = (PIND & _BV(CAN_RX_PIN)) >> CAN_RX_PIN;
        if (b != (bool) rxVal)
            return false;
    }

    // increase start value for next bit to transfer
    canTcnt2ValBitStart += TCNT2_VAL_PER_BIT;

    return true;
}



bool putchReadback(uint8_t val)
{
    UART_SRB = 0; // disable USART interface

// TODO: is this BS with extra CAN RX/TX defines?
    DDRD &= ~_BV(CAN_RX_PIN); // configure RX as input

    PORTD |= _BV(CAN_TX_PIN); // 1 is the CAN recessive state
    DDRD |= _BV(CAN_TX_PIN);  // configure TX as output


    canTcnt2ValBitStart = TCNT2; // set variable to determine next bit time window

    // send Start Bit
    if (!putBitReadback(UART_START_VAL))
    {
        UART_SRB = _BV(RXEN0) | _BV(TXEN0);  // re-enable USART
        return false;
    }


    // send payload
    for (uint8_t i = 0; i < 8; ++i)
    {
        if (!putBitReadback((val & (1 << i)) >> i))
        {
            UART_SRB = _BV(RXEN0) | _BV(TXEN0);  // re-enable USART
            return false;
        }
    }

    // send Stop Bit
    if (!putBitReadback(UART_STOP_VAL))
    {
        UART_SRB = _BV(RXEN0) | _BV(TXEN0);  // re-enable USART
        return false;
    }


    UART_SRB = _BV(RXEN0) | _BV(TXEN0);   // re-enable USART
    return true;
}


#ifdef RS485_COLLISION_DETECTION
#define uart_putc(x) putchReadback(x)
#else

inline bool uart_putc(const uint8_t ch) {
    putch(ch);
    return true;
}
#endif


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
        switch (_recPhase) {
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

// TODO: store stuff into uint16_t to save space?
bool writeRS485Packet(const void *data, const uint8_t len)
{
    unsigned char cs = 0;
    char *datap = (char *)data;

    // Start of header by writing SOH
    if(!uart_putc(NEW_SOH))
        return false;

    if(!uart_putc(len)) // Length of text
        return false;

    cs += len;
    if(!uart_putc(STX)) //Start of text
        return false;

    for (uint8_t i = 0; i < len; i++)
    {
        if(!uart_putc(datap[i])) // Text bytes
            return false;
        cs += datap[i];
    }

    if(!uart_putc(ETX)) // // End of text
        return false;


    if(!uart_putc(cs)) // checksum
        return false;


    if(!uart_putc(EOT)) // End of transfer
        return false;


// flush HW USART
#if !defined(RS485_COLLISION_DETECTION)
    UCSR0A |= 1<<TXC0;  // clear flag!
    while((UCSR0A & _BV(TXC0)) == 0); //wait for transission complete
#endif

    return true;
}



#define RS485_SEND_MESSAGE_TRY_CNT 10
#define RS485_BUS_AQUISITION_TRY_CNT 50
#define RS485_TRANSMIT_TRY_CNT 50

#define RS485_BIT_DURATION_US (1.f/ BAUD_RATE) *1000 *1000
#define RS485_BUS_AQUISITION_WAIT_US 5* RS485_BIT_DURATION_US * 10   // ~ 5 Bytes * (Bit period * 10 Bit / transaction (start/stop+8bit)) 

// example
// RS485_BUS_AQUISITION_WAIT_US for BAUD 38400 = 1,3 ms
// 10*50*50 * 1,3 ms = 32s

// TODO Will the watchdog trigger before?

bool writeMessage(const uint8_t to, const void *data, const uint8_t len)
{
    (void) to;  // unused in RS485

    // LED_PORT &= ~_BV(LED_PIN); // disable LED
    // LED_PORT |= _BV(LED_PIN); // enable LED

// step 0) repeat for RS485_SEND_MESSAGE_TRY_CNT
// step 1) Listen before talk: Wait RS485_BUS_AQUISITION_TRY_CNT times until bus showed no activity for a certain time (RS485_BUS_AQUISITION_WAIT_US)
// step 2) Try to transmit message for RS485_TRANSMIT_TRY_CNT times

    uint8_t sendMessageCnt = RS485_TRANSMIT_TRY_CNT;

    // step 0 repeat
    while (sendMessageCnt > 0)
    {
        // step 1: wait until bus idle
        uint8_t busAquisitionCnt = RS485_BUS_AQUISITION_TRY_CNT;

        while (busAquisitionCnt > 0)
        {
            if (_serialProcess())
            {
                // bus activity detected ... wait and try again
                _delay_us(RS485_BUS_AQUISITION_WAIT_US);
            }
            else {
                // bus seems idle
                // try to send bitwise for RS485_TRANSMIT_TRY_CNT times

                assertDE();
                bool ret = writeRS485Packet(data, len);
                deassertDE();
                if ( ret )
                {
                    // message has been successfully sent :)

                    LED_PORT &= ~_BV(LED_PIN); // disable LED
                    return true;
                }

                // bit transmission failed. Some other node is sending...
                // we need to wait here until some characters . If not all nodes would just try to fire
                _delay_us(RS485_BUS_AQUISITION_WAIT_US);
            }

            --busAquisitionCnt;
        } // while (busAquisitionCnt > 0)

        --sendMessageCnt;
    } // while (sendMessageCnt > 0)

    return false;
}

bool initRadio(void)
{
    _serialReset();
    MY_RS485_DE_DDR |= _BV(MY_RS485_DE_PIN);
    deassertDE();

#ifdef RS485_COLLISION_DETECTION
    // activate timer CNT2 to send bits in equidistant time slices

    PRR &= _BV(PRTIM2); // ensure that Timer2 is enabled in PRR (Power Reduction Register)

#ifdef USE_PRESCALER_8X
    TCCR2B = _BV(CS21); // set clkTS2 with prescaling factor of /8
#else
    TCCR2B = _BV(CS20); // set clkTS2 source to non prescaling
#endif
#endif

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