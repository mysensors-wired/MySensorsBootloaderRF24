/* Hardware ID via DIP Switch
   3 Bits fixed ID + 5 Bits from DIP switch
*/

#ifndef HardwareID_H
#define HardwareID_H

#define BROADCAST_ADDRESS	(0xFFu)
#define GATEWAY_ADDRESS		(0x00u)
#include <avr/io.h>

#define RS_NODE_BASE_ID         32
#define RELAY_MODULE_BASE_ID    64
#define FREE_BASE_ID            80
#define WALLNODE_BASE_ID        96

int8_t readHardwareIDtoEEPROM(){
    uint8_t id = 0;

    #ifdef RSNode_V1_0
    #define MY_RS485_DE_PIN PIND3
    #define MY_RS485_DE_PORT PORTD
    #define MY_RS485_DE_DDR DDRD
    /*  IO PINs
    PC2 = ID0
    PC3 = ID1
    PC4 = ID2
    PC5 = ID3
    PD2 = ID_C_0
    PD3 = ID_C_1 (Not working. Use only 4 bit for address)
    */
    DDRC    &=  ~(_BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5)) ;     //set as input
    PORTC   |=  _BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5) ;     //activate pullup

    DDRD    |= _BV(PD2); //set as ouput
    PORTD   &= ~_BV(PD2); //drive low
    _delay_ms(1);
    #ifdef RS_NODE_BASE_ID
        id = RS_NODE_BASE_ID | (0x0F & (  ~(  (PINC & (_BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5) ) ) >> 2))); 
    #else
        id = 0x0F & (  ~(  (PINC & (_BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5) ) ) >> 2)); 
    #endif

    #elif defined RELAY_BOARD_V1_0
    #define MY_RS485_DE_PIN PINE3
    #define MY_RS485_DE_PORT PORTE
    #define MY_RS485_DE_DDR DDRE
    /* IO Pins
    PD4 = ID0
    PD5 = ID1
    PD6 = ID2
    PD7 = ID3
    */
    const uint8_t INPUT_BIT_MASK = 0xF0;
    DDRD = DDRD & ~INPUT_BIT_MASK; //make sure ID pins are inputs
    PORTD = PORTD | INPUT_BIT_MASK; //enable ID pin pullups
    _delay_us(50);  // wait vor inputs stable

    #ifdef RELAY_MODULE_BASE_ID
        id = RELAY_MODULE_BASE_ID | (~INPUT_BIT_MASK & (  ~(  (PIND & (INPUT_BIT_MASK)) >> 4))); 
    #else
        id = ~INPUT_BIT_MASK & (  ~(  (PIND & INPUT_BIT_MASK) >> 4)); 
    #endif
    PORTD = PORTD & ~INPUT_BIT_MASK; //disable ID pin pullups

    #elif defined RELAY_WALLNODE_V1_0
    #define MY_RS485_DE_PIN PINC1
    #define MY_RS485_DE_PORT PORTC
    #define MY_RS485_DE_DDR DDRC

    /* IO Pins
    PD3 = ID4
    PD4 = ID3
    PD5 = ID2
    PD6 = ID1
    PD7 = ID0
    */
   const uint8_t INPUT_BIT_MASK = 0xF8;
    DDRD = DDRD & ~INPUT_BIT_MASK; //make sure ID pins are inputs
    PORTD = PORTD | INPUT_BIT_MASK; //enable ID pin pullups
    _delay_us(50);  // wait vor inputs stable

    #ifdef WALLNODE_BASE_ID
        id = WALLNODE_BASE_ID | (  (INPUT_BIT_MASK >> 3) & (~( PIND & (INPUT_BIT_MASK) )>> 3)); 
    #else
        id =( INPUT_BIT_MASK >> 3) & (~( PIND & (INPUT_BIT_MASK) )>> 3); 
    #endif
    PORTD = PORTD & ~INPUT_BIT_MASK; //disable ID pin pullups


    #else
    #warning "No valid HW-Platform"
    return -2;
    #endif
    if (id == GATEWAY_ADDRESS || id == BROADCAST_ADDRESS)   // Address reserved for Gateway or Broadcast
    {   
        return -1;
    }


    eeprom_update_byte(EEPROM_NODE_ID_ADDRESS, id);

    return 0;
}

#endif // HardwareID_H