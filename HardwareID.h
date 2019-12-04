/* Hardware ID via DIP Switch
   3 Bits fixed ID + 5 Bits from DIP switch
*/

#ifndef HardwareID_H
#define HardwareID_H

static const uint8_t RS_NODE_BASEID =  0x01 << 5;

int8_t readHardwareIDtoEEPROM(){
    uint8_t id = 0;

    #ifdef RSNode_V1_0
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
    id = RS_NODE_BASEID | (0x0F & (~((PINC & (_BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5))) >> 2))); 

    #else
    #warning "No valid HW-Platform"
    return -2;
    #endif
    if (id == GATEWAY_ADDRESS || id == BROADCAST_ADDRESS)   // Address reserved for Gateway or Broadcast
    {   
        return -1;
    }
    
    if(eeprom_read_byte(EEPROM_NODE_ID_ADDRESS) != id){
        eeprom_update_byte(EEPROM_NODE_ID_ADDRESS,id);
    }
    return 0;
}

#endif // HardwareID_H