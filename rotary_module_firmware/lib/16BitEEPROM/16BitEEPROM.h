#ifndef BasicModule_h
#define BasicModule_h

#include <Arduino.h>
#include <EEPROM.h>

/*
    Reads the 16 bit number from EEPROM with the low byte in the mem1 location 
    and the high byte in the mem2 location.
    @param uint8_t mem1 : the memory location of the low byte
    @param uint8_t mem2 : the memory location of the high byte
    @return uint16_t    : the 16 bit integer stored between the two memory locations
*/
uint16_t read16BitFromEEPROM(uint8_t mem1, uint8_t mem2){
    uint16_t xlow = EEPROM.read(mem1);
    uint16_t xhigh = EEPROM.read(mem2);
    xhigh = xhigh << 8;
    return xhigh | xlow;
}


/*
    Saves the given 16 bit number to EEPROM with the low byte in the mem1 location 
    and the high byte in the mem2 location.
    @param uint16_t numToSave   : the 16 bit number that will be saved to EEPROM
    @param uint8_t mem1         : the memory location of the low byte
    @param uint8_t mem2         : the memory location of the high byte
    @return bool                : returns 1 if after reading the saved value it is == numToSave (If it returns 0, the number failed to save)
*/
bool save16BitToEEPROM(uint16_t numToSave, uint8_t mem1, uint8_t mem2){ 
    uint8_t xlow = numToSave & 0xff;
    uint8_t xhigh = (numToSave >> 8);
    EEPROM.write(mem1,xlow);
    EEPROM.write(mem2,xhigh);
    EEPROM.commit();

    if(numToSave == read16BitFromEEPROM(mem1,mem2)){
        // Serial.println("Successfully saved 16 Bit Integer to EEPROM"); // This can be done wherever this function is callled if necessary
        return true;
    }else{
        Serial.println("Failed to save 16 Bit Integer to EEPROM");
        return false;
    }

}

#endif