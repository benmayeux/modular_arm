#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

struct Configuration {
    uint8_t moduleNum = -1;
    float length;
    bool orientation;
    uint16_t torqueLimit;
    int16_t posMin;
    int16_t posMax;
};

#endif