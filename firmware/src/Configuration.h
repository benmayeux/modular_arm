#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

struct Configuration {
    float length;
    int orientation;
    int torqueLimit;
    int posMin;
    int posMax;
};

#endif