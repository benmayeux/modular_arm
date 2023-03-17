#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

struct Configuration {
    float length;
    byte address;
    byte orientation;
};

#endif