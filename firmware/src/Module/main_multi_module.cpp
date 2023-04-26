
#ifdef MAIN_BASIC_MODULE_MULTI
#include <Arduino.h>
#include "BasicModule.h"
#include "LoopbackStream.h"

#ifndef N_JOINTS
#define N_JOINTS 3
#endif

// Initialize the module with the correct PWM Pin, Potentiometer Pin, and Orientation Switch Pin

BasicModule* modules[N_JOINTS];
LoopbackStream* streams[N_JOINTS-1];

// Used to track when to switch effort
uint32_t lastTime = millis();

// Set effort: 127 or full effort
int16_t effort = 127;

void setup() {
  Serial.begin(9600);
  Serial2.begin(115200);

  delay(100);
  for (int i = 0; i < N_JOINTS -1; i++) {
    streams[i] = new LoopbackStream();
  }
  for (int i = 0; i<N_JOINTS; i++) {
    modules[i] = new BasicModule(14,4,23); // TODO: set pins accordingly

    Configuration c = Configuration();
    c.length = i+1; // TODO: get length
    c.posMax = modules[i]->getMaxPotRange();
    c.posMin = modules[i]->getMinPotRange();
    c.orientation = modules[i]->getArmOrientation();
    modules[i]->setConfiguration(c);

    if(i == 0) {
      modules[i]->setup(&Serial2, streams[i]);
    } else if (i == N_JOINTS -1) {
      modules[i]->setup(streams[i-1], &Serial2);
    } else {
      modules[i]->setup(streams[i-1], streams[i]);
    }
  }
  // Run the module setup (reads calibration data from EEPROM, determines mounting orientation, waits for communication from base module)
}

void loop() {
  for (int i = 0; i < N_JOINTS; i++) {
    modules[i]->loop();
  }
  // The main state machine for the module. This runs all of the functionality of the module
}
#endif