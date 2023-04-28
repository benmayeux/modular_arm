
#ifdef MAIN_BASIC_MODULE_MULTI
#include <Arduino.h>
#include "ServoModule.h"
#include "LoopbackStream.h"

#ifndef N_JOINTS
#define N_JOINTS 3
#endif

// Initialize the module with the correct PWM Pin, Potentiometer Pin, and Orientation Switch Pin

ServoModule* modules[N_JOINTS];
LoopbackStream* streams[N_JOINTS-1];

// Used to track when to switch effort
uint32_t lastTime = millis();

// Set effort: 127 or full effort
int16_t effort = 127;
int8_t pwn_pins[3] = {13,26,32};
void setup() {
  
  Serial.begin(115200);
  Serial2.begin(115200);
  
  delay(3000);
  while(Serial2.available()) {
    Serial2.read();
  }

  for (int i = 0; i < N_JOINTS -1; i++) {
    streams[i] = new LoopbackStream();
  }
  for (int i = 0; i<N_JOINTS; i++) {
    modules[i] = new ServoModule(pwn_pins[i],23); // TODO: set pins accordingly
    Configuration c = Configuration();
    c.length = i+1; // TODO: get length
    c.posMax = modules[i]->getMaxPotation();
    c.posMin = modules[i]->getMinPotation();
    c.orientation = modules[i]->getArmOrientation();
    modules[i]->setConfiguration(c);

    if(i == 0) {
      modules[i]->setup(&Serial2, streams[i]);
    } else if (i == N_JOINTS -1) {
      modules[i]->setup(streams[i-1], &Serial2);
    } else {
      modules[i]->setup(streams[i-1], streams[i]);
    }
    delay(3000);
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