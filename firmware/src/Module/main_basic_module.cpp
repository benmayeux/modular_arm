#ifdef MAIN_BASIC_MODULE
#include <Arduino.h>
#include "BasicModule.h"

// Initialize the module with the correct PWM Pin, Potentiometer Pin, and Orientation Switch Pin
BasicModule thisModule = BasicModule(14,4,23);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  thisModule.setup(&Serial2,&Serial2);

  delay(3000);
  while(Serial2.available()) {
    Serial2.read();
  }
  delay(3000);
  // Run the module setup (reads calibration data from EEPROM, determines mounting orientation, waits for communication from base module)
}

void loop() {
  // The main state machine for the module. This runs all of the functionality of the module
  thisModule.loop();

}
#endif