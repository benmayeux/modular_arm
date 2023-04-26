#ifdef MAIN_BASIC_MODULE
#include <Arduino.h>
#include "BasicModule.h"

// Initialize the module with the correct PWM Pin, Potentiometer Pin, and Orientation Switch Pin
BasicModule thisModule = BasicModule(14,4,23);

// Used to track when to switch effort
// uint32_t lastTime = millis();

// // Set effort: 127 or full effort
// int16_t effort = 50;

void setup() {
  // Serial.begin(115200);
  delay(100);
  Serial2.begin(115200);

  // Run the module setup (reads calibration data from EEPROM, determines mounting orientation, waits for communication from base module)
  thisModule.setup(&Serial2,&Serial2);
}

void loop() {
  // The main state machine for the module. This runs all of the functionality of the module
  thisModule.loop();

  // // // Every n seconds, switch between +effort and -effort for the value defined above.
  // if(millis() - lastTime > 3000){
  //   effort = -effort;
  //   thisModule.setEffort(effort);
  //   lastTime = millis();
  // }

  // // Print out some of the values: 
  // // isActuatorWithinLimits() shows the allowed directions the motor is allowed to move (used to prevent traveling beyond rotational limits)
  // Serial.print(thisModule.isActuatorWithinLimits());
  // Serial.print('\t');

  // // getEffort() gets the effort that is currently being sent to the motor. Note that even if the effort is being set in code, this will be zero if the motor isnt allowed to move in the desired direction
  // Serial.print(thisModule.getEffort());
  // Serial.print('\t');

  // // getRawPosition() gets the raw potentiometer reading from 0 to 4095. In the code, this is used to map to the current angle of the module.
  // Serial.print(thisModule.getRawPosition());
  // Serial.println('\t');

  //765 is towards motor side 90 deg (Positive motor voltage lowers reading)
  //3050 is towards motor controller side (Negative motor voltage raises reading)


}
#endif