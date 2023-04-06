#ifdef MAIN_BASIC_MODULE
#include <Arduino.h>
#include <BasicModule.h>

// Initialize the module with the correct PWM Pin, Potentiometer Pin, and Orientation Switch Pin
BasicModule thisModule = BasicModule(14,4,23);

// Used to track when to switch effort
uint32_t lastTime = millis();

// Set effort: 127 or full effort
int16_t effort = 127;

void setup() {
  Serial.begin(9600);
  delay(100);

  // Run the module setup (reads calibration data from EEPROM, determines mounting orientation, waits for communication from base module)
  thisModule.setup();
}

void loop() {
  // The main state machine for the module. This runs all of the functionality of the module
  thisModule.loop();


  // Every two seconds, switch between +effort and -effort for the value defined above.
  if(millis() - lastTime > 3000){
    effort = -effort;
    thisModule.setEffort(effort);

    lastTime = millis();
  }

  // Print out some of the values: 
  // isActuatorWithinLimits() shows the allowed directions the motor is allowed to move (used to prevent traveling beyond rotational limits)
  // DEBUG_PRINT(thisModule.isActuatorWithinLimits());
  // DEBUG_PRINT('\t');

  // // getEffort() gets the effort that is currently being sent to the motor. Note that even if the effort is being set in code, this will be zero if the motor isnt allowed to move in the desired direction
  // DEBUG_PRINT(thisModule.getEffort());
  // DEBUG_PRINT('\t');

  // // getRawPosition() gets the raw potentiometer reading from 0 to 4095. In the code, this is used to map to the current angle of the module.
  // DEBUG_PRINT(thisModule.getRawPosition());
  // DEBUG_PRINT('\t');

}
#endif