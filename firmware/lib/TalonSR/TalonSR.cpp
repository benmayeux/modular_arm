#include <Arduino.h>
#include "TalonSR.h"
#include <Servo.h>

/*
    Creates an instance of a TalonSR Object.
    @param uint8_t PWMPin       : The GPIO Pin to use for PWM control of motor controller 
*/
TalonSR::TalonSR(uint8_t PWMPin){
    this->motorController.attach(PWMPin);
    Serial.println("Talon Pin attached");
}

/*
    Sets the motor effort of the TalonSR. Directly proportional to voltage -12 to 12v (or whatever the input voltage is)
    @param int8_t effort        : desired effort of the motor from -128 to 127
*/
void TalonSR::setEffort(int8_t effort){
    this->motorController.writeMicroseconds(map(effort,-128,127,1000,2000));
}