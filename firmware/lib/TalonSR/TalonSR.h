#ifndef TalonSR_h
#define TalonSR_h

#include <Arduino.h>
#include <Servo.h>

class TalonSR{
    public:
        TalonSR(uint8_t PWMPin);
        void setEffort(int8_t effort);

    private:
        // The servo object that represents the motor controller
        Servo motorController;
};

#endif