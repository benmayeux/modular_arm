#include <Arduino.h>
#include <lx16a-servo.h>


#include <RBE502Robot.h>

#include <serialReader.h>


//HOME POSITION: 15504, 4296, 9360/9384
//Q1 Max: 25512, Min: 4000
//Q2 Max: 13824, Min: -3648?????
//Q3 Max: 14800, Min: 2000

// NEED FK Calculation to double check functionality of IK!!! REMEMBER THAT 100 CENTIDEGREES == 1 DEG
// Need to add a "robot.running" method which is what allows the robot to run (maybe setup like watchdog with ~100ms timeout?) so that you can monitor motor positions and ensure they
// stay within their bounds, otherwise they disable.
// Add ability to send commands through serial???

// ALL FUNCTIONS DEALING WITH MOTOR POSITION CAN AND SHOULD BE int16_t rather than uint16_t as this eliminates need for conversion, and all motor positions must be less than ~32,000 anyways
// Need to add ability to save/plot motor data. Might want to make a plotting mode with the debugPrinter for that, especially since need to plot pos, vel, acc. 




//Is it worth adding LCD and Pots so that you can set an xyz position or something, and then generate feedback linearization control to that point????

const int8_t enableSwitchPin = 3;
const int8_t txPin = 1;
const int8_t txFlagPin = 9;
const int8_t motor1ID = 1;
const int8_t motor2ID = 2;
const int8_t motor3ID = 3;

RBE502Robot robot(txPin,txFlagPin,enableSwitchPin,motor1ID,motor2ID,motor3ID);
serialReader sr;

void setup() {
// Interval in microsecs
    robot.begin();
    robot.debug(false);
    robot.setMotorHomePositions(15504, 4296, 9360);
    robot.setMotorLimits(15504-9500, 15504+9500,     0, 12000,     1500,15000 );
}

void loop() {
    robot.stateMachine();
 
}



