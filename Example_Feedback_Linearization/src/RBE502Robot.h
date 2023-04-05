#ifndef __RBE502ROBOT_H__
#define __RBE502ROBOT_H__

#include <Arduino.h>
#include <lx16a-servo.h>
#include <debugPrinter.h>
#include <math.h>
#include <serialReader.h>

class RBE502Robot{
    private:
        bool _debug = false;
        debugPrinter db;
        serialReader sr;
        LX16ABus servoBus;
        LX16AServo* motor1;
        LX16AServo* motor2;
        LX16AServo* motor3;
        uint8_t txPin;
        uint8_t txFlagPin;
        uint16_t motor1MinPos = 0;
        uint16_t motor1MaxPos = 24000;
        uint16_t motor2MinPos = 0;
        uint16_t motor2MaxPos = 24000;
        uint16_t motor3MinPos = 0;
        uint16_t motor3MaxPos = 24000;
        uint16_t zeroPositons[3] = {0,0,0};
        static void disableISR();
        int16_t* removeZeroOffset(int16_t* offsetMotorPositions); // removes offset to check if values are within bounds

    public:
        //Implemented
        RBE502Robot(uint8_t txPin, uint8_t txFlagPin, uint8_t enableSwitchPin, uint8_t motor1ID, uint8_t motor2ID, uint8_t motor3ID);
        void begin();
        void debug(bool enableDebug);
        void setMotorPositions(int16_t motor1Pos, int16_t motor2Pos, int16_t motor3Pos, int16_t moveTime = 0, bool withOffset = true);
        void disableMotors();
        void enableMotors();
        void setMotorLimits(uint16_t m1Min, uint16_t m1Max, uint16_t m2Min, uint16_t m2Max, uint16_t m3Min, uint16_t m3Max);
        void getMotorPositions(int16_t* motorPositions);
        void getRawMotorPositions(int16_t* rawMotorPositions);
        void getMotorPositionsDeg(float* motorPositions);
        void setMotorHomePositions(uint16_t m1ZeroPos, uint16_t m2ZeroPos, uint16_t m3ZeroPos);
        bool checkWithinServoBounds(int16_t* motorPositions, bool withOffset = true);
        int16_t* calcServoPosFromXYZ(int16_t x, int16_t y, int16_t z);
        void determineVelocity(int16_t* curMotorPos, float* curMotorVel);
        void PDController(float* Kp, float* Kd, int16_t* desiredPos, int16_t* curPos, float* curVel);
        void determineError(int16_t* desiredPos, int16_t* curPos, float* err, float* errDot);
        void determineErrorFromTraj(double* desiredPos, int16_t* curPos, float* desVel, float* curVel, float* errOut, float* errDotOut);

        void stateMachine();
        
        void cubicTraj(float t0, float tf, float q0, float qf, float q0d, float qfd, float* A);
        void cubicDeriv(float* A, float* Ad);
        void calcTrajPos(uint32_t t, float* A, float* desired);

        //Unimplemented
        void setXYZPosition(int16_t x, int16_t y, int16_t z){}; // Using IK
        void setMotorSpeeds(int16_t motor1Speed = 0, int16_t motor2Speed = 0, int16_t motor3Speed = 0); // To be used for 502 code
        void feedbackLinearization(float* Kp, float* Kd, int16_t* desiredPos, int16_t* curPos, float* curVel, uint32_t startTime = 0, float* A1 = 0, float* A2 = 0, float* A3 = 0); 
        
        void quinticTraj();
        void trapezoidalTraj();
};
#endif // __RBE502ROBOT_H__