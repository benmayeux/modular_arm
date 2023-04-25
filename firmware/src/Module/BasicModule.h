#ifndef BasicModule_h
#define BasicModule_h

#include <Arduino.h>

#include <TalonSR.h>

#include <EEPROM.h>

#include "UARTBusDataDelegate.h"
#include "UARTBus.h"


class BasicModule: public UARTBusDataDelegate {
    public:
        BasicModule(uint8_t PWMPin, uint8_t potentiometerPin, uint8_t mountingOrientationSwitchPin);
        void setConfiguration(Configuration c);
        byte fetchData(CommandType command, int16_t *data);
        void setup(Stream *in, Stream *out);
        void loop();
        int16_t fetchData(CommandType command);
        Configuration getConfiguration();
        void processCommand(Command c);
        void stateMachine();
        void setPosition(int16_t positionCentidegrees);
        int16_t getPosition();
        uint16_t getRawPosition(){
            return this->rawPotentiometerVal;
        }
        int16_t getDesiredPosition(){
            return this->desiredPositionCentidegrees;
        }
        void setPosKpKiKd(float Kp, float Ki, float Kd);
        void setVelKpKiKd(float Kp, float Ki, float Kd);
        void getPosKpKiKd(float* retVal);
        void getVelKpKiKd(float* retVal);
        void setVelocity(int16_t velocityCentidegrees);
        int16_t getVelocity();
        void setEffort(int16_t effort);
        int8_t getEffort();
        bool getArmOrientation();
        int8_t isActuatorWithinLimits();

        // MODEs of the module
        enum MODE {MODE_DISABLE, MODE_EFFORT, MODE_POSITION, MODE_VELOCITY, MODE_CALIBRATION};
        uint8_t getMode(); 
        void calibrate(); 
        void disable(); 
        uint16_t getMaxPotRange(){
            return this->maxPotentiometerRange;
        }
        uint16_t getMinPotRange(){
            return this->minPotentiometerRange;
        }
        // void enable(); // Will enable the module to move (Probably dont need)
        // String getStatus(); // (Probably not needed, getMode can be used instead)   
        
        // The motor controller
        TalonSR motor;

    private:
        // Private Methods:--------------------------------------------------------------------
        void setEffortPrivate(int16_t effort);
        void setMode(MODE mode);
        void controlLoopEffort();
        void controlLoopPosition(bool Reset = false);
        void controlLoopVelocity();
        void controlLoopCalibration();
        static bool save16BitToEEPROM(uint16_t numToSave, uint8_t mem1, uint8_t mem2);
        static uint16_t read16BitFromEEPROM(uint8_t mem1, uint8_t mem2);




        // General data:------------------------------------------------------------------------
        UARTBus dataBus;

        // GPIO Pin of the motor controller
        uint8_t PWMPin;

        // GPIO Pin of the position potentiometer
        uint8_t potentiometerPin;

        // GPIO Pin of the orientation switch
        uint8_t mountingOrientationSwitchPin;

        // Mode of the module, Initialized to MODE_DISABLE
        MODE mode = MODE_DISABLE;
        
        
        // Setup/Calibration Data:---------------------------------------------------------------
        // Maximum value of the potentiometer to stay within physical range of movement, Pulled from Flash during setup()
        uint16_t maxPotentiometerRange = 3071; // Pull this from flash on startup (Defaults to 3072 if hasnt had a value yet)

        // Minimum value of the potentiometer to stay within physical range of movement, Pulled from Flash on setup()
        uint16_t minPotentiometerRange = 1023; // Pull this from flash on startup (Defaults to 1024 if hasnt had a value yet)

        // The orientation of the robot, determined during setup()
        bool orientation;

        // TODO: determine the min and max angles from CAD, then confirm them in the real world (Used to map the calibration values)
        // TODO: Limit angle from -90 to 90 to prevent collisions with other modules. Ensure that there are no other ways for modules to collide based on orientation.
        // Minimum angle reached during calibration (When moving part is contacting stationary part)
        const int16_t minAngle = -120; // Min angle reached during calibration to the nearest degree

        // Maximum angle reached during calibration (When moving part is contacting stationary part)
        const int16_t maxAngle = 120; // Max angle reached during calibration to the nearest degree
        
        
        // Control Loop Variables/Methods:------------------------------------------------------
        void updatePosVelAcc();

        // TODO: Determine good PID values for position and velocity control for turnkey functionality
        // PID Kp value for Positioning control loop
        float posKp = 1;

        // PID Ki value for Positioning control loop
        float posKi = 0;

        // PID Kd value for Positioning control loop
        float posKd = 0;

        // PID Kp value for Velocity control loop
        float velKp = 1;

        // PID Ki value for Velocity control loop
        float velKi = 0;

        // PID Kd value for Velocity control loop
        float velKd = 0;

        // Flag to update PID after the pos,vel,acc is updated. Reset to false after updating PID
        bool readyToUpdatePID = false;

        int8_t motorEffort = 0;

        // Desired position for joint to be driven to in centidegrees using PID
        int16_t desiredPositionCentidegrees = 0;

        // Raw value of the joint potentiometer
        uint16_t rawPotentiometerVal = 0;

        // Current position of the joint in centidegrees from center (Straight)        
        int16_t currentPositionCentidegrees = 0;

        // controlLoop_ms ago position of the joint in centidegrees from center (Straight)        
        int16_t lastPositionCentidegrees = 0; // needs to be read in state machine every x ms

        // Desired velocity for the joint to move at using PID
        int16_t desiredVelocityCentidegrees = 0;

        // Current velocity of the joint in centidegrees/second
        int16_t currentVelocityCentidegrees = 0;

        // controlLoop_ms ago velocity of the joint in centidegrees/second
        int16_t lastVelocityCentidegrees = 0;

        // Current acceleration of the joint in centidegrees/second^2
        int16_t currentAccelCentidegrees = 0;

        // TODO: Determine a good number of ms for control loop for good functionality
        // The number of milliseconds between position measurements from the potentiometer
        uint16_t controlLoop_ms = 10;

        // The last time in milliseconds that the position was read from the potentiometer
        uint32_t lastTimeCalculated = millis();

        Configuration configuration;
};


#endif