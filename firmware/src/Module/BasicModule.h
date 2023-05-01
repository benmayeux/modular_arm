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
        static bool save16BitToEEPROM(uint16_t numToSave, uint8_t mem1, uint8_t mem2);
        static uint16_t read16BitFromEEPROM(uint8_t mem1, uint8_t mem2);

        // MODEs of the module
        enum MODE {MODE_DISABLE, MODE_EFFORT, MODE_POSITION, MODE_VELOCITY, MODE_CALIBRATION, MODE_FOLLOWING_CUBIC_TRAJ};
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

        void startCubicTraj(int16_t desiredPosCentidegrees);

    private:
        // Private Methods:--------------------------------------------------------------------
        void setEffortPrivate(int16_t effort);
        void setMode(MODE mode);
        void controlLoopEffort();
        void controlLoopPosition(bool Reset = false);
        void controlLoopVelocity();
        void controlLoopCalibration();
        void controlLoopCubicTraj();


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
        uint16_t maxPotentiometerRange = 3050; // Pull this from flash on startup (Defaults to 3072 if hasnt had a value yet)

        // Minimum value of the potentiometer to stay within physical range of movement, Pulled from Flash on setup()
        uint16_t minPotentiometerRange = 765; // Pull this from flash on startup (Defaults to 1024 if hasnt had a value yet)

        // The orientation of the robot, determined during setup()
        bool orientation;

        // TODO: determine the min and max angles from CAD, then confirm them in the real world (Used to map the calibration values)
        // TODO: Limit angle from -90 to 90 to prevent collisions with other modules. Ensure that there are no other ways for modules to collide based on orientation.
        // Minimum angle reached during calibration (When moving part is contacting stationary part)
        const int16_t minAngle = -90; // Min angle reached during calibration to the nearest degree

        // Maximum angle reached during calibration (When moving part is contacting stationary part)
        const int16_t maxAngle = 90; // Max angle reached during calibration to the nearest degree
        
        // Deadband where below this value, motor effort is set to zero
        const int8_t PIDMotorEffortDeadband = 5;

        // Minimum motor effort required to allow motor to move  
        const int8_t minimumMotorEffortForMovement = 10;
        
        // Control Loop Variables/Methods:------------------------------------------------------
        void updatePosVelAcc();

        // TODO: Determine good PID values for position and velocity control for turnkey functionality
        // PID Kp value for Positioning control loop
        float posKp = 5;

        // PID Ki value for Positioning control loop
        float posKi = 0.03;

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



        /*---------------------------------------------------------------- HERE DOWN WAS ADDED TO ALLOW FOR CUBIC TRAJECTORIES (and some stuff above)
        */

        // SPEED/VELOCITY of the joint
        uint32_t centiDegreesPerSecond = 2000;

        /* 
        Takes a relative time value, and plugs in T to the equation where A represents the coefficents: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
        @param uint32_t  t      : The time value in milliseconds
        @param float*    A      : The A matrix of coeffients of the equation above
        @param float*    desired: The output, desired position, velocity and acceleration at that time [q;qdot;qdotdot] in DEG and Seconds
        */
        void calcTrajPos(uint32_t t, float* A, float* desired){
            double tsec = t/1000.0;
            // float Ad[4]; //A dot (velocity)
            // float Add[4]; // A dot dot (acceleration)
            // this->cubicDeriv(A, Ad);
            // this->cubicDeriv(Ad, Add);
            
            desired[0] = (A[0]*1 + A[1]*tsec + A[2]*tsec*tsec + A[3]*tsec*tsec*tsec)/100.0;
            // desired[1] = (Ad[0]*1 + Ad[1]*tsec + Ad[2]*tsec*tsec + Ad[3]*tsec*tsec*tsec)/100.0;
            // desired[2] = (Add[0]*1 + Add[1]*tsec + Add[2]*tsec*tsec + Add[3]*tsec*tsec*tsec)/100.0;
        }


        /*
            Plan a cubic trajectory from a given position to another position
            @param float  t0     : Starting time of traj (0)
            @param float  tf     : Ending time of traj
            @param float  q0     : starting joint position
            @param float  qf     : ending joint position
            @param float  q0d    : starting joint velocity
            @param float  q0f    : ending joint velocity
            @param float* A      : Output A array that fulfills the equation: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
        */
        void cubicTraj(float t0, float tf, float q0, float qf, float q0d, float qfd, float* A){
            A[0] = -(q0*tf*tf*tf - qf*t0*t0*t0 - 3*q0*t0*tf*tf - q0d*t0*tf*tf*tf + 3*qf*t0*t0*tf + qfd*t0*t0*t0*tf + q0d*t0*t0*tf*tf - qfd*t0*t0*tf*tf)/((t0 - tf)*(t0*t0 - 2*t0*tf + tf*tf));
            A[1] = -(q0d*tf*tf*tf - qfd*t0*t0*t0 + q0d*t0*tf*tf - 2*q0d*t0*t0*tf + 2*qfd*t0*tf*tf - qfd*t0*t0*tf + 6*q0*t0*tf - 6*qf*t0*tf)/((t0 - tf)*(t0*t0 - 2*t0*tf + tf*tf));
            A[2] = (3*q0*t0 + 3*q0*tf - 3*qf*t0 - 3*qf*tf - q0d*t0*t0 + 2*q0d*tf*tf - 2*qfd*t0*t0 + qfd*tf*tf - q0d*t0*tf + qfd*t0*tf)/((t0 - tf)*(t0*t0 - 2*t0*tf + tf*tf));
            A[3] = -(2*q0 - 2*qf - q0d*t0 + q0d*tf - qfd*t0 + qfd*tf)/((t0 - tf)*(t0*t0 - 2*t0*tf + tf*tf));

        }

        /*
            Take an A matrix representing the coefficients of a (maximum of cubic) trajectory, and find the derivatives coefficients
            @param float* A     : An array of 4 values representing the coefficients of a trajectory: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
            @param float* Ad    : An output array of 4 values representing the coefficients of derivative of the A trajectory.
        */
        void cubicDeriv(float* A, float* Ad){
            Ad[0] = A[1];
            Ad[1] = A[2]*2;
            Ad[2] = A[3]*3;
            Ad[3] = 0;
        }

        // Place to store the trajectory values
        float A[4] = {0,0,0,0};

        uint32_t trajStartTime = 0;
        uint32_t trajEndTime = 0;
        int16_t trajDesiredPosCentidegrees = 0;

};


#endif