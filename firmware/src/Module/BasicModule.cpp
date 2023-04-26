/*
    This class is used by:
        - running nameOfYourModule.setup() in the setup loop
        - running nameOfYourModule.stateMachine() in the main loop continuously (non-blocking code required)
        - Setting either the position or effort using nameOfYourModule.setPosition() or nameOfYourModule.setEffort() within the main loop. 
            Note that effort needs to be continuously set, or motor will run to end of its travel. End stop protection IS built in AS LONG AS MODULE IS CALIBRATED

*/



#include <Arduino.h>
#include "BasicModule.h"
#include <TalonSR.h>
#include "DebugPrint.h"

/*
 * Turn A into a string literal without expanding macro definitions
 * (however, if invoked from a macro, macro arguments are expanded).
 */





/*
    Constructor for BasicModule Object.
    @param uint8_t PWMPin           : The GPIO Pin to use for PWM control of motor controller 
    @param uint8_t potentiometerPin : The GPIO Pin to use for the potentiometer position reading
    @param uint8_t mountingOrientationSwitchPin : The GPIO Pin to use to determine the module orientation 
*/
BasicModule::BasicModule(uint8_t PWMPin, uint8_t potentiometerPin, uint8_t mountingOrientationSwitchPin): motor(TalonSR(PWMPin)){
    this->PWMPin = PWMPin;
    this->potentiometerPin = potentiometerPin;
    this->mountingOrientationSwitchPin = mountingOrientationSwitchPin;
}

void BasicModule::setConfiguration(Configuration c) {
    configuration = c;
}

byte BasicModule::fetchData(CommandType command, int16_t* data) {
    byte nData = 0;
    CommandType returnType = (CommandType)(command & CommandType::RETURN_MASK);
    if (returnType & CommandType::RETURN_POSITION) {
        data[nData] = this->getPosition();
        nData++;
    }
    if (returnType & CommandType::RETURN_EFFORT) {
        data[nData] = this->getEffort();
        nData++;
    }
    if (returnType & CommandType::RETURN_VELOCITY) {
        data[nData] = this->getVelocity();
        nData++;
    }
    return nData;
}

// TODO: get actual values from EEPROM?
Configuration BasicModule::getConfiguration() {
    return configuration;
}

void BasicModule::processCommand(Command c) {
    switch(c.getCommandTarget()) {
        case CommandType::EFFORT:
            DEBUG_PRINT((String) dataBus.address + ": setting effort to " + (String)c.data[0]);
            this->mode = MODE_EFFORT;
            this->setEffort(c.data[0]);
            break;
        case CommandType::POSITION:
            DEBUG_PRINT((String) dataBus.address + ": setting pos to " + (String)c.data[0]);
            this->mode = MODE_POSITION;
            this->setPosition(c.data[0]);
            break;
        default:
            break;
    }
}

/*
    The general state machine to be run in a loop to allow for functionality of the Module.
*/
void BasicModule::stateMachine(){
    /* Read and update position and velocity every controlLoop_ms
            updates the rawPotentiometerVal, 
            currentPositionCentidegrees, lastPositionCentidegrees, 
            currentVelocityCentidegrees, lastVelocityCentidegrees
            currentAccelCentidegrees
    */
   
    switch(this->getMode()){
        case MODE_EFFORT:
            this->controlLoopEffort(); 
        break;

        case MODE_POSITION:
            this->controlLoopPosition();
        break;

        case MODE_VELOCITY:
            // NOT IMPLEMENTED
        break;

        case MODE_CALIBRATION:
            this->controlLoopCalibration();
        break;

        default: // Default will set the mode to disable, then run the DISABLE case this iteration
            this->setMode(MODE_DISABLE);
        case MODE_DISABLE:
            // Sets motor effort to zero
            this->setEffort(0);
            this->setMode(MODE_DISABLE);// setEffort changes mode to MODE_EFFORT, this changes it back
        break;
    }
}

/*
    The control loop that the state machine runs when in MODE_EFFORT
    - Sends raw effort commands to motor as long as it does not cause joint exceed bounds
*/
void BasicModule::controlLoopEffort(){
    this->setEffort(this->motorEffort); // Runs this method, requires user to set effort using setEffort() method
}

/*
    The control loop that the state machine runs when in MODE_POSITION
    - Uses the value set using the setPosition() method, and drives actuator to that position
    - Uses the PID values set using setPosKpKiKd() method to drive actuator
    @param bool Reset : When true, it resets the held value for the I term. Only used when setting a new position
*/
void BasicModule::controlLoopPosition(bool Reset){  
    float Err = (this->desiredPositionCentidegrees - this->currentPositionCentidegrees)/100; // Error in Degrees to keep P, I, D values in reasonable range
    // DEBUG_PRINT("Err: ");
    // DEBUG_PRINT(Err);
    // DEBUG_PRINT("Kp");
    // DEBUG_PRINT(this->posKp);

    static float lastErr = 0; // Hold last value for I calculations
    float P = Err * this->posKp; // Calculate P term
    static float I = 0; // Create static var to hold I
    float D = (Err - lastErr) * posKd; // Calculate D term
    
    if(Reset){ // Reset I term when needed, and exit function
        I = 0;
        return;
    }

    if(this->readyToUpdatePID){ 
        I = I + (Err * this->posKi); // Update I
        lastErr = Err; // Update lastErr
        this->readyToUpdatePID = false; // Reset flag
    }

    this->setEffortPrivate(P+I+D); // Set motor effort (argument is limited to int8_t)
}



/*
    The control loop that the state machine runs when in MODE_CALIBRATION
    - Runs the motor to both limits at low effort for 5 seconds to determine max and min position
*/
void BasicModule::controlLoopCalibration(){
    
    const int8_t calEffort = 13; // Motor effort from -128 to 127 to drive at for calibration
    static uint32_t startTime = millis(); // timer to run motor
    static uint8_t state = 0; // State of the calibration
    static uint16_t newPotentiometerMax = 4095; // variable to hold new potentiometer max
    static uint16_t newPotentiometerMin = 0; // variable to hold new potentiometer min

    // allow movement past current set limits by changing current set limits to max range
    this->maxPotentiometerRange = 4095;
    this->minPotentiometerRange = 0;
    

    switch(0){
        case 0: // Calibrate the max
            this->setEffort(calEffort);
            if(millis() - startTime >= 5000){
                newPotentiometerMax = this->rawPotentiometerVal;
                state++;
                startTime = millis();
            }
        break;

        case 1: // Calibrate the min
            this->setEffort(-calEffort);
            if(millis() - startTime >= 5000){
                newPotentiometerMin = this->rawPotentiometerVal;
                state++;
                startTime = millis();
            }
        break;

        // TODO: Find a way to confirm calibrated values are a good calibration
        case 2: // Save the values to flash and exits calibration
            //TODO: Save the calibration values to flash WITH AN OFFSET (Only allow -90 to 90 deg movement.)
            // Probably use the map of pot->angle and desired max/min angles to map back angle->pot (will need another var for calibration angle)
            this->maxPotentiometerRange = newPotentiometerMax;
            this->minPotentiometerRange = newPotentiometerMin;
            this->setMode(MODE_DISABLE);
        break;
    }
}


/*
    Boots up the module, determines necessary constants, and initiates communications
*/
void BasicModule::setup(Stream* in, Stream* out){
    // Set initial mode (disabled)
        this->mode = MODE_DISABLE;
        this->dataBus = UARTBus(this, in, out);
    // Pull calibration data from flash
        // Begin EEPROM
#ifdef SIMULATION
        EEPROM.begin();
#else
        EEPROM.begin(4);
#endif

        // Read values from flash and format into 16 bit unsigned integers
        uint16_t minPotRange = read16BitFromEEPROM(0,1);
        uint16_t maxPotRange = read16BitFromEEPROM(2,3);

        // Check that the minPotRange is reasonable, and update in EEPROM to default value if it is not reasonable
        DEBUG_PRINT("minPotRange read from flash: ");
        DEBUG_PRINT(minPotRange);
        if(minPotRange > 2048){// Check that value is reasonable (MUST be less than 2048). If it is not, update to default
            minPotRange = 1024; // default is from 1/4 of rotation to 3/4 of rotation of potentiometer
            DEBUG_PRINT("Updated minPotRange to: ");
            DEBUG_PRINT(minPotRange);

            if(save16BitToEEPROM(minPotRange,0,1)){ // Save the default value to EEPROM
                DEBUG_PRINT("minPotRange saved to EEPROM");
            }else{
                DEBUG_PRINT("minPotRange FAILED to save to EEPROM");
            }
        }
        this->minPotentiometerRange = minPotRange;

        DEBUG_PRINT("maxPotRange read from flash: ");
        DEBUG_PRINT(maxPotRange);
        if(maxPotRange >= 4095 or maxPotRange <= 2048){// If the value hasnt been saved yet, or was saved/read incorrect, or is well below a reasonable value, set it to default
            maxPotRange = 3072; // default is from 1/4 of rotation to 3/4 of rotation
            DEBUG_PRINT("Updated maxPotRange to: ");
            DEBUG_PRINT(maxPotRange);

            if(save16BitToEEPROM(maxPotRange,2,3)){
                DEBUG_PRINT("maxPotRange saved to EEPROM");
            }else{
                DEBUG_PRINT("maxPotRange FAILED to save to EEPROM");
            }
        }
        this->maxPotentiometerRange = maxPotRange;

    // Determine and set the orientation flag
    // Confirm that you can digitalRead in the setup loop
        pinMode(this->mountingOrientationSwitchPin, INPUT_PULLUP);
        this->orientation = digitalRead(this->mountingOrientationSwitchPin);


    // Set the PWMPin to output
    // this->motor = TalonSR(this->PWMPin);

    // TODO: Wait for previous modules communication
    // TODO: Once received, interpret/add data and forward msg
}

void BasicModule::loop() {
    // sense
    this->updatePosVelAcc(); 
    // process
    Command command = this->dataBus.handleCommunication();
    this->processCommand(command);
    // act
    this->stateMachine();
}
/*
    Sets desired position of the joint using built in PID controller.
    @param uint16_t positionCentidegrees : the desired position of the joint in centidegrees from center (straight)
*/
void BasicModule::setPosition(int16_t positionCentidegrees){
    // Reset the I term of the control loop
        this->controlLoopPosition(true);
    // set mode to position mode
        this->setMode(MODE_POSITION);
    // Set Desired Position
        this->desiredPositionCentidegrees = positionCentidegrees;
}


/*
    Gets and returns the current position of the joint in centidegrees.
    @return uint8_t : The current position of the joint in centidegrees from center (straight) 
*/
int16_t BasicModule::getPosition(){
    return this->currentPositionCentidegrees;
}


/*
    Sets the Position Kp, Ki, Kd values to the given values, returns the updated Kp, Ki, Kd. 
    Any given value that is <0 will not change the current value
    @param float Kp : The new Kp value (<0 doesnt change current Kp) 
    @param float Ki : The new Ki value (<0 doesnt change current Ki)
    @param float Kd : The new Kd value (<0 doesnt change current Kd) 
*/
void BasicModule::setPosKpKiKd(float Kp, float Ki, float Kd){
    if(Kp >= 0){
        this->posKp = Kp;
    }
    if(Ki >= 0){
        this->posKi = Ki;
    }
    if(Kd >= 0){
        this->posKd = Kd;
    }
}


/*
    Getter for Position Kp, Ki, Kd
    @param float*  : A 3x1 Array to store the current Kp, Ki, Kd values 
*/
void BasicModule::getPosKpKiKd(float* retVal){
    retVal[0] = this->posKp;
    retVal[1] = this->posKi;
    retVal[2] = this->posKd;
}



/*
    Sets the effort of the joint -128 to 127. Limit movement to stay within range
    @param int8_t effort : motor effort mapped from (-128 to 127) to (-Vcc to Vcc)
*/
void BasicModule::setEffort(int16_t effort){
    this->setMode(MODE_EFFORT); // This function is public, so when user sets effort of motor, it is assumed they want module in effort mode.
    this->setEffortPrivate(effort);
}

/*
    Sets the effort of the joint -128 to 127. Limit movement to stay within range. Does NOT set mode to MODE_EFFORT
    This is meant to be used within methods so that motor effort can be set without changing mode to MODE_EFFORT
    @param int8_t effort : motor effort mapped from (-128 to 127) to (-Vcc to Vcc)
*/
void BasicModule::setEffortPrivate(int16_t effort){

    effort = constrain(effort,-128,127); //constrain value to 8 bit range

    // Limit the possible efforts
    int8_t allowedDirection = this->isActuatorWithinLimits();

    // allowedDirection * the set effort is >= zero if the effort is in an allowed direction
    // ex. allowedDirection = 1, effort = 122: 122 >=0
    // ex. allowedDirection = -1,effort = 122: -122 NOT >=0
    // ex. allowedDirection = 0, effort = 122: 0 >= 0
    if(int16_t(allowedDirection) * effort >= 0){
            this->motor.setEffort(effort);
            this->motorEffort = effort;
    }else{
        this->motor.setEffort(0);
        this->motorEffort = 0;
    }
}



/*
    Returns the motor effort that was previously set
    @return int8_t : The currently set motor effort -128 to 127
*/
int8_t BasicModule::getEffort(){
    return this->motorEffort;
}

/*
    Returns the orientation of the module. 0 means the actuator is inline with the previous links actuator. 1 means it is 90deg offset.
    @return bool : The orientation of the module where 0 = parallel to previous module, 1 = 90 degrees offset from previous module.  
*/
bool BasicModule::getArmOrientation(){
    return this->orientation;
}


/*
    Sets the mode of the module based on the MODE enumeration
    @param MODE mode : The mode to set the module to based on the enumeration 
*/
void BasicModule::setMode(MODE mode){
    // Set the desired position to the current position
    this->desiredPositionCentidegrees = this->currentPositionCentidegrees;
    // Set the motor Effort to 0
    this->motor.setEffort(0);
    //set the mode to the new mode
    this->mode = mode;
} 


/*
    Updates the position and velocity attributes if more than controlLoop_ms has passed
*/
void BasicModule::updatePosVelAcc(){
    // If the control loop time has passed,
    if(millis() - this->lastTimeCalculated > this->controlLoop_ms){
        // Read the raw potentiometer value
        this->rawPotentiometerVal = analogRead(this->potentiometerPin);

        // Save the last measured value into the lastPositionCentidegrees attribute
        this->lastPositionCentidegrees = this->currentPositionCentidegrees;

        // Map the raw potentiometer value to the angle of the joint using calibration data and min/max angles (angles of calibration)
        this->currentPositionCentidegrees =  map(this->rawPotentiometerVal,this->minPotentiometerRange,this->maxPotentiometerRange,this->minAngle * 100, this->maxAngle * 100);
        
        // save the old velocity to the lastVelocityCentidegrees attribute
        this ->lastVelocityCentidegrees = this->currentVelocityCentidegrees;

        // centidegrees/s == centidegrees/ms * 1000
        this->currentVelocityCentidegrees = ((this->currentPositionCentidegrees - this->lastPositionCentidegrees)*1000)/this->controlLoop_ms;
        
        // centidegrees/s^2 = ((centidegrees/s)/ms) * 1000
        this->currentAccelCentidegrees = ((this->currentVelocityCentidegrees - this->lastVelocityCentidegrees)*1000)/this->controlLoop_ms;

        // Update the last time so this will update in another controlLoop_ms
        this->lastTimeCalculated = millis();

        // Set flag that values have updated, so PID can update.
        this->readyToUpdatePID = true;
    }
}


/*
    Returns the mode of the module.
    @return MODE : the current mode that the module is in
*/
uint8_t BasicModule::getMode(){
    return this->mode;
}


/*
    Calibrates the module by moving the joint to either end until the joint can no longer move at a low motor effort. DO NOT USE if module is attached to other modules
*/
void BasicModule::calibrate(){
    this->setMode(MODE_CALIBRATION); 
} 


/*
    Sets the modules mode to DISABLED 
*/
void BasicModule::disable(){
    this->setMode(MODE_DISABLE);
}

/*
    Returns the allowed directions of movement of the actuator: 
    -1 means actuator can only move in negative direction (at postive limit), 
     0 means actuator can move in both directions
     1 means actuator can only move in positive direction (at negative limit)
    @return int8_t : the directions the actuator is allowed to move
*/
int8_t BasicModule::isActuatorWithinLimits(){
    if(this->rawPotentiometerVal >= this->maxPotentiometerRange){
        return -1;
    }else if(this->rawPotentiometerVal <= this->minPotentiometerRange){
        return 1;
    }else{
        return 0;
    }
}





/*
    Reads the 16 bit number from EEPROM with the low byte in the mem1 location 
    and the high byte in the mem2 location.
    @param uint8_t mem1 : the memory location of the low byte
    @param uint8_t mem2 : the memory location of the high byte
    @return uint16_t    : the 16 bit integer stored between the two memory locations
*/
uint16_t BasicModule::read16BitFromEEPROM(uint8_t mem1, uint8_t mem2){
    uint16_t xlow = EEPROM.read(mem1);
    uint16_t xhigh = EEPROM.read(mem2);
    xhigh = xhigh << 8;
    return xhigh | xlow;
}


/*
    Saves the given 16 bit number to EEPROM with the low byte in the mem1 location 
    and the high byte in the mem2 location.
    @param uint16_t numToSave   : the 16 bit number that will be saved to EEPROM
    @param uint8_t mem1         : the memory location of the low byte
    @param uint8_t mem2         : the memory location of the high byte
    @return bool                : returns 1 if after reading the saved value it is == numToSave (If it returns 0, the number failed to save)
*/
bool BasicModule::save16BitToEEPROM(uint16_t numToSave, uint8_t mem1, uint8_t mem2){ 
    uint8_t xlow = numToSave & 0xff;
    uint8_t xhigh = (numToSave >> 8);
    EEPROM.write(mem1,xlow);
    EEPROM.write(mem2,xhigh);
#ifdef SIMULATION
    EEPROM.end();
#else
    EEPROM.commit();
#endif

    if(numToSave == read16BitFromEEPROM(mem1,mem2)){
        // DEBUG_PRINT("Successfully saved 16 Bit Integer to EEPROM"); // This can be done wherever this function is callled if necessary
        return true;
    }else{
        DEBUG_PRINT("Failed to save 16 Bit Integer to EEPROM");
        return false;
    }

}



// Below this is either Unimplemented, or will likely not be used for this module ///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
    The control loop that the state machine runs when in MODE_VELOCITY
*/
void BasicModule::controlLoopVelocity(){
    // Implement velocity control loop (Likely not needed, will be controlled by main robot controller)
}

/*
    Sets the Velocity Kp, Ki, Kd values to the given values, returns the updated Kp, Ki, Kd. 
    Any given value that is <0 will not change the current value
    @param float Kp : The new Kp value (<0 doesnt change current Kp) 
    @param float Ki : The new Ki value (<0 doesnt change current Ki)
    @param float Kd : The new Kd value (<0 doesnt change current Kd) 
*/
void BasicModule::setVelKpKiKd(float Kp, float Ki, float Kd){
    if(Kp >= 0){
        this->velKp = Kp;
    }
    if(Ki >= 0){
        this->velKi = Ki;
    }
    if(Kd >= 0){
        this->velKd = Kd;
    }
}

/*
    Getter for Velocity Kp, Ki, Kd
    @param  float*  : A 3x1 Array to populate with the new Kp, Ki, Kd values if desired (if only changing one PID value, this can be useful)
*/
void BasicModule::getVelKpKiKd(float* retVal){
    retVal[0] = this->velKp;
    retVal[1] = this->velKi;
    retVal[2] = this->velKd;
}


/* 
    NOT IMPLEMENTED Sets the velocity of the joint in centidegrees/second.
    @param int16_t velocityCentidegrees : The velocity in centidegrees to set the joint to
*/
void BasicModule::setVelocity(int16_t velocityCentidegrees){
    this->desiredVelocityCentidegrees = velocityCentidegrees;
}


/*
    Gets the current velocity of the joint in centidegrees/second.
    @return int16_t : The velocity in centidegrees/second 
*/
int16_t BasicModule::getVelocity(){
    return this->currentVelocityCentidegrees;
}
