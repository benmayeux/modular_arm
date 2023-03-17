#ifndef MODULE_H
#define MODULE_H

#include "Command.h"
#include "UARTBus.h"
#include "UARTBusDataDelegate.h"

class Module: public UARTBusDataDelegate {
  public:

    enum Mode {
        TORQUE,
        POSITION
    };
   
    float torque;
    float torqueSetPoint;
    float position;
    float positionSetPoint;
    float velocity;
    float velocitySetPoint;

    Mode operationMode;
    UARTBus bus;
    Command currentCommand;

    float fetchData(CommandType command) {
        command = (CommandType)(command & CommandType::RETURN_MASK);
        switch(command) {
            case CommandType::RETURN_POSITION:
                return position;
            case CommandType::RETURN_TORQUE:
                return torque;
            case CommandType::RETURN_VELOCITY:
                return velocity;
            default:
                return position;
            
        }
    }

    Configuration getConfiguration() {
        Configuration c = Configuration();
        c.address = bus.address;
        c.length = 0x10;
        c.orientation = 0x01;
        return c;
    }

    void setup()
    {
        Serial.begin(9600);
        bus = UARTBus(this);
    }

    // Update relevant member data, set mode, forward info
    void processCommand(Command c) {
        switch(c.command) {
        case CommandType::TORQUE_WRITE:
            operationMode = Mode::TORQUE;
            torqueSetPoint = c.data;
            break;
        case CommandType::POSITION_WRITE:
            operationMode = Mode::POSITION;
            positionSetPoint = c.data;
            break;
        default:
            break;
        }
    }
    

    // TODO: probably implement strategy pattern instead of state machine
    void handlePositionSetpoint() {
    
    }

    void handleTorqueSetpoint() {

    }


    // State Machine
    void runControls() {
        switch (operationMode) {
            case Mode::POSITION:
                handlePositionSetpoint();
                break;
            case Mode::TORQUE:
                handleTorqueSetpoint();
                break;
        }
    }

    void loop()
    {
        currentCommand = bus.handleCommunication();
        processCommand(currentCommand);
        runControls();
        delay(10);
    }
};

#endif