#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H
#include <Arduino.h>
#include "SerialInputCommand.h"


namespace base {
    static const String SERIAL_COMMAND_JOINT_POSITION = "setJointPos";
    static const String SERIAL_COMMAND_TASK_POSITION = "setTaskPos";
    static const String SERIAL_COMMAND_RECONFIGURE = "reconfigure";
    static const String SERIAL_COMMAND_POLL = "poll";

    class SerialInterface {
    public:
        void begin(unsigned long baud);
        SerialInputCommand parseCommand();
    };
}
#endif