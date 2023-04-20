#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H
#include <Arduino.h>
#include "SerialInputCommand.h"


namespace base {
    static const String SERIAL_COMMAND_JOINT_POSITION = "SETJOINTPOSITION";
    static const String SERIAL_COMMAND_TASK_POSITION = "SETTASKPOSITION";
    static const String SERIAL_COMMAND_RECONFIGURE = "RECONFIGURE";

    class SerialInterface {
    public:
        void begin(unsigned long baud);
        SerialInputCommand parseCommand();
    };
}
#endif