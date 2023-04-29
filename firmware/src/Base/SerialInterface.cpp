#include <Arduino.h>
#include "SerialInputCommand.h"
#include "SerialInterface.h"
#include "DebugPrint.h"
namespace base {
    void SerialInterface::begin(unsigned long baud) {
        Serial.begin(baud);
    }

    SerialInputCommand SerialInterface::parseCommand() {
        SerialInputCommand result = SerialInputCommand();
        char input[100];
        if (Serial.available()) {
            strcpy(input, Serial.readStringUntil(';').c_str());
            DEBUG_PRINT(input);
            result.length = 0;
            String inputCommand = strtok(input, ",");
            if (inputCommand == SERIAL_COMMAND_JOINT_POSITION) {
                result.commandType = SerialInputCommandType::SET_JOINT_POSITION;
            } else if (inputCommand == SERIAL_COMMAND_TASK_POSITION) {
                result.commandType = SerialInputCommandType::SET_TASK_POSITION;
            } else if (inputCommand == SERIAL_COMMAND_RECONFIGURE) {
                result.commandType = SerialInputCommandType::RECONFIGURE;
            } else if (inputCommand == SERIAL_COMMAND_POLL) {
                result.commandType = SerialInputCommandType::POLL;
            } else if (inputCommand == SERIAL_COMMAND_SET_ALL_POSITION) {
                result.commandType = SerialInputCommandType::SET_ALL_POSITION;
            } else {
                result.commandType = SerialInputCommandType::INVALID;
                return result;
            }

            char* output;
            while (1) {
                output = strtok(NULL, ",");
                if (output == NULL) {
                    break;
                }
                result.data[result.length++] = atoi(output);
            }
            return result;
        }
        return result;
    }
};

