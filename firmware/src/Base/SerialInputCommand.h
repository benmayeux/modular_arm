#ifndef SERIAL_INPUT_COMMAND_H
#define SERIAL_INPUT_COMMAND_H

namespace base {
    enum SerialInputCommandType {
        SET_JOINT_POSITION,
        SET_TASK_POSITION,
        INVALID,
        POLL,
        RECONFIGURE,
        NONE
    };

    class SerialInputCommand {
    public:
        int16_t data[20];
        uint8_t length;
        SerialInputCommandType commandType = SerialInputCommandType::NONE;
    };
}
#endif