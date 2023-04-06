#ifndef COMMAND_H
#define COMMAND_H

enum CommandType {
    // bits 0-2
    COMMANDTYPE_MASK = 0x07,
    NOOP = 0x00,
    POSITION = 0x01,
    VELOCITY = 0x02,
    EFFORT = 0x03,
    CONFIGURATION = 0x04,
    __RESERVED_COMMAND1 = 0x05,
    __RESERVED_COMMAND2 = 0x06,
    __RESERVED_COMMAND3 = 0x07,

    CONTROL_MASK = 0x18,
    // bit 3
    WRITE_BIT = 0x08,
    // bit 4
    ROUND_ROBIN_BIT = 0x10,

    // bit 5-6
    RETURN_MASK = 0x60,
    RETURN_POSITION = POSITION << 5,
    RETURN_EFFORT = EFFORT << 5,
    RETURN_VELOCITY = VELOCITY << 5,
    // bit 7
    __RESERVED = 0x80,

    // full commands
    POSITION_READ = POSITION,
    EFFORT_READ = EFFORT,
    CONFIGURE = CONFIGURATION | WRITE_BIT | ROUND_ROBIN_BIT,
    POSITION_WRITE = POSITION | WRITE_BIT,
    RR_POSITION_WRITE = POSITION_WRITE | ROUND_ROBIN_BIT,
    EFFORT_WRITE = EFFORT | WRITE_BIT,
    RR_EFFORT_WRITE = EFFORT | WRITE_BIT | ROUND_ROBIN_BIT,
};

struct Command {
    CommandType command;
    double data;
    byte address;
};


#endif