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


    // bit 3


    // bit 4
    CAROUSEL = 0x10,

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
    CONFIGURE = CONFIGURATION | CAROUSEL,
    POSITION_WRITE = POSITION,
    POSITION_WRITE_CAROUSEL = POSITION_WRITE | CAROUSEL,
    EFFORT_WRITE = EFFORT,
    EFFORT_WRITE_CAROUSEL = EFFORT | CAROUSEL,
};

struct Command {
    CommandType command;
    double data;
    byte address;
    CommandType getCommandTarget() {
        return (CommandType) (command & COMMANDTYPE_MASK);
    }
    bool isCarousel() {
        return command & CAROUSEL;
    }
};


#endif