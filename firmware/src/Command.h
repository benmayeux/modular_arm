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
    CAROUSEL = 1 << 3,

    // bit 4-6
    RETURN_MASK = 0x70,
    RETURN_POSITION = 1 << 4,
    RETURN_EFFORT = 1 << 5,
    RETURN_VELOCITY = 1 << 6,
    // bit 7
    __RESERVED_BIT2 = 0x80,

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
    int16_t data[4];
    byte address;
    byte nDataOut;
    CommandType getCommandTarget() {
        return (CommandType) (command & COMMANDTYPE_MASK);
    }
    bool isCarousel() {
        return command & CAROUSEL;
    }
    int getNReturn() {
        byte c = ((byte)command & (byte)RETURN_MASK);
        int count = 0;
        for (int bit = 1; bit < 1 << 8; bit = bit << 1) {
            if (bit & c) {
                count++;
            }
        }
        return count;
    }
};


#endif