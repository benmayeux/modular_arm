#ifndef DATA_DELEGATE_H
#define DATA_DELEGATE_H

#include "Command.h"
#include "Configuration.h"

/**
 * @brief Implemented in UARTBus clients to forward data on-demand
 *
 */
class UARTBusDataDelegate {
public:
    virtual byte fetchData(CommandType command, int16_t* dataBuffer) = 0;
    virtual Configuration getConfiguration() = 0;
};

#endif