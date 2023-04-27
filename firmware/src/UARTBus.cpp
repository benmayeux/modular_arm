#include <Arduino.h>
#include <Command.h>
#include "UARTBusDataDelegate.h"
#include "Configuration.h"
#include "DebugPrint.h"
#include "UARTBus.h"



    int UARTBus::available() {
      return serialPort->available();
    }
    /**
     * @brief Construct a new UARTBus object
     *
     * @param delegateIn A data delegate based on current command on the bus
     */
    UARTBus::UARTBus(UARTBusDataDelegate* delegateIn, Stream* in, Stream* out) {
      delegate = delegateIn;
      serialPort = (HardwareSerial*)(new SerialAdapter(in, out));
    }

    UARTBus::UARTBus() {
      UARTBus(0, nullptr, nullptr);
    }

   
  

    /**
     * @brief Sends a command on the bus
     *
     * @param c
     */
    void UARTBus::sendCommand(Command c) {
      sendData(c);
    }

   

    /**
     * @brief Consumes a raw command from the bus
     *
     * @return Command
     */
    Command UARTBus::receiveCommand() {
      return receiveData<Command>();
    }

    /**
     * @brief Consumes a Configuration object from the bus
     *
     * @return Configuration
     */
    Configuration UARTBus::receiveConfiguration() {
      return receiveData<Configuration>();
    }

    /**
     * @brief Handles forwarding and data fetching. Returns any new commands to be processed
     *
     * @return Command
     */
    Command UARTBus::handleCommunication() {
      Command currentCommand = Command();
      currentCommand.command = CommandType::NOOP;
      while(serialPort->available()) {
          currentCommand = receiveCommand();

          // Configure Bus
          if (currentCommand.command == CommandType::CONFIGURE) {
            DEBUG_PRINT((String) address + ": Starting configure...");
            address = currentCommand.address++;
            sendCommand(currentCommand);
            Configuration c = delegate->getConfiguration();
            forwardAndAppend(c, address);
            currentCommand.command = CommandType::NOOP;
            DEBUG_PRINT((String) address + ": Done!");
            return currentCommand;
          }

          // For this joint
          if (currentCommand.address == address) {
            currentCommand = currentCommand;
            Command forwarded = Command(currentCommand);
            delegate->fetchData(currentCommand.command, forwarded.data);
            sendCommand(forwarded);
            return currentCommand;
          }

          if (currentCommand.command & CAROUSEL) {
            int nJoints = (int)currentCommand.data[0];
            // Forward original command
            sendCommand(currentCommand);
            int16_t* dataBuffer = new int16_t[currentCommand.getNReturn()];
            byte nDataOut = delegate->fetchData(currentCommand.command, dataBuffer);
            leftShiftBus(nJoints, currentCommand.data, 1, dataBuffer, nDataOut);
            delete[] dataBuffer;

            // Forward data
            return currentCommand;
          }

          // Forward
          sendCommand(currentCommand);
          currentCommand.command = CommandType::NOOP;
          return currentCommand;
        }
        return currentCommand;
    }


