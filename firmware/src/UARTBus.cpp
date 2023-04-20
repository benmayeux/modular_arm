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
    UARTBus::UARTBus(UARTBusDataDelegate* delegateIn, int uartPort) {
      delegate = delegateIn;
      switch(uartPort) {
        case 0:
          serialPort = &Serial;
          break;
        case 1:
          serialPort = &Serial1;
          break;
        case 2:
          serialPort = &Serial2;
          break;
        default:
          serialPort = &Serial;
          break;
      }
      serialPort->begin(115200);
    }

    UARTBus::UARTBus() {
      UARTBus(0, 1);
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
          DEBUG_PRINT("received " + (String)currentCommand.command);
         
          // Configure Bus
          if (currentCommand.command == CommandType::CONFIGURE) {
            DEBUG_PRINT("Starting configure...");
            address = currentCommand.address++;
            sendCommand(currentCommand);
            Configuration c = delegate->getConfiguration();
            DEBUG_PRINT("forwarding");
            forwardAndAppend(c, address);
            currentCommand.command = CommandType::NOOP;
            return currentCommand;
          }

          // For this joint 
          if (currentCommand.address == address) {
            currentCommand = currentCommand;
            Command forwarded = Command(currentCommand);
            forwarded.data = delegate->fetchData(currentCommand.command);
            sendCommand(forwarded);
            return currentCommand;
          }

         

          if (currentCommand.command & CAROUSEL) {
            int size = currentCommand.data;
            // Forward original command
            sendCommand(currentCommand);

            // Forward data
            currentCommand.data = leftShiftBus(size, delegate->fetchData(currentCommand.command));
            return currentCommand;
          }

          // Forward
          sendCommand(currentCommand);
          DEBUG_PRINT("forwarding to: " + (String)currentCommand.address);
          currentCommand.command = CommandType::NOOP;
          return currentCommand;
        }
        return currentCommand;
    }


