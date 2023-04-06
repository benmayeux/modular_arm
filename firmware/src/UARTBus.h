#ifndef UARTBUS_H
#define UARTBUS_H

#include <Arduino.h>
#include <Command.h>
#include "UARTBusDataDelegate.h"
#include "Configuration.h"
#include "DebugPrint.h"

class UARTBus {
  public:
    UARTBusDataDelegate* delegate;
    int address = -1;
    HardwareSerial* serialPort;
    int available() {
      return serialPort->available();
    }
    /**
     * @brief Construct a new UARTBus object
     *
     * @param delegateIn A data delegate based on current command on the bus
     */
    UARTBus(UARTBusDataDelegate* delegateIn, int uartPort) {
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

    UARTBus() {
      UARTBus(0, 1);
    }

    //TODO: Templatize after we don't need debugging
    void sendData(byte* data, int size) {
      serialPort->write(data, size);
    }

    // TODO: templatize
    float leftShiftBus(int size, float data){
      float val = receivefloat();
      size--;
      while(size--) {
        float v = receivefloat();;
        sendData((byte *)&v, sizeof(v));
      }
      sendData((byte *)&val, sizeof(val));
      return val;
    }
    
    // TODO: templatize
    /**
     * @brief Forwards all data on bus and appends new data
     * 
     * @param data 
     * @param size 
     */
    void forwardAndAppend(Configuration data, uint8_t size) {
      size--;
      while(size--) {
        sendConfiguration(receiveConfiguration());
      }
      sendConfiguration(data);
    }

    /**
     * @brief Sends a command on the bus
     *
     * @param c
     */
    void sendCommand(Command c) {
      serialPort->write((uint8_t*)&c, sizeof(Command));
    }

    /**
     * @brief Consumes a raw float from the bus
     *
     * @return float
     */
    float receivefloat() {
      float data;
      serialPort->readBytes((uint8_t*)&data, sizeof(data));
      return data;
    }

    /**
     * @brief Consumes a raw command from the bus
     *
     * @return Command
     */
    Command receiveCommand() {
      Command data;
      serialPort->readBytes((uint8_t*)&data, sizeof(data));
      return data;
    }

    /**
     * @brief Consumes a Configuration object from the bus
     *
     * @return Configuration
     */
    Configuration receiveConfiguration() {
      Configuration data;
      serialPort->readBytes((uint8_t*)&data, sizeof(data));
      return data;
    }

    /**
     * @brief Sends a configuration block on the bus
     *
     * @param data
     */
    void sendConfiguration(Configuration data) {
      serialPort->write((uint8_t*)&data, sizeof(data));
    }

    /**
     * @brief Handles forwarding and data fetching. Returns any new commands to be processed
     *
     * @return Command
     */
    Command handleCommunication() {

      Command currentCommand = Command();
      currentCommand.command = CommandType::NOOP;
      while(serialPort->available()) {
          currentCommand = receiveCommand();
          DEBUG_PRINT("received " + (String)currentCommand.command);
          // Don't forward
          if (currentCommand.address == address) {
            DEBUG_PRINT("it's for me");
            currentCommand = currentCommand;
            DEBUG_PRINT("data: " + (String)currentCommand.data);
            Command forwarded = Command(currentCommand);
            forwarded.data = delegate->fetchData(currentCommand.command);
            sendCommand(forwarded);
            return currentCommand;
          }
          DEBUG_PRINT(currentCommand.getCommandTarget());
          // Configure Bus
          if (currentCommand.command == CommandType::CONFIGURE) {
            DEBUG_PRINT("Starting configure...");
            address = currentCommand.address++;
            sendCommand(currentCommand);
            Configuration c = delegate->getConfiguration();
            DEBUG_PRINT("forwarding");
            forwardAndAppend(c, address);
            currentCommand.command == CommandType::NOOP;
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
};

#endif