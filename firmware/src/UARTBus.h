#ifndef UARTBUS_H
#define UARTBUS_H

#include <Arduino.h>
#include <Command.h>
#include "UARTBusDataDelegate.h"
#include "Configuration.h"

class UARTBus {
  public: 
    UARTBusDataDelegate* delegate;
    int address;
    
    /**
     * @brief Construct a new UARTBus object
     * 
     * @param delegateIn A data delegate based on current command on the bus
     */
    UARTBus(UARTBusDataDelegate* delegateIn) {
      delegate = delegateIn;
    }

    UARTBus() {
      UARTBus(0);
    } 

    //TODO: Templatize after we don't need debugging
    void sendData(byte* data, int size) {
      Serial.write(data, size);
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
      Serial.write((uint8_t*)&c, sizeof(Command));
    }

    /**
     * @brief Consumes a raw float from the bus
     * 
     * @return float 
     */
    float receivefloat() {
      float data;
      Serial.readBytes((uint8_t*)&data, sizeof(data));
      return data;
    }

    /**
     * @brief Consumes a raw command from the bus
     * 
     * @return Command 
     */
    Command receiveCommand() {
      Command data;
      Serial.readBytes((uint8_t*)&data, sizeof(data));
      return data;
    }

    /**
     * @brief Consumes a Configuration object from the bus
     * 
     * @return Configuration 
     */
    Configuration receiveConfiguration() {
      Configuration data;
      Serial.readBytes((uint8_t*)&data, sizeof(data));
      return data;
    }
   
    /**
     * @brief Sends a configuration block on the bus
     * 
     * @param data 
     */
    void sendConfiguration(Configuration data) {
      Serial.write((uint8_t*)&data, sizeof(data));
    }

    /**
     * @brief Handles forwarding and data fetching. Returns any new commands to be processed
     * 
     * @return Command 
     */
    Command handleCommunication() {
      Command currentCommand = Command();
      currentCommand.command = CommandType::NOOP;
      while(Serial.available()) {
          currentCommand = receiveCommand();
          
          // Don't forward
          if (currentCommand.address == address) {
            currentCommand = currentCommand;
            return currentCommand;
          } 

          // Configure Bus
          if (currentCommand.command == CommandType::CONFIGURE) {
            address = currentCommand.address++;
            sendCommand(currentCommand);
            Configuration c = delegate->getConfiguration();
            forwardAndAppend(c, address);
            return currentCommand;
          }

          if (currentCommand.command & ROUND_ROBIN_BIT) {
            int size = currentCommand.data;
            // Forward original command
            sendCommand(currentCommand);

            // Forward data
            currentCommand.data = leftShiftBus(size, delegate->fetchData(currentCommand.command));
            return currentCommand;
          } 

          // Forward
          sendCommand(currentCommand);
          currentCommand.command = CommandType::NOOP;
          return currentCommand;
        }
        return currentCommand;
    }
};

#endif