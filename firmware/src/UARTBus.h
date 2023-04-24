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
    
    int available();
    /**
     * @brief Construct a new UARTBus object
     *
     * @param delegateIn A data delegate based on current command on the bus
     */
    UARTBus(UARTBusDataDelegate* delegateIn, int uartPort);
    UARTBus();

    /**
     * @brief Sends a command on the bus
     *
     * @param c
     */
    void sendCommand(Command c);


    template <typename T> void sendData(T data) {
      serialPort->write((uint8_t*)&data, sizeof(data));
    }

    template <typename T> T leftShiftBus(int size, T data){
      T val = receiveData<T>();
      size--;
      while(size--) {
        T v = receiveData<T>();;
        sendData(v);
      }
      sendData(val);
      return val;
    }

    /**
     * @brief Forwards all data on bus and appends new data
     * 
     * @param data 
     * @param size 
     */
    template <typename T> void forwardAndAppend(T data, uint8_t size) {
      size--;
      while(size--) {
        sendData(receiveData<T>());
      }
      sendData(data);
    }

        /**
     * @brief Consumes a raw float from the bus
     *
     * @return float
     */
    template <typename T> T receiveData() {
      T data;
      serialPort->readBytes((uint8_t*)&data, sizeof(data));
      return data;
    }

    /**
     * @brief Consumes a raw command from the bus
     *
     * @return Command
     */
    Command receiveCommand();

    /**
     * @brief Consumes a Configuration object from the bus
     *
     * @return Configuration
     */
    Configuration receiveConfiguration();

    /**
     * @brief Handles forwarding and data fetching. Returns any new commands to be processed
     *
     * @return Command
     */
    Command handleCommunication();
};

#endif