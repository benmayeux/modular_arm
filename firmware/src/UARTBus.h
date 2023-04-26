#ifndef UARTBUS_H
#define UARTBUS_H

#include <Arduino.h>
#include <Command.h>
#include "UARTBusDataDelegate.h"
#include "Configuration.h"
#include "DebugPrint.h"
#include "SerialAdapter.h"

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
    UARTBus(UARTBusDataDelegate* delegateIn, Stream* in, Stream* out);
    UARTBus();

    /**
     * @brief Sends a command on the bus
     *
     * @param c
     */
    void sendCommand(Command c);


    template <typename T> void sendData(T data) {
      DEBUG_PRINT("rawSend: " + (String)sizeof(data));
      serialPort->write((uint8_t*)&data, sizeof(data));
    }

    /**
     * @brief shift through [in, in, out, out, out]
     * in this case, nJoints = 5
     * address = 4
     * take one in off to [in, out, out, out]
     *    send one in
     *    send 3 outs
     *    send 1 out of own
     * 
     * @tparam TIN 
     * @tparam TOUT 
     * @param nJoints 
     * @param data 
     * @return TOUT 
     */
    byte leftShiftBus(int nJoints, int16_t* dataIn, byte nDataIn, int16_t* dataOut, byte nDataOut) {
      DEBUG_PRINT("left shift");
      DEBUG_PRINT(nJoints);
      DEBUG_PRINT(nDataIn);
      for (int i = 0; i < nDataIn; i++) {
        dataIn[i] = receiveData<int16_t>();
        DEBUG_PRINT("Got : " + (String)dataIn[i]);
      }

      int nForwardWords = nDataOut * (address - 1) + nDataIn * (nJoints - address);
      DEBUG_PRINT(nForwardWords);
      while (nForwardWords > 0) {
        int16_t d = receiveData<int16_t>();
        DEBUG_PRINT("forwarding " + (String)d);
        sendData(d);
        nForwardWords--;
      }

      DEBUG_PRINT(nDataOut);
      for (int i = 0; i < nDataOut; i++) {
        sendData(dataOut[i]);
        DEBUG_PRINT("sending " + (String)dataOut[i]);
      }
      return nDataIn;
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