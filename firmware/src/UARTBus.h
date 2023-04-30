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

    void startComms();

    void sendCommand(Command c, bool clearBuffer = false);


    template <typename T> void sendData(T data) {
      DEBUG_PRINT("tx: " + (String) sizeof(data));
      DEBUG_PRINT("sent: " + (String)serialPort->write((uint8_t*)&data, sizeof(data)));
     // serialPort->flush();
    }


    template <typename T> void sendData(T* data, int n) {
      DEBUG_PRINT("tx: " + (String) (sizeof(data[0]) * n));
      DEBUG_PRINT("sent: " + (String)serialPort->write((uint8_t*)data, sizeof(data[0]) * n));
     // serialPort->flush();
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
      DEBUG_PRINT(nDataIn);
      if (nDataIn > 0) {
        receiveData<int16_t>(dataIn, nDataIn);
      }

      int nForwardWords = nDataOut * (address - 1) + nDataIn * (nJoints - address);
      DEBUG_PRINT("Forwarding: " + (String)nForwardWords);
      DEBUG_PRINT(nDataOut);
      DEBUG_PRINT(address);
      DEBUG_PRINT(nDataIn);
      DEBUG_PRINT(nJoints);
      if (nForwardWords > 0) {
        int16_t* data = new int16_t[nForwardWords];
        receiveData<int16_t>(data, nForwardWords);
        DEBUG_PRINT("received data.");
        sendData<int16_t>(data, nForwardWords);

        delete[] data;
      }
      DEBUG_PRINT("Sending data");
      sendData<int16_t>(dataOut, nDataOut);
     // serialPort->flush();
      DEBUG_PRINT("done");
      return nDataIn;
    }

    /**
     * @brief Forwards all data on bus and appends new data
     * 
     * @param data 
     * @param size 
     */
    template <typename T> void forwardAndAppend(T data, uint8_t size) {
      T* d = new T[size];
      if(size-1) {
        DEBUG_PRINT((size-1));
        receiveData<T>(d, size-1);
      }
      d[size-1] = data;
      sendData<T>(d, size);
    }

        /**
     * @brief Consumes a raw float from the bus
     *
     * @return float
     */
    template <typename T> T receiveData() {
      T data;
      int timeout = 30;
      while(serialPort->available() == 0 && timeout--) {
        DEBUG_PRINT("waiting on serial for " + (String)sizeof(data));
        vTaskDelay(20);
      }
      if(timeout <= 0) {
        DEBUG_PRINT("Skipping");
        return data;
      }
      DEBUG_PRINT("avail " + (String)serialPort->available());
      serialPort->readBytes((uint8_t*)&data, sizeof(data));
      DEBUG_PRINT("rx :" + (String)sizeof(data));
      DEBUG_PRINT("avail " + (String)serialPort->available());
      return data;
    }

    template <typename T> int receiveData(T* buffer, int n) {
      T data;
      int timeout = 30;
      DEBUG_PRINT("avail " + (String)serialPort->available());
      while(serialPort->available() == 0 && timeout--) {
        DEBUG_PRINT("waiting on serial for " + (String)sizeof(data));
        vTaskDelay(20);
      }
      if(timeout <= 0) {
        DEBUG_PRINT("Skipping");
        return 0;
      }
      DEBUG_PRINT("avail " + (String)serialPort->available());
    
      int recv = serialPort->readBytes((uint8_t*)buffer, sizeof(data) * n);
      DEBUG_PRINT("rx :" + (String)recv);
      return recv;
    }
    /**
     * @brief Consumes a raw command from the bus
     *
     * @return Command
     */
    Command receiveCommand();


    static void handleCommunicationTask(void* params);

    /**
     * @brief Consumes a Configuration object from the bus
     *
     * @return Configuration
     */
    Configuration receiveConfiguration();

    void updateCommand(Command c);

    Command getCurrentCommand();

    /**
     * @brief Handles forwarding and data fetching. Returns any new commands to be processed
     *
     * @return Command
     */
    Command handleCommunication();

    private:
      QueueHandle_t commandQueue = xQueueCreate(100, sizeof(Command));
};

#endif