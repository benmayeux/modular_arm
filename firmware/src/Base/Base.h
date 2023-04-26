#ifndef BASE_H
#define BASE_H

#include "SerialInterface.h"
#include "Configuration.h"
#include "Command.h"
#include "UARTBus.h"

namespace base {
  class Base {
    private:
      SerialInterface serialInterface;
    public:
      static const int COM_PORT = 2;
      uint8_t nJoints;
      Configuration* config;
      UARTBus bus;

      void setup();
      bool fetchConfiguration(int timeout = 10000);

      Command pollData(int16_t *buffer);

      Command sendCarouselCommand(CommandType commandType, int16_t *dataIn, int16_t *dataOut, int nDataOut, int length);

      Command sendToJoint(CommandType commandType, int address, int16_t data);
      Command sendEffort(int address, int16_t data);
      Command sendPosition(int address, int16_t data);

      Command sendCarouselPosition(int16_t *data, int length, int16_t *dataOut);

      int calculateIK(int16_t* output, int16_t x, int16_t y, int16_t z);
      Command sendCarouselPoll(int length, int16_t *dataOut);
      void executeCommand(SerialInputCommand command);
      void loop();
  };
};
#endif