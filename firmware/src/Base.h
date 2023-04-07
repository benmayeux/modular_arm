#ifndef BASE_H
#define BASE_H

#include <Arduino.h>
#include "Command.h"
#include "UARTBus.h"
#include "DebugPrint.h"


class Base {
  public:
    uint8_t nJoints;
    Configuration* config;
    UARTBus bus;

    void setup() {
      Serial.begin(9600);
      bus = UARTBus(0, 1);
      fetchConfiguration();
    }

    bool fetchConfiguration(int timeout = 10000) {
      Command c = Command();
      c.command = CommandType::CONFIGURE;
      c.address = 1;
      DEBUG_PRINT("sending config...");
      DEBUG_PRINT(c.command);
      bus.sendCommand(c);
      int elapsed = 0;
      while(!bus.available()) {
        DEBUG_PRINT("waiting");
        elapsed += 100;
        delay(100);
        if(elapsed >= timeout) {
          return false;
        }
      }
      c = bus.receiveCommand();
      DEBUG_PRINT("got "+ (String)c.command);
      nJoints = c.address - 1;
      DEBUG_PRINT("joints: " + (String)nJoints);
      config = new Configuration[nJoints];

      for(int n = 0; n < nJoints; n++) {
        config[n] = bus.receiveConfiguration();
      }
      return true;
    }

    void loop()
    {
      DEBUG_PRINT("sending carousel pos 1.337");
      Command command = Command();
      command.command = CommandType::POSITION_WRITE_CAROUSEL;
      command.data = nJoints;
      bus.sendCommand(command);
      double data = 1.337;
      bus.sendData(data);
      DEBUG_PRINT(bus.receiveCommand().command);
      DEBUG_PRINT("received back: " + (String)bus.receiveData<float>());

      DEBUG_PRINT("sending effort 200.0");
      command = Command();
      command.address = 1;
      command.command = (CommandType)(CommandType::EFFORT_WRITE | CommandType::RETURN_VELOCITY);
      command.data = 200.0;
      bus.sendCommand(command);
      command = bus.receiveCommand();

      DEBUG_PRINT("received back: " + (String)command.data);

      delay(100);
    }

};

#endif