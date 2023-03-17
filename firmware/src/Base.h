#ifndef BASE_H
#define BASE_H

#include <Arduino.h>
#include "Command.h"
#include "UARTBus.h"


class Base {
  public:
    uint8_t nJoints;
    Configuration* config;
    UARTBus bus = UARTBus();

    void setup() {
      Serial.begin(9600);
      fetchConfiguration();
    }

    bool fetchConfiguration(int timeout = 10000) {
      Command c = Command();
      c.command = CommandType::CONFIGURE;
      c.address = 1;

      bus.sendCommand(c);
      int elapsed = 0;
      while(!Serial.available()) {
        elapsed += 100;
        delay(100);
        if(elapsed >= timeout) {
          return false;
        }
      }
      c = bus.receiveCommand();
      nJoints = c.address - 1;
      config = new Configuration[nJoints];
      
      for(int n = 0; n < nJoints; n++) {
        config[n] = bus.receiveConfiguration();
      }
      return true;
    }
  
    void loop()
    {
      Command command = Command();
      command.command = (CommandType) (CommandType::POSITION_WRITE | CommandType::ROUND_ROBIN_BIT);
      command.data = nJoints;
      bus.sendCommand(command);
      double data = 1.337;
      bus.sendData((byte *)&data, sizeof(data));
      
      delay(100);
    }

};

#endif