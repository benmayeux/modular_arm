#include <Arduino.h>
#include "Command.h"
#include "UARTBus.h"
#include "DebugPrint.h"
#include "SerialInterface.h"
#include "Base.h"

namespace base {
  void Base::setup() {
      serialInterface = SerialInterface();
      serialInterface.begin(115200);
      delay(300);
 
      bus = UARTBus(0, &Serial2, &Serial2);
      fetchConfiguration();
  }

  bool Base::fetchConfiguration(int timeout) {
    Command c = Command();
    c.command = CommandType::CONFIGURE;
    // starting address
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
    if(config) {
      delete[] config;
      config = NULL;
    }
    config = new Configuration[nJoints];

    for(int n = 0; n < nJoints; n++) {
      config[n] = bus.receiveConfiguration();
      DEBUG_PRINT("joint " + (String)n + ":");
      DEBUG_PRINT("    length: " + (String)config[n].length);
      DEBUG_PRINT("    orientation: " + (String)config[n].orientation);
    }
    return true;
  }


  Command Base::sendCarouselCommand(CommandType commandType, int16_t* dataIn, int16_t* dataOut, int nDataOut, int length) {
    DEBUG_PRINT("sending carousel: " + (String)commandType + ": " + (String)length);
    Command command = Command();
    command.command = (CommandType)(commandType | CommandType::CAROUSEL);
    if (length != nJoints) {
      DEBUG_PRINT("ERROR: size of carousel data does not match number of joints!");
      return command;
    }
    command.data[0] = nJoints;
    bus.sendCommand(command);
    for (int i = 0; i < length; i++) {
      bus.sendData(dataIn[i]);
    }
    command = bus.receiveCommand();
    DEBUG_PRINT(command.command);
    for (int i = 0; i < length*nDataOut; i++) {
      dataOut[i] = bus.receiveData<int16_t>();
    }
    return command;
  }

  // TODO: async receive
  Command Base::sendToJoint(CommandType commandType, int address, int16_t data) {
    DEBUG_PRINT("Sending: " + (String)commandType + ":" + (String)data + " to joint: " + (String)address);
    Command command = Command();
    command.command = commandType;
    command.data[0] = data;
    command.address = address;
    bus.sendCommand(command);
    command = bus.receiveCommand();
    DEBUG_PRINT(command.command);
    DEBUG_PRINT("received back: " + (String) command.data[0] + ": " + (String) command.getNReturn());
    return command;
  }

  Command Base::sendEffort(int address, int16_t data) {
    return sendToJoint(CommandType::EFFORT_WRITE, address, data);
  }

  Command Base::sendPosition(int address, int16_t data) {
    return sendToJoint((CommandType)(CommandType::POSITION_WRITE | CommandType::RETURN_POSITION | CommandType::RETURN_VELOCITY | CommandType::RETURN_EFFORT), address, data);
  }

  Command Base::sendCarouselPosition(int16_t* data, int length, int16_t* dataOut) {
    return sendCarouselCommand((CommandType)(CommandType::POSITION_WRITE_CAROUSEL | CommandType::RETURN_POSITION | CommandType::RETURN_EFFORT | CommandType::RETURN_VELOCITY), data, dataOut, 3, length);
  }

  // TODO: actual IK
  int Base::calculateIK(int16_t* output, int16_t x, int16_t y, int16_t z) {
    DEBUG_PRINT("Setting EE pos to: (" + (String)x + ", " + (String)y + ", " + (String)z + ")");
    for (int i = 0; i < nJoints; i++) {
      output[i] = i + x + y + z;
    }
    return nJoints;
  }

  void Base::executeCommand(SerialInputCommand command) {
    int n = 0;
    int16_t dataBuffer[40];
    Command c;
    switch (command.commandType) {
      case SerialInputCommandType::RECONFIGURE:
        fetchConfiguration();
        break;
      case SerialInputCommandType::SET_JOINT_POSITION:
        c = sendPosition(command.data[0], command.data[1]);
        n = c.getNReturn();
        Serial.println("1,JOINT,POS,EFF,VEL");
        Serial.print((String)c.address + ",");
        for (int i = 0; i < n; i++) {
          Serial.print(c.data[i]);
          Serial.print(",");
        }
        Serial.println();
        break;
      case SerialInputCommandType::SET_TASK_POSITION:
        n = calculateIK(dataBuffer, command.data[0],command.data[1],command.data[2]);
        c = sendCarouselPosition(dataBuffer, n, dataBuffer);
        Serial.println((String)n + ",JOINT,POS,EFF,VEL");
        for (int i = 0; i < n; i++) {
          Serial.print((String)i + ",");
          for (int j = 0; j < c.getNReturn(); j++) {
            Serial.print((String)dataBuffer[c.getNReturn()*i + j] + ",");
          }
          Serial.println();
        }
        break;
      case SerialInputCommandType::INVALID:
        DEBUG_PRINT("Invalid command!");
        break;
      case SerialInputCommandType::NONE:
        break;
      default:
        DEBUG_PRINT("Unexpected commandType: " + (String)command.commandType);
        break;
    }
  }

  void Base::loop()
  {
    SerialInputCommand command = serialInterface.parseCommand();
    if (command.commandType != SerialInputCommandType::NONE) {
      executeCommand(command);
    }
    delay(100);
  }
};

