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
      bus = UARTBus(0, COM_PORT);
      fetchConfiguration();
  }

  bool Base::fetchConfiguration(int timeout) {
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
    if(config) {
      delete[] config;
      config = NULL;
    }
    config = new Configuration[nJoints];

    for(int n = 0; n < nJoints; n++) {
      config[n] = bus.receiveConfiguration();
    }
    return true;
  }

  // TODO: return data
  Command Base::sendCarouselCommand(CommandType commandType, int16_t* data, int length) {
    DEBUG_PRINT("sending carousel: " + (String)commandType + ": " + (String)length);
    Command command = Command();
    command.command = (CommandType)(commandType | CommandType::CAROUSEL);
    if (length != nJoints) {
      DEBUG_PRINT("ERROR: size of carousel data does not match number of joints!");
      return command;
    }
    command.data = nJoints;
    bus.sendCommand(command);
    for (int i = 0; i < length; i++) {
      bus.sendData(data[i]);
    }
    command = bus.receiveCommand();
    DEBUG_PRINT(command.command);
    for (int i = 0; i < length; i++) {
      DEBUG_PRINT((String)bus.receiveData<int16_t>());
    }
    return command;
  }

  // TODO: async receive
  Command Base::sendToJoint(CommandType commandType, int address, int16_t data) {
    DEBUG_PRINT("Sending: " + (String)commandType + ":" + (String)data + " to joint: " + (String)address);
    Command command = Command();
    command.command = commandType;
    command.data = data;
    command.address = address;
    bus.sendCommand(command);
    command = bus.receiveCommand();
    DEBUG_PRINT(command.command);
    DEBUG_PRINT("received back: " + (String) command.data);
    return command;
  }

  Command Base::sendEffort(int address, int16_t data) {
    return sendToJoint(CommandType::EFFORT_WRITE, address, data);
  }

  Command Base::sendPosition(int address, int16_t data) {
    return sendToJoint((CommandType)(CommandType::POSITION_WRITE | CommandType::RETURN_POSITION), address, data);
  }

  Command Base::sendCarouselPosition(int16_t* data, int length) {
    return sendCarouselCommand(CommandType::POSITION_WRITE_CAROUSEL, data, length);
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
    int16_t dataBuffer[10];
    switch (command.commandType) {
      case SerialInputCommandType::RECONFIGURE:
        fetchConfiguration();
        break;
      case SerialInputCommandType::SET_JOINT_POSITION:
        sendPosition(command.data[0], command.data[1]);
        break;
      case SerialInputCommandType::SET_TASK_POSITION:
        n = calculateIK(dataBuffer, command.data[0],command.data[1],command.data[2]);
        sendCarouselPosition(dataBuffer, n);
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

