#ifndef BASE_H
#define BASE_H

#include "roundRobinComms.h"
#include "Configuration.h"
#include "Command.h"
#include "commandReader.h"

class Base {
  private:
    roundRobinComms rrc2;
    commandReader cr;

    void monitorComms();
    void handleReceivedCommunication(Communication C);
    void startCommunication(Communication C);
    void monitorCommandInputs();
    BasicModuleData modules[10];
    CommRequests convertStringCommandToCommRequest(String str);

    // Initialize the command value for the Command Reader
    String command[20] = {"N/A"};

  public:
    Base();
    uint8_t nJoints;
    Configuration config;
    void setup();
    void loop();
  };



struct BasicModuleData{
  uint8_t moduleNum = -1;
  float length;
  bool orientation;
  int16_t desiredPositionCentidegrees;
  int16_t currentPositionCentidegrees;
  int8_t currentEffort;
  float posKp;
  float posKi;
  float posKd;
  int16_t currentVeloityCentidegrees;
  int16_t currentAccelerationCentidegrees;
};



#endif