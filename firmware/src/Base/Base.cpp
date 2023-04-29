#include <Arduino.h>
#include "Command.h"
#include "Base.h"
#include "Communication.h"
#include "roundRobinComms.h"
#include "DebugPrint.h"


Base::Base(): rrc2(2,115200,BYTES_REQUIRED), cr(0,115200){
}


void Base::setup() {
    //Initialize the Serial 2 Comms
    this->rrc2.begin();
    cr.begin();
}

void Base::loop(){
  this->monitorComms();


}


void Base::monitorCommandInputs(){
  if(this->cr.monitorForCommands()){
    this->cr.getCommand(this->command);
  }

  Communication C;

  switch(convertStringCommandToCommRequest(this->command[0])){
    case SET_CONFIGURATON_REQUEST:
      C.commType = SET_CONFIGURATON_REQUEST;
      C.moduleNum = -1; // ALL MODULES
      for(uint8_t i=0;i<sizeof(C.data) / sizeof(C.data[0]); i++){
          C.data[i] = -1;
      }
    break;
  }
}

CommRequests Base::convertStringCommandToCommRequest(String str){
  if(this->command[0] == "SETCONFIG"){
    return SET_CONFIGURATON_REQUEST;
  }
}

void Base::monitorComms(){
  this->rrc2.loop();

  if(rrc2.newCommandReceived()){
    Communication C = rrc2.decipherCommand();
    DEBUG_PRINT("Communication Received over Serial 2: ");
    DEBUG_PRINT(C.commType);
    DEBUG_PRINT(C.moduleNum);
    for(int i=0;i<10;i++){
        DEBUG_PRINT(C.data[i]);
    }

    // Handle the communication when it is received, and save the response
    this->handleReceivedCommunication(C);
  }
}

void Base::startCommunication(Communication C){
  rrc2.sendCommunication(C);
}




void Base::handleReceivedCommunication(Communication C){
  switch(C.commType){
    case SET_CONFIGURATON_REQUEST:
        for(uint8_t i=0;i<sizeof(C.data) / sizeof(C.data[0]); i++){
          if(C.data[i] == -1){
            this->nJoints == i;
            break;
          }
          this->modules[i].orientation = C.data[i];
        }
    break;
    case GET_CONFIGURATON_REQUEST:
        // For each module
        for(uint8_t i = 0; i < this->nJoints; i++){
          this->modules[i].orientation = C.data[i];
        }
    break;
    case SET_DESIRED_POSITION_REQUEST:
    case GET_DESIRED_POSITION_REQUEST:
        // For each module
        for(uint8_t i = 0; i < this->nJoints; i++){
          this->modules[i].desiredPositionCentidegrees = C.data[i];
        }
    break;
    case GET_CURRENT_POSITION_REQUEST:
        // For each module
        for(uint8_t i = 0; i < this->nJoints; i++){
          this->modules[i].currentPositionCentidegrees = C.data[i];
        }
    break;
    case SET_EFFORT_REQUEST:
    case GET_EFFORT_REQUEST:
        // For each module
        for(uint8_t i = 0; i < this->nJoints; i++){
          this->modules[i].currentEffort = C.data[i];
        }
    break;
    case SET_POS_KP_KI_KD_REQUEST:
    case GET_POS_KP_KI_KD_REQUEST:
        // Only for one module unless moduleNum == -1
        if(C.moduleNum != -1){
          this->modules[C.moduleNum - 1].posKp = C.data[0];
          this->modules[C.moduleNum - 1].posKi = C.data[1];
          this->modules[C.moduleNum - 1].posKd = C.data[2];
        }else{ // update for all if it was set for all
          for(uint8_t i = 0; i < this->nJoints; i++){
            this->modules[i].posKp = C.data[0];
            this->modules[i].posKi = C.data[1];
            this->modules[i].posKd = C.data[2];
          }
        }
    break;
    case GET_CURRENT_VELOCITY_REQUEST:
        // For each module
        for(uint8_t i = 0; i < this->nJoints; i++){
          this->modules[i].currentVeloityCentidegrees = C.data[i];
        }
    break;
    case GET_CURRENT_ACCELERATION_REQUEST:
        // For each module
        for(uint8_t i = 0; i < this->nJoints; i++){
          this->modules[i].currentAccelerationCentidegrees = C.data[i];
        }
    break;
}
}

