#ifndef __ROUNDROBINCOMMS_H__
#define __ROUNDROBINCOMMS_H__

#include <Arduino.h>
#include <serialReadWrite.h>
#include "Communication.h"



// This class will be stored within the BasicModule class
// The moduleData struct will have pointers to the data stored within BasicModule
// When data is requested through comms, data will be sent from data stored in struct
// When data is sent through comms, data will be stored by the BasicModule Class

class roundRobinComms{
    private:
        serialReadWrite srw;
        uint8_t comPort;
        uint32_t baudRate;
        uint8_t numBytesPerMsg;
        byte* command;
        
        //Bytes separated by commas: commType, targetModule, data1HighBytes, data1LowBytes, data2HighBytes, ..., data10LowButes
        bool isNewCommand = false;
    public:
        byte* encipheredCommand;

        roundRobinComms(uint8_t comPort, uint32_t baudRate, uint8_t numBytesPerMsg) : command(new byte[numBytesPerMsg]), encipheredCommand(new byte[numBytesPerMsg]), srw(comPort, baudRate, numBytesPerMsg){
            this->comPort = comPort;
            this->baudRate = baudRate;
            this->numBytesPerMsg = numBytesPerMsg;
            // this->srw = serialReadWrite(comPort,baudRate, numBytesPerMsg);
        }

        void begin(){
            this->srw.begin();
        }


        // Monitors for Serial Comms, 
        void loop(){
            // Needs to separate into command byte, and subsequent other bytes
            if(srw.monitorForCommands()){
                srw.getCommand(command);
                this->isNewCommand = true;
                // Serial.print("Received new command on Serial 2: ");
                // Serial.write(command,numBytesPerMsg);
                // Serial.println();
            }
        }

        // Used to check if there is a new command
        bool newCommandReceived(){
            return isNewCommand;
        }

        // With no input, it runs on the most recent command, or you can feed it the byte array to convert it
        Communication decipherCommand(){
            // Takes the command bytes and puts them into the Communication Struct
            this->isNewCommand = false;
            return decipherCommand(this->command);
        }

        Communication decipherCommand(byte* newCommand){
            // Takes the command bytes and puts them into the Communication Struct
            Communication newComm;
            newComm.commType = (CommRequests)(uint8_t) newCommand[0];
            newComm.moduleNum = (int8_t)newCommand[1];

            // Receive the remainder of the data
            for(uint8_t i = 2; i < numBytesPerMsg; i = i + 2){
                uint16_t data = newCommand[i] << 8 | newCommand[i+1];
                newComm.data[(i-2)/2] = data;
            }

            return newComm;
        }

        void sendCommunication(Communication C){
            this->encipherCommand(C);
            Serial.println("Command Enciphered");
            this->srw.forwardCommand(this->encipheredCommand);
        }


        byte* encipherCommand(Communication comm){
            this->encipheredCommand[0] = (byte)(uint8_t)comm.commType;
            this->encipheredCommand[1] = (byte)(int8_t)comm.moduleNum;

            // For each element in the comm.data array
            for(uint8_t i = 0; i < sizeof(comm.data) / sizeof(comm.data[0]); i++){
                this->encipheredCommand[(2*i)+2] = (byte) (comm.data[i] >> 8);
                this->encipheredCommand[(2*i)+3] = (byte) (comm.data[i]);
            }

            return this->encipheredCommand;
        }
};

#endif