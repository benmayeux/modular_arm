#ifndef __SERIALREADWRITE_H__
#define __SERIALREADWRITE_H__

#include <Arduino.h>
#include "Communication.h"


class serialReadWrite{
    private:
        bool debugEnable = false;
        String command = "";
        byte commandByte[BYTES_REQUIRED]; 
        uint8_t bytesUsed = BYTES_REQUIRED;
        uint8_t serialPort = 10;
        uint32_t baudRate = 115200;
    
    public:
        serialReadWrite(uint8_t serialPort, uint32_t baudRate, uint8_t bytesUsed){
            this->serialPort = serialPort;
            this->baudRate = baudRate;
            this->bytesUsed = bytesUsed;
        }
        serialReadWrite(){}

        // Begin on the correct serial port
        void begin(){
            if(this->serialPort == 0){
                Serial.begin(this->baudRate);
                Serial.println("Serial 0 Has Begun at a baud rate of: " + String(this->baudRate));
            }else if (this->serialPort == 1){
                Serial1.begin(this->baudRate);
                Serial.println("Serial 1 Has Begun at a baud rate of: " + String(this->baudRate));
            }else if(this->serialPort == 2){
                Serial2.begin(this->baudRate);
                Serial.println("Serial 2 Has Begun at a baud rate of: " + String(this->baudRate));
            }
        }

        /*
        Enable or disable debugging printer within the serialReader class
        @param bool enableDebugging  : true enables debugging, false disables debugging
        */
        void debug(bool enableDebugging){
            this-> debugEnable = enableDebugging;
        }

        
        /*
        Get the most recently sent command and arguments. Clears the stored commandBytes so that monitor for commands only returns true once per command (after command is received with getCommand)
        */
        void getCommand(byte* args){
            // Copy values into args array, then erase values from commandByte Array
            for(uint8_t i = 0; i < this->bytesUsed; i++){
                args[i] = commandByte[i];
                commandByte[i] = 0;
            }
        }

        /*
            Returns True when there is a command available
        */
        bool monitorForCommands(){
            bool serialAvailable;

            if(this->serialPort == 0){
                serialAvailable = Serial.available();
                if(serialAvailable){
                    Serial.readBytesUntil('\n',commandByte,bytesUsed);
                }
            }else if (this->serialPort == 1){
                serialAvailable = Serial1.available();
                if(serialAvailable){
                    Serial1.readBytesUntil('\n',commandByte,bytesUsed);
                }
            }else if(this->serialPort == 2){
                serialAvailable = Serial2.available();
                if(serialAvailable){
                    Serial2.readBytesUntil('\n',commandByte,bytesUsed);
                }
            }
            
            if(commandByte[0] != 0 and !serialAvailable){
                return true;
            }
            return false;
        }

        void forwardCommand(byte* commandArray){
            if(this->serialPort == 0){
                Serial.write(commandArray,this->bytesUsed);
            }else if (this->serialPort == 1){
                Serial1.write(commandArray,this->bytesUsed);
            }else if(this->serialPort == 2){
                Serial2.write(commandArray,this->bytesUsed);
            }else{
                Serial.println("Invalid Serial Port Chosen");
            }
            Serial.println("Command Forwarded");
        }

};
#endif