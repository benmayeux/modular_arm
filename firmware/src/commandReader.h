#ifndef __COMMANDREADER_H__
#define __COMMANDREADER_H__

#include <Arduino.h>



class commandReader{
    private:
        bool debugEnable = false;
        String command = "";
        uint8_t serialPort = 0;
        uint32_t baudRate = 115200;
    
    public:
        commandReader(uint8_t serialPort, uint32_t baudRate){
            this->serialPort = serialPort;
            this->baudRate = baudRate;
        }

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
        Get the most recently sent command and arguments
        @param String* args : a String array to store the returned values to
        */
        void getCommand(String* args){
            const int maxVars = 7;
            // String retVal[maxVars] = {""};
            
            //Get the name of the function to be called
            command.replace("\n",""); //remove the return in case it is a command and not a function call (is lacking parenthesis)
            String functionName = command.substring(0,command.indexOf("("));
            args[0] = functionName;
            args[0].toUpperCase();

            //Get the parameters that go with that command
            command = command.substring(command.indexOf("(") + 1); //Trim off function name
            int numVars = 1;
            while(command.indexOf(",") != -1 && numVars < maxVars - 1){

                args[numVars] = command.substring(0,command.indexOf(","));
                command = command.substring(command.indexOf(",")+1);//trim off the var that was just taken
                numVars++;
            }

            args[numVars] = command.substring(0,command.indexOf(")"));
            command = ""; //clear command value
        }


        bool monitorForCommands(){
            bool serialAvailable;

            if(this->serialPort == 0){
                serialAvailable = Serial.available();
                if(serialAvailable){
                    command =  Serial.readStringUntil('\n');
                }
            }else if (this->serialPort == 1){
                serialAvailable = Serial1.available();
                if(serialAvailable){
                    command =  Serial1.readStringUntil('\n');
                }
            }else if(this->serialPort == 2){
                serialAvailable = Serial2.available();
                if(serialAvailable){
                    command =  Serial2.readStringUntil('\n');
                }
            }
            
            if(command != "" and !serialAvailable){
                return true;
            }
            return false;
        }

};
#endif