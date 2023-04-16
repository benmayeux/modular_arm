#ifndef __SERIALREADER_H__
#define __SERIALREADER_H__

#include <Arduino.h>
// #include <debugPrinter.h>

class serialReader
{
private:
    // debugPrinter db;
    bool debugEnable = false;
    String command = "";

public:
    serialReader()
    {
        // db.debug(debugEnable);
    }

    /*
    Enable or disable debugging printer within the serialReader class
    @param bool enableDebugging  : true enables debugging, false disables debugging
    */
    void debug(bool enableDebugging)
    {
        this->debugEnable = enableDebugging;
        // db.debug(this->debugEnable);
    }

    /*
    Get the most recently sent command and arguments
    @param String* args : a String array to store the returned values to
    */
    int getCommand(String *args)
    {
        if (this->debugEnable)
        {
            Serial.println("Trying to parse command");
        }
        const int maxVars = 7;
        // String retVal[maxVars] = {""};

        // Get the name of the function to be called
        command.replace("\n", ""); // remove the return in case it is a command and not a function call (is lacking parenthesis)
        String functionName = command.substring(0, command.indexOf("("));
        args[0] = functionName;
        args[0].toUpperCase();

        // Get the parameters that go with that command
        command = command.substring(command.indexOf("(") + 1); // Trim off function name
        int numVars = 1;
        while (command.indexOf(",") != -1 && numVars < maxVars - 1)
        {
            if (this->debugEnable)
            {
                Serial.println(command.indexOf(","));
            }

            args[numVars] = command.substring(0, command.indexOf(","));
            command = command.substring(command.indexOf(",") + 1); // trim off the var that was just taken
            numVars++;

            if (this->debugEnable)
            {
                Serial.println("Stuck in loop");
            }
        }

        if (this->debugEnable)
        {
            Serial.println("Out of loop");
        }

        args[numVars] = command.substring(0, command.indexOf(")"));
        if (this->debugEnable)
        {
            Serial.println("finalized return value");
        }
        command = ""; // clear command value

        return numVars;
    }

    bool monitorForCommands()
    {
        if (Serial.available())
        {
            command = Serial.readStringUntil('\n');
        }
        else
        {
            if (command != "")
            {
                if (this->debugEnable)
                {
                    Serial.print("I received: ");
                    Serial.println(command);
                    Serial.println("Received command");
                }
                return true;
            }
        }
        return false;
    }
};
#endif