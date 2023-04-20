#ifndef __DEBUGPRINTER_H__
#define __DEBUGPRINTER_H__

#include <Arduino.h>

class debugPrinter
{
private:
    bool debugEnable = false;

public:
    debugPrinter()
    {
    }

    int debug(bool enableDebugging)
    {
        this->debugEnable = enableDebugging;
    }

    void print(const char *args)
    {
        if (this->debugEnable)
        {
            Serial.print(*args);
        }
    }

    void println(const char *args)
    {
        if (this->debugEnable)
        {
            Serial.println(*args);
        }
    }

    void print(String args)
    {
        if (this->debugEnable)
        {
            Serial.print(args);
        }
    }

    void println(String args)
    {
        if (this->debugEnable)
        {
            Serial.println(args);
        }
    }

    void print(int args)
    {
        if (this->debugEnable)
        {
            Serial.print(args);
        }
    }

    void println(int args)
    {
        if (this->debugEnable)
        {
            Serial.println(args);
        }
    }
};
#endif