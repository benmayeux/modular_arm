#ifndef __DEBUGPRINTER_H__
#define __DEBUGPRINTER_H__

#include <Arduino.h>

class debugPrinter{
    private:
        bool debugEnable = false;
    
    public:

        debugPrinter(){}
        void debug(bool enableDebugging){
            this-> debugEnable = enableDebugging;
        }
        void println(const String &s){
            if(this->debugEnable){
                Serial.println(s);
            }
        }
        void println(const char s[]){
            if(this->debugEnable){
                Serial.println(s);
            }
        }
        void println(char s){
            if(this->debugEnable){
                Serial.println(s);
            }
        }
        void println(int s){
            if(this->debugEnable){
                Serial.println(s);
            }
        }
        void println(float s){
            if(this->debugEnable){
                Serial.println(s);
            }
        }
        void println(double s){
            if(this->debugEnable){
                Serial.println(s);
            }
        }

        void print(String str){
            if(this->debugEnable){
                Serial.print(str);
            }
        }

};

#endif