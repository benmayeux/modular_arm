#ifdef MAIN_BASIC_MODULE_STANDALONE_TESTING
#include <Arduino.h>
#include "BasicModule.h"
#include "serialReader.h"

// Initialize the module with the correct PWM Pin, Potentiometer Pin, and Orientation Switch Pin
BasicModule thisModule = BasicModule(14,4,23);

String command[7] = {"N/A"};
serialReader sr;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  thisModule.setup(&Serial2,&Serial2);

  delay(3000);
  while(Serial2.available()) {
    Serial2.read();
  }
  delay(3000);
}

void loop() {
  thisModule.loop();

  if(sr.monitorForCommands()){
    sr.getCommand(command);
    Serial.print("Received new command: ");
    Serial.println(command[0]);
  }

  if(command[0] == "SETEFFORT"){
    thisModule.setEffort(command[1].toInt());
    Serial.print("Set the motor effort to: ");
    Serial.println(command[1]);
  }
  if(command[0] == "GETEFFORT"){
    Serial.print("The motor effort is currently: ");
    Serial.println(thisModule.getEffort());
  }
  if(command[0] == "SETPOS"){
    thisModule.setPosition(command[1].toInt());
    Serial.print("Set the motor position to: ");
    Serial.println(command[1]);
  }
  if(command[0] == "SETPID"){
    thisModule.setPosKpKiKd(command[1].toFloat(),command[2].toFloat(),command[3].toFloat());
    Serial.print("Set the PID values to: ");
    Serial.print(command[1]);
    Serial.print("\t");
    Serial.print(command[2]);
    Serial.print("\t");
    Serial.print(command[3]);
    Serial.print("\n");
  }
  if(command[0] == "GETPID"){
    float retVal[3];
    thisModule.getPosKpKiKd(retVal);
    Serial.print("Current PID Values: ");
    Serial.print(retVal[0]);
    Serial.print("\t");
    Serial.print(retVal[1]);
    Serial.print("\t");
    Serial.print(retVal[2]);
    Serial.print("\n");
  }
  if(command[0] == "GETRAWPOS"){
    Serial.print("The raw position is currently: ");
    Serial.println(thisModule.getRawPosition());
  }
  if(command[0] == "GETPOS"){
    Serial.print("The position in centidegrees is currently: ");
    Serial.println(thisModule.getPosition());
  }
  if(command[0] == "SAVETOEEPROM"){
    Serial.println("Saving Value to EEPROM");
    thisModule.save16BitToEEPROM(command[1].toInt(), command[2].toInt(), command[3].toInt());
  }
  if(command[0] == "RETRIEVEFROMEEPROM"){
    Serial.print("Retrieving Value From EEPROM: ");
    Serial.println(thisModule.read16BitFromEEPROM(command[1].toInt(), command[2].toInt()));
  }

  command[0] = "";

}
#endif