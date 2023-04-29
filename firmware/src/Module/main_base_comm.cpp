#ifdef MAIN_BASE_COMM_TEST
#include <Arduino.h>
#include "roundRobinComms.h"
#include "serialReadWrite.h"

const uint8_t numBytesPerMsg = 22;
byte command[numBytesPerMsg];
// byte command2[numBytesPerMsg];

roundRobinComms rrc2(2,115200,numBytesPerMsg);
roundRobinComms rrc0(0,115200,numBytesPerMsg);

serialReadWrite srw0(0,115200,numBytesPerMsg);

void setup() {
    srw0.begin(); // Serial begin on the correct port and baud rate
    rrc2.begin();
    rrc0.begin();
}

void loop() {
    
    
    rrc2.loop();

    // Monitor for computer inputs
    if(srw0.monitorForCommands()){
        srw0.getCommand(command);
        Serial.print("Received new command: ");
        Serial.write(command,numBytesPerMsg);
        Serial.println();
        Serial.println("Encoding Bytes to Communication");
        Communication C = rrc2.decipherCommand(command);
        Serial.println(C.commType);
        Serial.println(C.moduleNum);
        for(int i=0;i<10;i++){
            Serial.println(C.data[i]);
        }

        // Serial.println("Decoding Communication to Bytes");
        // rrc2.encipherCommand(C);
        // for(int i=0;i<10;i++){
        //     Serial.println(rrc2.encipheredCommand[i]);
        // }

        Serial.println("Writing command to Serial 2: ");
        rrc2.sendCommunication(C);

        // Serial.println("Writing command to Serial 0: ");
        // rrc0.sendCommunication(C);
    }

  //Wait for response
  if(rrc2.newCommandReceived()){
    Communication C = rrc2.decipherCommand();
    Serial.println("Received Communication:");
    Serial.println(C.commType);
    Serial.println(C.moduleNum);
    for(int i=0;i<10;i++){
        Serial.println(C.data[i]);
    }
  }
}
#endif