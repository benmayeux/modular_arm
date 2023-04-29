#ifdef MAIN_MODULE_COMM_TEST
#include <Arduino.h>
#include "roundRobinComms.h"
#include "serialReadWrite.h"

const uint8_t numBytesPerMsg = 22;
byte command[numBytesPerMsg];

roundRobinComms rrc2(2,115200,numBytesPerMsg);

// serialReadWrite srw0(0,115200,numBytesPerMsg);

void setup() {
    // srw0.begin(); // Serial begin on the correct port and baud rate
    rrc2.begin();
    Serial.begin(115200);
}

void loop() {
    rrc2.loop();

  //Wait for response
  if(rrc2.newCommandReceived()){
    Communication C = rrc2.decipherCommand();
    Serial.println("Received Communication:");
    Serial.println(C.commType);
    Serial.println(C.moduleNum);
    for(int i=0;i<10;i++){
        Serial.println(C.data[i]);
    }
    rrc2.sendCommunication(C);
  }
}
#endif