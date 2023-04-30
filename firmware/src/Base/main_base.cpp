#ifdef MAIN_BASE
#pragma GCC optimize ("-O0")
#pragma GCC push_options
#include "Base.h"

base::Base b = base::Base();
void setup() {
  Serial2.begin(115200);
  delay(3000);
  while(Serial2.available()) {
    Serial.println(Serial2.read());
  }
  delay(3000);
  b.setup();
}

void loop() {
  b.loop();
}

#pragma GCC pop_options
#endif