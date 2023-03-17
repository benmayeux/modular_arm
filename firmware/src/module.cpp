#ifdef ARM_MODULE

#include <Arduino.h>
#include "Module.h"
  Module m = Module();
  void setup()
  {
    m.setup();
  }

  void loop()
  {
    m.loop();
  }

#endif 