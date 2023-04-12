#ifdef MAIN_BASE
#pragma GCC optimize ("-O0")
#pragma GCC push_options

#include "Base.h"

Base b = Base();
void setup() {
  b.setup();
}

void loop() {
  b.loop();
}

#pragma GCC pop_options
#endif