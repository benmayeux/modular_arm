#ifdef MAIN_BASE
#pragma GCC optimize ("-O0")
#pragma GCC push_options

#include "Base.h"

base::Base b = base::Base();
void setup() {
  b.setup();
}

void loop() {
  b.loop();
}

#pragma GCC pop_options
#endif