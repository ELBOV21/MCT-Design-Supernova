// =============================================================================
// main.cpp  —  entry point
//
// All firmware logic lives in App/App.cpp.
// This file only wires setup/loop into the application layer.
// =============================================================================

#include <Arduino.h>
#include "App/App.h"

void setup() { App_Init(); }
void loop()  { App_Run();  }
