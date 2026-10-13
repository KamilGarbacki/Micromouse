// Entry points called by the Arduino core. All logic lives in app/.
// Arduino.h is needed only for the extern "C" setup()/loop() declarations.
#include <Arduino.h>

#include "app/robot_app.h"

namespace {
micromouse::RobotApp gApp;
}  // namespace

void setup() { gApp.begin(); }

void loop() { gApp.step(); }
