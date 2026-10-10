// Entry points called by the Arduino core. All logic lives in app/.
// Arduino.h is needed only for the extern "C" setup()/loop() declarations.
#include <Arduino.h>

#include "app/robot_app.h"

namespace {
micromouse::RobotApp gApp;
}  // namespace
//claude --resume bd9720a8-90f5-4dd2-a39a-13956a205ea5

void setup() { gApp.begin(); }

void loop() { gApp.step(); }
