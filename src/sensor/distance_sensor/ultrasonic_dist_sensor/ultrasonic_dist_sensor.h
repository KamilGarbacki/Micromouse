//
// Created by kamil on 20.08.2025.
//

#ifndef ULTRASONIC_DIST_SENSOR_H
#define ULTRASONIC_DIST_SENSOR_H
#include <Arduino.h>
#include <stdint.h>
#include <NewPing.h>

#include "direction/Direction.h"

namespace micromouse {

class UltrasonicDistSensor {
  const uint8_t triggerPin;
  const uint8_t echoPin;
  const Direction sensorDir;
  NewPing newPing;

 public:
  UltrasonicDistSensor(const uint8_t triggerPin, const uint8_t echoPin, const Direction sensorDir)
      : triggerPin(triggerPin), echoPin(echoPin), sensorDir(sensorDir), newPing(NewPing(triggerPin, echoPin, 200)) {
    pinMode(echoPin, INPUT);
    pinMode(triggerPin, OUTPUT);
  }

  Direction getSensorDir() const { return sensorDir; }

  long readUltrasonicCM();
};

}  // namespace micromouse

#endif  // ULTRASONIC_DIST_SENSOR_H
