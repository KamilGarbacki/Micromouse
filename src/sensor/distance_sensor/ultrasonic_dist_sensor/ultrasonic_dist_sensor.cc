//
// Created by kamil on 20.08.2025.
//

#include "ultrasonic_dist_sensor.h"

namespace micromouse {

long UltrasonicDistSensor::readUltrasonicCM() {
  return newPing.ping_cm();
}

}  // namespace micromouse