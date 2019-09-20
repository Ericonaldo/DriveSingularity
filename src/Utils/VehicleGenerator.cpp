//
// Created by ming on 7/30/19.
//
#include "DriveSingularity/Utils/VehicleGenerator.h"
#include "DriveSingularity/Math/All.h"

namespace ds {
namespace utils {
void VehicleGenerator::gen() {
  halfLength = random(halfLengthRange);
  halfWidth = random(halfWidthRange);
  velocity = random(velocityRange);
  targetVelocity = random(targetVelocityRange);
}

  void VehicleGenerator::setHalfLengthRange(const Range &halfLengthRange) {
    VehicleGenerator::halfLengthRange = halfLengthRange;
  }

  void VehicleGenerator::setHalfWidthRange(const Range &halfWidthRange) {
    VehicleGenerator::halfWidthRange = halfWidthRange;
  }

  void VehicleGenerator::setVelocityRange(const Range &velocityRange) {
    VehicleGenerator::velocityRange = velocityRange;
  }

  void VehicleGenerator::setTargetVelocityRange(const Range &targetVelocityRange) {
    VehicleGenerator::targetVelocityRange = targetVelocityRange;
  }

} // namespace utils
} // namespace ds
