//
// Created by ming on 7/30/19.
//

#ifndef DRIVE_SINGULARITY_VEHICLEGENERATOR_H
#define DRIVE_SINGULARITY_VEHICLEGENERATOR_H

#include <iostream>
#include "DriveSingularity/Math/All.h"

namespace ds {
namespace utils {

using Range = std::pair<double, double>; // first: mu, second: var

class VehicleGenerator {
public:
  VehicleGenerator() {
    halfLengthRange = {0, 0};
    halfWidthRange = {0, 0};
    velocityRange = {0, 0};
    targetVelocityRange = {0, 0};
  }

  VehicleGenerator(Range halfLengthRange, Range halfWidthRange,
                   Range velocityRange, Range targetVelocityRange)
                   :halfLengthRange(std::move(halfLengthRange)),
                    halfWidthRange(std::move(halfWidthRange)),
                    velocityRange(std::move(velocityRange)),
                    targetVelocityRange(std::move(targetVelocityRange)) {
  }

  void setHalfLengthRange(const Range &halfLengthRange);

  void gen();

  void setHalfWidthRange(const Range &halfWidthRange);

  void setVelocityRange(const Range &velocityRange);

  void setTargetVelocityRange(const Range &targetVelocityRange);

public:
  double halfLength = 0;
  double halfWidth = 0;
  double velocity = 0;
  double targetVelocity = 0;

private:

	Range halfLengthRange;
	Range halfWidthRange;
	Range velocityRange;
	Range targetVelocityRange;
};
} // namespace utils
} // namespace ds

#endif // DRIVE_SINGULARITY_VEHICLEGENERATOR_H
