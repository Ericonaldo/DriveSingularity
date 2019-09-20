//
// Created by ming on 7/30/19.
//

#ifndef DRIVE_SINGULARITY_CREATOR_H
#define DRIVE_SINGULARITY_CREATOR_H

#include "DriveSingularity/Control/Control.h"

namespace ds {
namespace utils {
using namespace control;

class Creator {
public:
  virtual std::shared_ptr<control::VehicleController> create() = 0;
};

template <class T> class CreatorController : public Creator {
public:
  std::shared_ptr<control::VehicleController> create() override {
    return new T();
  }
};
} // namespace utils
} // namespace ds

#endif // DRIVE_SINGULARITY_CREATOR_H
