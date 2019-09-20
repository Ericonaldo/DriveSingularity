#include "Control/AgentVehicle.h"
#include "Control/Action.h"
#include "DriveSingularity/Math/Util.h"

#include <algorithm>
#include <iostream>

namespace ds {
namespace control {

void AgentVehicle::step(double dt) {
  followRoad();

  // behavior select
  act();

  // control
  VehicleController::step(dt);
}

void AgentVehicle::controlAccelerate() {
  setAcceleration(KpAccel * (getTargetVelocity() - getVelocity()));
}

void AgentVehicle::pickLane() {}

void AgentVehicle::act() {
  if (actionExecuted)
    return;

  switch (action) {
  case discrete::Faster:
    setTargetVelocity(
        std::min(getTargetVelocity() + DeltaVelocity, MaxVelocity));
    break;
  case discrete::Slower:
    setTargetVelocity(std::max(getTargetVelocity() - DeltaVelocity, 0.));
    break;
  case discrete::Idle:
    break;
  case discrete::TurnLeft:
  case discrete::TurnRight:
    auto straight = roadMap->getRoad<roadmap::StraightRoad>(getLaneId().first);
    if (straight) {
      int lane = (int)getLaneId().second;
      int side;
      if (straight->getLanes()[lane].isReversed()) {
        if (action == discrete::TurnLeft) {
          side = lane + 1;
        } else {
          side = lane - 1;
        }
      } else {
        if (action == discrete::TurnLeft) {
          side = lane - 1;
        } else {
          side = lane + 1;
        }
      }
      // TODO(Ming): remove lane filter for agents ?
      if (side >= 0 && side < (int)straight->getLanes().size() &&
          straight->getLanes()[side].isReversed() ==
              straight->getLanes()[lane].isReversed()) {
        setTargetLaneId(std::make_pair(getLaneId().first, (std::size_t)side));
      }
    }
    break;
  }
  actionExecuted = true;
}

void AgentVehicle::setAction(discrete::Action action) {
  AgentVehicle::action = action;
  actionExecuted = false;
}

double AgentVehicle::calculateReward() const {
  double actionReward = 0;
  if (action == discrete::TurnLeft || action == discrete::TurnRight)
    actionReward += LaneChangeReward;

  double stateReward = 0;
  if (isCrashed())
    stateReward += CollisionReward;
  stateReward += HighVelocityReward * getVelocity() / MaxVelocity;
  return remap(actionReward + stateReward,
               std::make_pair(CollisionReward, HighVelocityReward),
               std::make_pair(0, 1));
}

#ifdef false
void AgentVehicle::addListener(const event::EVENT_TYPE &event) {
  if (event::condition::isValid(event)) {
    enabledEvents.emplace(event, 0.);
  } else if (event::interaction::isValid(event)) {
    enabledEvents.emplace(event, 0.);
  } else {
    std::cout
        << "[WARNING] AgentVehicle::addListener: illegal event register named: "
        << event << std::endl;
  }
}

std::unordered_map<event::EVENT_TYPE, float>
AgentVehicle::retrieveEventEvaluation() {
  std::unordered_map<event::EVENT_TYPE, float> res = enabledEvents;

  // clean value
  for (auto &enabledEvent : enabledEvents) {
    enabledEvent.second = 0.;
  }

  return res;
}
#endif

} // namespace control
} // namespace ds