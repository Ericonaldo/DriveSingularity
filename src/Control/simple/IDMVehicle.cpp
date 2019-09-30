//
// Created by ming on 7/30/19.
//
#include "DriveSingularity/Control/simple/IDMVehicle.h"
#include <iostream>
#include <cassert>

namespace ds {
namespace control {

/**
 * Mobil for lane switch.
 *
 * @param candidateLaneId
 * @return
 */
bool IDMVehicle::mobil(roadmap::LaneId candidateLaneId) {
  auto newPreceding = getFrontVehicle(candidateLaneId, true);
  auto newFollowing = getRearVehicle(candidateLaneId);

  double newFollowingAccel =
      newFollowing ? newFollowing->calculateAcc(newPreceding, true) : 0;
  double newFollowingPredAccel = calculateAcc(newFollowing, false);

  if (newFollowingPredAccel < -control::LaneChangeMaxBrakingImposed)
    return false;

  auto oldPreceding = getFrontVehicle(getLaneId(), true);
  auto oldFollowing = getRearVehicle(getLaneId());

  double selfAccel = calculateAcc(oldPreceding, true);
  double selfPredAccel = calculateAcc(newPreceding, true);

  if (selfPredAccel < control::LaneChangeMaxBrakingImposed)
    return false;

  double oldFollowingAccel = calculateAcc(oldFollowing, false);
  double oldFollowingPredAccel =
      oldFollowing ? oldFollowing->calculateAcc(oldPreceding, true) : 0;

  double jerk =
      selfPredAccel - selfAccel +
      control::Politeness * (newFollowingPredAccel - newFollowingAccel +
                             oldFollowingPredAccel - oldFollowingAccel);

  return jerk > control::LaneChangeMinAccelGain;
}

double
IDMVehicle::getDesiredGap(const std::shared_ptr<VehicleController> &frontVehicle) {
  double d0 =
      control::DistanceWanted + getHalfLength() + frontVehicle->getHalfLength();
  double tau = control::TimeWanted;
  double ab = -control::MinComfortableAcceleration * MaxComfortableAcceleration;
  double dv = getVelocity() - frontVehicle->getVelocity();
  return d0 + getVelocity() * tau + getVelocity() * dv / (2 * std::sqrt(ab));
}

double IDMVehicle::calculateAcc(const std::shared_ptr<VehicleController> &vehicle,
                                bool front) {
  double acceleration = 0;
//	std::cout << "IDM::calcaulate acc " << std::endl;

  if (front) {
    acceleration =
        MaxComfortableAcceleration *
        (1 - std::pow(getVelocity() / notZero(getTargetVelocity()), Delta));
  } else {
    if (vehicle) {
      acceleration = MaxComfortableAcceleration *
                     (1 - std::pow(vehicle->getVelocity() /
                                       notZero(vehicle->getTargetVelocity()),
                                   2));
    }
  }

  if (vehicle) {
    // TODO(ming): improve
    double distance = (getPosition() - vehicle->getPosition()).getLength();
    acceleration -=
        front ? MaxComfortableAcceleration *
                    std::pow(getDesiredGap(vehicle) / notZero(distance), 2)
              : MaxComfortableAcceleration *
                    std::pow(vehicle->getDesiredGap(std::dynamic_pointer_cast<VehicleController>(shared_from_this())) /
                                 notZero(distance),
                             2);
  } else {
    auto road = roadMap->getRoads().at(getTargetLaneId().first);

    bool needSlow = false;
    VectorD barrierPos;
    if (road->getNodeType() == roadmap::NodeType::Onramp) {
      auto onramp = std::static_pointer_cast<roadmap::Onramp>(road);
      needSlow = onramp->needSlowDown(getLaneId().first);
      if (needSlow) {
        if (getLaneId().first == onramp->getLinkedRoads()[0]) {
          barrierPos = getBarrierPositionOnStraightRoad();
        } else {
          assert(getLaneId().first == onramp->getLinkedRoads()[1]);
          barrierPos = (onramp->getAnchors()[1] + onramp->getAnchors()[2]) / 2;
        }
      }
    } else if (road->getNodeType() == roadmap::NodeType::Offramp) {
      // there is no need to slow down at offramp
    } else if (road->getNodeType() == roadmap::NodeType::Crossroads) {
      auto crossroads = std::static_pointer_cast<roadmap::Crossroads>(road);
      roadmap::RoadId currentRoadId = getLaneId().first;
      assert(!getRoute().empty());
      roadmap::RoadId nextRoadId = getRoute().front().first;
      needSlow = crossroads->needSlowDown(currentRoadId, nextRoadId);
      if (needSlow) {
        // calculate barrierPos
        barrierPos = getBarrierPositionOnStraightRoad();
      }
    }
    if (needSlow) {
      double decay = accDecay(barrierPos);
      acceleration -= decay;
    }
  }

  return clip(acceleration, -control::MaxAcceleration,
              control::MaxAcceleration);
}

std::pair<double, VehicleId>
IDMVehicle::getFirstVPair(std::shared_ptr<roadmap::StraightRoad> &road,
                          bool reversed) const {
  double oldDis = reversed ? 0 : road->getLength();

  VehicleId vId = ErrorVehicleId;

  for (const auto &it : road->getVehicles()) {
    auto v = it.second;
    auto vpos = road->getRoadCoord(v->getPosition());

    if ((reversed && oldDis < vpos.getX()) ||
        (!reversed && oldDis > vpos.getX())) {
      oldDis = vpos.getX();
      vId = it.first;
    }
  }

  return std::make_pair(oldDis, vId);
}

std::shared_ptr<VehicleController>
IDMVehicle::getFrontVehicle(const ds::roadmap::LaneId &laneId,
                            bool switchLane) const {
  auto road = roadMap->getRoads().at(laneId.first);

  VehicleId myId = getId();
  VehicleId otherId = ErrorVehicleId;

  const auto &vehicles = road->getVehicles();
  double oldDis = 1000.;

  double dis = 0;

  for (auto &kv : vehicles) {
    auto other = kv.second;
    if (other->getId() == myId || other->getLaneId() != laneId)
      continue;

    if ((getLaneId().first == laneId.first &&
         road->getRoadType() == roadmap::RoadType::Straight) ||
        road->getRoadType() == roadmap::RoadType::UNKNOWN) {
      auto &otherPos = other->getPosition();
      double cosin =
          cos(otherPos - getPosition(),
              VectorD(std::cos(getRotation()), std::sin(getRotation())));
      dis = (otherPos - getPosition()).getLength();

      // TODO(ming): too sensitive eps, fix it.
      if (cosin >= 1 - eps && dis < oldDis) {
        otherId = other->getId();
        oldDis = dis;
      }

      if (switchLane && dis < oldDis) {
        otherId = other->getId();
        oldDis = dis;
      }
    } else {
      // TODO(ming): default is target
      if (roadGraph->getEdges().find(laneId.first) == roadGraph->getEdges().end())
        continue;
      auto _road = roadMap->getRoad<roadmap::StraightRoad>(laneId.first);
      bool reverse = true;

      if (roadGraph->getEdges().at(_road->getId()).from == getLaneId().first)
        reverse = false;

      auto vPair = getFirstVPair(_road, reverse);

      if (vPair.first < oldDis) {
        otherId = vPair.second;
        oldDis = vPair.first;
      }
    }
  }

  if (otherId == ErrorVehicleId)
    return nullptr;
  return vehicles.at(otherId);
}

std::shared_ptr<VehicleController> IDMVehicle::getFrontVehicle() const {
  auto vehicle = getFrontVehicle(getLaneId(), false);

  if (nullptr == vehicle && getTargetLaneId() != getLaneId()) {
    vehicle = getFrontVehicle(getTargetLaneId(), false);
  }

  return vehicle;
}

std::shared_ptr<VehicleController>
IDMVehicle::getRearVehicle(roadmap::LaneId laneId) const {
  auto straight = roadMap->getRoad<roadmap::StraightRoad>(laneId.first);
  double lRear = 0, lSelf = straight->getRoadCoord(getPosition()).getX();

  VehicleId otherId = ErrorVehicleId;
  VehicleId myId = getId();

  const auto &vehicles = straight->getVehicles();

  for (auto &kv : vehicles) {
    auto other = kv.second;
    if (other->getId() == myId || other->getLaneId() != laneId)
      continue;
    double lOther = straight->getRoadCoord(other->getPosition()).getX();
    bool flag = straight->getLanes().at(laneId.second).isReversed()
                    ? (lOther > lSelf &&
                       (otherId == ErrorVehicleId || lOther < lRear))
                    : (lOther < lSelf &&
                       (otherId == ErrorVehicleId || lOther > lRear));
    if (flag) {
      otherId = other->getId();
      lRear = lOther;
    }
  }
  if (otherId == ErrorVehicleId)
    return nullptr;
  return std::dynamic_pointer_cast<VehicleController>(vehicles.at(otherId));
}

double IDMVehicle::accDecay(const VectorD & barrierPos) {
  double distance = (barrierPos - getPosition()).getLength();
  double d0 = control::DistanceWanted + getHalfLength();
  double tau = control::TimeWanted;
  double ab =
          -control::MinComfortableAcceleration * MaxComfortableAcceleration;
  double dv = getVelocity() - 0;
  double wantedDis =
          d0 + getVelocity() * tau + getVelocity() * dv / (2 * std::sqrt(ab));
  return MaxComfortableAcceleration * std::pow(wantedDis / notZero(distance), 2);
}

void IDMVehicle::pickLane() {
  const auto &road = roadMap->getRoads().at(getLaneId().first);

  auto &vOnRoad = road->getVehicles();

  if (getLaneId() !=
      getTargetLaneId()) { // check whether it is capable to do lane switching
    // TODO(ming): consider switch lane on different road ?
    if (getLaneId().first != getTargetLaneId().first ||
        roadMap->getRoads().at(getLaneId().first)->getRoadType() ==
            roadmap::RoadType::UNKNOWN) {
      // The vehicle is driving from current road to next road.
      // Changing lane is not allowed in this case.
      return;
    }

    /*
     * ---------------------------------------------------------
     *                otherVehicle ------                          [lane 0]
     * ---------------------------------|-----------------------
     *                             /|\ \|/                         [lane 1]
     * -----------------------------|---------------------------
     *              egoVehicle ------                              [lane 2]
     * ---------------------------------------------------------
     *
     * If otherVehicle and egoVehicle are changing to the same lane
     * and otherVehicle's position is ahead of egoVehicle's a little,
     * then egoVehicle would cancel changing of lane.
     */
    roadmap::LaneId targetLaneId = getTargetLaneId();

    for (const auto &kv : vOnRoad) {
      auto &other = kv.second;

      // TODO(ming): (fix up) it is wrong in parallel case.
      if (getId() == other->getId() ||
          other->getLaneId() == other->getTargetLaneId() ||
          targetLaneId != other->getTargetLaneId()) // has no collision
        continue;

      auto straight =
          roadMap->getRoad<roadmap::StraightRoad>(getTargetLaneId().first);
      assert(straight);

      // Euler distance
      //			double d = (other->getPosition() -
      //getPosition()).getLeng
      double d =
          project(other->getPosition() - getPosition(),
                  VectorD(std::cos(getRotation()), std::sin(getRotation())));
      double dStart =
          getDesiredGap(std::dynamic_pointer_cast<VehicleController>(other));

      if (d > 0 && d < dStart) {
//        std::cout << "cancel" << std::endl;
        setTargetLaneId(getLaneId());
      }
    }
    return;
  }

  // check current lane is straight road or node, node is no need to change lane
  if (roadMap->getRoads().at(getLaneId().first)->getRoadType() ==
      roadmap::RoadType::UNKNOWN)
    return;

  // Two lane changes need to be separated by certain
  // time(Vehicle::LaneChangeDelay).
  if (getTimer() < control::LaneChangeDelay) // give up switch lane
    return;

  setTimer(0);
  std::array<int, 2> dir = {-1, 1}; // random move right or left

  for (int i : dir) {
    auto straight = roadMap->getRoad<roadmap::StraightRoad>(getLaneId().first);
    assert(straight);

    // pick a feasible side lane
    int side = (int)getLaneId().second + i;
    if (side < 0 || side >= (int)straight->getLanes().size() ||
        straight->getLanes()[side].isReversed() !=
            straight->getLanes()[getLaneId().second].isReversed())
      continue;

    roadmap::LaneId candidateLaneId =
        std::make_pair(getLaneId().first, (std::size_t)side);

    if (mobil(candidateLaneId)) { // use mobil algorithm to select a available
                                  // lane
//      std::cout << "fuck" << std::endl;
      setTargetLaneId(candidateLaneId);
      break;
    }
  }
}

void IDMVehicle::controlAccelerate() {
//	std::cout << "IDM:control" << std::endl;
  auto vehicle = getFrontVehicle();
//  std::cout << "IDM::control vehicle type: " << std::endl;
  setAcceleration(calculateAcc(vehicle, true));
}

void IDMVehicle::step(double dt) {
  followRoad();
  pickLane();
//	controlSteering();
//	controlAccelerate();
  VehicleController::step(dt);
}

VectorD IDMVehicle::getBarrierPositionOnStraightRoad() {
  auto currentRoad = roadMap->getRoad<roadmap::StraightRoad>(getLaneId().first);
  assert(currentRoad);
  std::size_t laneIdx = getLaneId().second;
  const auto &edges = currentRoad->getEdges();
  if (currentRoad->getLanes().at(laneIdx).isReversed()) {
    return (edges.at(laneIdx).getStart() + edges.at(laneIdx + 1).getStart()) / 2;
  } else {
    return (edges.at(laneIdx).getEnd() + edges.at(laneIdx + 1).getEnd()) / 2;
  }
}

} // namespace control
} // namespace ds
