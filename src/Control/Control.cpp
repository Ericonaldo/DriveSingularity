#include <iostream>
#include "DriveSingularity/Control/Control.h"

namespace ds {
namespace control {

void VehicleController::step(double dt) {
  controlSteering();
  controlAccelerate();

  double curVelocity = getVelocity(), maxV = getTargetVelocity();

  if (curVelocity > maxV) {
    setAcceleration(std::min(getAcceleration(), maxV - curVelocity));
  } else if (curVelocity < -maxV) {
    setAcceleration(std::max(getAcceleration(), maxV - curVelocity));
  }

  getX() += curVelocity * std::cos(getRotation()) * dt;
  getY() += curVelocity * std::sin(getRotation()) * dt;

  double steering = getSteering(), acceleration = getAcceleration();

  getRotation() +=
          curVelocity * std::tan(steering) / getHalfLength() / 2 * dt;

  if (getRotation() > 2 * Pi)
    getRotation() -= 2 * Pi;
  else if (getRotation() < 0)
    getRotation() += 2 * Pi;

  curVelocity += acceleration * dt;

  curVelocity += acceleration * dt;

  curVelocity = clip(curVelocity, 0, getTargetVelocity());
  setVelocity(curVelocity);

  updateLight(eps);

  timer += dt;
}

void VehicleController::controlSteering() {
  auto curLane = getLaneId();
  auto curRoad = roadMap->getRoads().at(getLaneId().first);

  roadmap::RoadId roadId = curRoad->getRoadType() == roadmap::RoadType::UNKNOWN
                           ? getTargetLaneId().first
                           : curLane.first;

  auto toLane = getTargetLaneId();
  auto targetRoad = roadMap->getRoads().at(getTargetLaneId().first);
  if ((!getRoute().empty() &&
      targetRoad->getRoadType() == roadmap::RoadType::UNKNOWN)
      || (curRoad->getRoadType() != roadmap::RoadType::UNKNOWN
      && getTargetLaneId().first != getLaneId().first)) {
    toLane = curLane;
  }

  if (roadMap->getRoads().at(roadId)->getRoadType() ==
      roadmap::RoadType::UNKNOWN)
    return;

  auto straight = roadMap->getRoad<roadmap::StraightRoad>(roadId);
  // TODO(ming): should we convert relative coordination to absolute
  // coordination ?
  auto roadCoord = straight->getRoadCoord(getPosition());

  double lateral =
          roadCoord.getY() - (-straight->getWidth() / 2 +
                              (toLane.second + 0.5) * straight->getLaneWidth());

  if (straight->getLanes().at(toLane.second).isReversed())
    lateral = -lateral;

  double lateralVelocityCommand = control::KpLateral * lateral;
  double headingCommand =
          std::asin(clip(lateralVelocityCommand / notZero(getVelocity()), -1, 1));

  auto start = straight->getAxis().getStart();
  auto end = straight->getAxis().getEnd();
  auto delta = straight->getLanes().at(toLane.second).isReversed()
               ? start - end
               : end - start;
  double headingRef =
          delta.getRotation() + clip(headingCommand, -Pi / 2.4, Pi / 2.4);
  double headingRateCommand =
          control::KpHeading * wrapToPi(headingRef - getRotation());

  double steering = clip(getHalfLength() * 2 / notZero(getVelocity()) *
                         std::atan(headingRateCommand),
                         -control::MaxSteeringAngle, control::MaxSteeringAngle);

  setSteering(steering);
}

void VehicleController::followRoad() {
  const auto &edges = roadGraph->getEdges();
  auto curLaneId = getLaneId();

  const auto &curRoad = roadMap->getRoads().at(curLaneId.first);
//  assert(curRoad);

  if (curRoad->onRoad(getPosition())) {
    if (gotFirstTargetLane) {
      return;
    } else {
      gotFirstTargetLane = true;
    }
//    return;
  }

  setLastLaneId(getLaneId());
  setLaneId(getTargetLaneId());

  // TODO(yifan): for route, we use push_front, front and pop_front,
  //  which is weird. But now route has at most one element, so it
  //  looks ok.

  if (!getRoute().empty()) {
    auto laneId = getRoute().front();
    setTargetLaneId(laneId);
    popRoute();
    return;
  }

  auto straight =
          roadMap->getRoad<roadmap::StraightRoad>(getLaneId().first);
  assert(straight);

  roadmap::RoadId nextId;
  const auto &edge = edges.at(getLaneId().first);

  if (straight->getLanes().at(getLaneId().second).isReversed())
    nextId = edge.from;
  else
    nextId = edge.to;

  // if the next one is road
  if (edges.find(nextId) != edges.end()) {
    auto nextRoad = roadMap->getRoad<roadmap::StraightRoad>(nextId);
    assert(nextRoad);

    getTargetLaneId().first = nextId; // TODO(ming): it require lane size ...

    const auto &nextEdge = edges.at(nextId);
    if ((edge.from == nextId && nextEdge.from == getLaneId().first)
        || (edge.to == nextId && nextEdge.to == getLaneId().first)) {
      getTargetLaneId().second = nextRoad->getLanes().size() - 1 - getLaneId().second;
    }
    return;
  }

  // if the next one is node
  auto &node = roadGraph->getVertexes().at(nextId);

  if (node.nodeType == roadmap::NodeType::Crossroads) {
    // For crossroads, pick a feasible road randomly.
    assert(node.linkedRoad.size() == 4);
    std::vector<roadmap::RoadId> nextRoadIds;

    // search the start feasible road
    size_t i = 0;
    while (node.linkedRoad[i] != straight->getId() && i < 4)
      ++i;
    assert(i < 4);

    // get direction of current lane
    uint8_t direction =
            straight->getLanes().at(getLaneId().second).getDirection();

    // select all feasible lane
    if ((direction & roadmap::TurnRight) != 0)
      nextRoadIds.push_back(node.linkedRoad[(i + 1) % 4]);
    if ((direction & roadmap::GoStraight) != 0)
      nextRoadIds.push_back(node.linkedRoad[(i + 2) % 4]);
    if ((direction & roadmap::TurnLeft) != 0)
      nextRoadIds.push_back(node.linkedRoad[(i + 3) % 4]);

    // random select
    roadmap::RoadId nextRoadId = nextRoadIds.at(rand() % nextRoadIds.size());
    auto nextRoad = roadMap->getRoad<roadmap::StraightRoad>(nextRoadId);
    std::size_t nextLaneIdx;
    // if the next road is reverses, then get reversed target lane
    if (edges.at(straight->getId()).from == edges.at(nextRoad->getId()).from ||
        edges.at(straight->getId()).to == edges.at(nextRoad->getId()).to) {
      nextLaneIdx = straight->getLanes().size() - 1 - getLaneId().second;
    } else {
      nextLaneIdx = getLaneId().second;
    }
    getRoute().push_front(std::make_pair(nextRoadId, nextLaneIdx));
    setTargetLaneId(std::make_pair(node.nodeId, 0));
    return;
  }

  /*
   * onramp
   *
   * -------------------------------------------------------------
   *       /___                            /___
   *    linkedRoad[0]                    linkedRoad[2]
   *        ___\                            ___\
   * ---------------------       ---------------------------------
   *                    /       /
   *                   /   /|  /
   *                  /   /   /
   *                 /   /   /    linkedRoad[1]
   *                /       /
   *               /       /
   *
   */
  if (node.nodeType == roadmap::NodeType::Onramp) {
    if (getLaneId().first == node.linkedRoad[2]) { // if drive from road 2
      auto nextStraight = roadMap->getRoad<roadmap::StraightRoad>(node.linkedRoad[0]);
      assert(nextStraight);
      std::size_t nextLaneIdx;
      if (edges.at(straight->getId()).from == edges.at(nextStraight->getId()).from ||
          edges.at(straight->getId()).to == edges.at(nextStraight->getId()).to) {
        nextLaneIdx = straight->getLanes().size() - 1 - getLaneId().second;
      } else {
        nextLaneIdx = getLaneId().second;
      }
      getRoute().push_front(
              std::make_pair(node.linkedRoad[0], nextLaneIdx));
      setTargetLaneId(std::make_pair(node.nodeId, 0));
      return;
    }

    // other case set target to road 2
    auto nextRoad =
            roadMap->getRoad<roadmap::StraightRoad>(node.linkedRoad[2]);
    assert(nextRoad);

    if (getLaneId().first == node.linkedRoad[0]) { // drive from road 0
      assert(straight->getLanes().size() == nextRoad->getLanes().size());
      // reverse lane or not
      if (edges.at(straight->getId()).from ==
          edges.at(nextRoad->getId()).from ||
          edges.at(straight->getId()).to == edges.at(nextRoad->getId()).to) {
        //				getTargetLaneId().second =
        //straight->getLanes().size() - 1 - getLaneId().second;
        getRoute().push_front(
                std::make_pair(node.linkedRoad[2], straight->getLanes().size() - 1 -
                                                   getLaneId().second));
      } else {
        getRoute().push_front(
                std::make_pair(node.linkedRoad[2], getLaneId().second));
      }
    } else { // drive from road 1
      // TODO(yifan): for now we assume linkedRoad[1] has only one lane.
      if (roadGraph->getEdges().at(nextRoad->getId()).from == node.nodeId) {
        //				getTargetLaneId().second =
        //nextRoad->getLanes().size() - 1;
        getRoute().push_front(std::make_pair(node.linkedRoad[2],
                                             nextRoad->getLanes().size() - 1));
      } else {
        getRoute().push_front(std::make_pair(node.linkedRoad[2], 0));
      }
    }
    setTargetLaneId(std::make_pair(node.nodeId, 0));
    return;
  }

  /*
   * offramp
   *
   *        \       \
   *          \       \
   *            \  |\   \
   *              \   \   \   linkedRoad[1]
   *                \   \   \
   *                  \       \
   * -------------------       -----------------------------------
   *       /___                              /___
   *    linkedRoad[2]                     linkedRoad[0]
   *        ___\                              ___\
   * --------------------------------------------------------------
   *
   */
  if (node.nodeType == roadmap::NodeType::Offramp) {
    if (getLaneId().first == node.linkedRoad[0]) {
      if (((edges.at(straight->getId()).to == nextId && getLaneId().second == straight->getLanes().size() - 1)
           || (edges.at(straight->getId()).from == nextId && getLaneId().second == 0)) && (rand() % 2 == 0)) {
        getRoute().push_front(std::make_pair(node.linkedRoad[1], 0));
      } else {
        roadmap::RoadId toRoadId = node.linkedRoad[2];
        std::size_t nextLaneIdx;
        if (edges.at(straight->getId()).from == edges.at(toRoadId).from ||
            edges.at(straight->getId()).to == edges.at(toRoadId).to) {
          nextLaneIdx = straight->getLanes().size() - 1 - getLaneId().second;
        } else {
          nextLaneIdx = getLaneId().second;
        }
        getRoute().push_front(std::make_pair(toRoadId, nextLaneIdx));
      }
    } else {
      assert(getLaneId().first == node.linkedRoad[2]);
      roadmap::RoadId toRoadId = node.linkedRoad[0];
      std::size_t nextLaneIdx;
      if (edges.at(straight->getId()).from == edges.at(toRoadId).from ||
          edges.at(straight->getId()).to == edges.at(toRoadId).to) {
        nextLaneIdx = straight->getLanes().size() - 1 - getLaneId().second;
      } else {
        nextLaneIdx = getLaneId().second;
      }
      getRoute().push_front(std::make_pair(toRoadId, nextLaneIdx));
    }
    setTargetLaneId(std::make_pair(node.nodeId, 0));
    return;
  }

  if (node.nodeType == roadmap::NodeType::Terminal) {
    // do nothing
  }
}

} // namespace control
} // namespace ds