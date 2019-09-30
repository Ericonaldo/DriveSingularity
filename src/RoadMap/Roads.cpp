#include "DriveSingularity/RoadMap/Roads.h"
#include "DriveSingularity/Control/Control.h"

namespace ds {
namespace roadmap {

Road::~Road() = default;

StraightRoad::StraightRoad(RoadId id, double width, const Segment &axis,
                           std::array<VectorD, 4> anchors,
                           std::vector<LaneInfo> inlanes)
    : Road(id), width(width), axis(axis),
      lanes(std::move(inlanes)) { // 0 1 2 3 - counter-clockwise
  linkedBorders.emplace_back(Segment(anchors[0], anchors[1])); // add from border
  linkedBorders.emplace_back(Segment(anchors[2], anchors[3])); // add to border
  std::size_t n = lanes.size();

  // if no lanes -> change n to n + 1
  for (int i = n; i >= 0; --i) {
    edges.emplace_back((anchors[0] * i + anchors[1] * (n - i)) / n,
                       (anchors[2] * i + anchors[3] * (n - i)) / n);
  }
  roadType = RoadType::Straight;
}

std::vector<std::size_t> StraightRoad::occupiedLanes(const Obb &obb) const {
  std::vector<std::size_t> res;
  bool lastFlag =
      ::ds::checkCollision({edges[0].getStart(), edges[0].getEnd()}, obb);
  for (std::size_t i = 0; i < edges.size() - 1; ++i) {
    bool nextFlag = ::ds::checkCollision(
        {edges[i + 1].getStart(), edges[i + 1].getEnd()}, obb);
    if (!lastFlag && !nextFlag)
      continue;
    lastFlag = nextFlag;
    res.emplace_back(i);
  }
  return res;
}

std::size_t StraightRoad::locatedLane(const VectorD &pos) const {
  auto dist = signedDistance(edges.front(), pos);
  assert(dist > -eps);
  double laneWidth = width / lanes.size();
  return (std::size_t)(dist / laneWidth);
}

bool StraightRoad::checkCollision(const Obb &obb) const {
  return ::ds::checkCollision(obb, edges.front()) ||
         ::ds::checkCollision(obb, edges.back());
}

Segment StraightRoad::getLaneAxis(std::size_t laneNum) const {
  return {(edges.at(laneNum).getStart() + edges.at(laneNum + 1).getStart()) / 2,
          (edges.at(laneNum).getEnd() + edges.at(laneNum + 1).getEnd()) / 2};
}

VectorD StraightRoad::getRoadCoord(const VectorD &position) const {
  auto delta = position - axis.getStart();
  auto direction1 = normalize(axis.getEnd() - axis.getStart());
  auto direction2 = rotate90(direction1);
  return {dot(delta, direction1), dot(delta, direction2)};
}

bool StraightRoad::onRoad(const VectorD &position) const {
  if (cross(edges.begin()->getEnd() - edges.begin()->getStart(), position - edges.begin()->getStart()) *
      cross(edges.rbegin()->getEnd() - edges.rbegin()->getStart(), position - edges.rbegin()->getStart()) > eps)
    return false;
  return cross(edges.begin()->getEnd() - edges.rbegin()->getEnd(), position - edges.rbegin()->getEnd()) *
          cross(edges.begin()->getStart() - edges.rbegin()->getStart(), position - edges.rbegin()->getStart()) < eps;
}

LaneId StraightRoad::locate(const VectorD &position) const {
  auto roadCoord = getRoadCoord(position);
  auto lateral = roadCoord.getY();

  assert(fabs(lateral) <= width / 2);
  double laneWidth = width / lanes.size();
  return std::make_pair(getId(), (size_t)((lateral + width / 2) / laneWidth));
}

void Crossroads::updateTrafficLights() {
  --counter;
  if (counter == 0) {
    trafficLightsPatternId = (trafficLightsPatternId + 1) % 8;
    counter = trafficLightsPatternId % 2 == 0 ? GreenDuration : RedDuration;
  }
}

bool Crossroads::onRoad(const ds::VectorD &position) const {
	// get linked road
	const auto& block = getObb();
//	return block.inBox(position);
	auto roadCoord = position - block.getPosition();
	auto wVec = obb.getWidVec();
	auto lenVec = obb.getLenVec();

	double w = project(roadCoord, wVec);
	double h = project(roadCoord, lenVec);

	return std::abs(w) <= block.getHalfWidth() && std::abs(h) <= block.getHalfLength();
}

LaneId Crossroads::locate(const ds::VectorD &position) const {
	return std::make_pair(getId(), 0);
}

bool Crossroads::needSlowDown(RoadId from, RoadId to) const {
  int fromIdx = getRoadIdx(from), toIdx = getRoadIdx(to);
  uint8_t direction = 0;
  switch ((toIdx - fromIdx + 4) % 4) {
    case 1:
      direction = TurnRight;
      break;
    case 2:
      direction = GoStraight;
      break;
    case 3:
      direction = TurnLeft;
      break;
    default:
      assert(false);
  }
  return (TrafficLightsPatterns[trafficLightsPatternId][fromIdx] & direction) == 0 || counter < 60;
}


int Crossroads::getRoadIdx(ds::roadmap::RoadId id) const {
  for (int i = 0; i < 4; ++i) {
    if (id == roads[i]) {
      return i;
    }
  }
//  std::cout << "ooops" << std::endl;
  assert(false);
}

bool Onramp::onRoad(const ds::VectorD &position) const {
  if (cross(anchors[3] - anchors[0], position - anchors[0]) *
      cross(anchors[2] - anchors[1], position - anchors[1]) > eps)
    return false;
  return cross(anchors[3] - anchors[2], position - anchors[2]) *
         cross(anchors[0] - anchors[1], position - anchors[1]) < eps;
}

// TODO(yifan): I think locate only makes sense in StraightRoad
LaneId Onramp::locate(const ds::VectorD &position) const {
	assert(false);
}

bool Onramp::needSlowDown(roadmap::RoadId from) const {
  return (from == roads[0] && counter[1] != 0) || (from == roads[1] && counter[0] != 0);
}

// TODO(yifan): need improving. same as Onramp::onRoad
bool Offramp::onRoad(const ds::VectorD &position) const {
  if (cross(anchors[2] - anchors[1], position - anchors[1]) *
      cross(anchors[3] - anchors[0], position - anchors[0]) > eps)
    return false;
  return cross(anchors[2] - anchors[3], position - anchors[3]) *
         cross(anchors[1] - anchors[0], position - anchors[0]) < eps;
}

LaneId Offramp::locate(const ds::VectorD &position) const {
  assert(false);
}


} // namespace roadmap
} // namespace ds
