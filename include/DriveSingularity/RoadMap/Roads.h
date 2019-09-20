#include <utility>

#ifndef DRIVE_SINGULARITY_ROADMAP_ROADS_H
#define DRIVE_SINGULARITY_ROADMAP_ROADS_H

#include <json/json.h>
#include <memory>
#include <unordered_map>
#include <vector>

#include "DriveSingularity/Math/All.h"
#include "DriveSingularity/RoadMap/Common.h"
#include "DriveSingularity/Vehicle/Common.h"

namespace ds {
namespace control {
class VehicleController;
}
} // namespace ds

namespace ds {
namespace roadmap {

struct PairHash {
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2> &p) const {
    auto h1 = std::hash<T1>{}(p.first);
    auto h2 = std::hash<T2>{}(p.second);
    return h1 ^ h2;
  }
};

class Road {
public:
  explicit Road(RoadId id) : id(id), vehicles({}) {}

  virtual ~Road() = 0;

  RoadId getId() const { return id; }
  RoadType getRoadType() const { return roadType; }
  NodeType getNodeType() const { return nodeType; }

  const std::unordered_map<VehicleId,
                           std::shared_ptr<control::VehicleController>> &
  getVehicles() {
    return vehicles;
  }

  void clear() { vehicles.clear(); }

  void addVehicle(VehicleId vId,
                  std::shared_ptr<control::VehicleController> &v) {
    vehicles.emplace(vId, v);
  }
  void removeVehicle(VehicleId vId) { vehicles.erase(vId); }

  virtual bool onRoad(const VectorD &position) const = 0;
  virtual LaneId locate(const VectorD &position) const = 0;
  virtual VectorD getPosition() const = 0;
  virtual Json::Value retrieveInfo() { return Json::Value(); }

private:
  RoadId id;
  std::unordered_map<VehicleId, std::shared_ptr<control::VehicleController>>
      vehicles;

protected:
  RoadType roadType = RoadType::UNKNOWN;
  NodeType nodeType = NodeType::UNKNOWN;
};

// TODO(ming): replace road class name to area
using Area = Road;

constexpr std::array<std::array<uint8_t, 4>, 8> TrafficLightsPatterns = {
    {{GoStraight | TurnRight, 0, GoStraight | TurnRight, 0},
     {0, 0, 0, 0},
     {TurnLeft | TurnRight, 0, TurnLeft | TurnRight, 0},
     {0, 0, 0, 0},
     {0, GoStraight | TurnRight, 0, GoStraight | TurnRight},
     {0, 0, 0, 0},
     {0, TurnLeft | TurnRight, 0, TurnLeft | TurnRight},
     {0, 0, 0, 0}}};

constexpr std::size_t GreenDuration = 200;
constexpr std::size_t RedDuration = 80;

class Crossroads : public Road {
public:
  Crossroads(RoadId id, const Obb &obb, std::array<RoadId, 4> roads)
      : Road(id), obb(obb), roads(roads), trafficLightsPatternId(0),
        counter(GreenDuration) {
    nodeType = NodeType::Crossroads;
  }

  void updateTrafficLights();

  const Obb &getObb() const { return obb; }
  const std::array<RoadId, 4> &getRoads() const { return roads; }

  bool onRoad(const VectorD &position) const override;
  LaneId locate(const VectorD &position) const override;
  VectorD getPosition() const override { return obb.getPosition(); }

  /**
   * 	give VEHICLE a suggestion on whether it should slow down on crossroads
   *
   * 	@param vehicle the vehicle approaching crossroads
   * 	@return whether the vehicle should slow down
   */
  bool needSlowDown(RoadId from, RoadId to) const;

  //  /**
  //   * give current trafficLight for the ROADIDX-th road at crossroads
  //   *
  //   * @param roadIdx which road's trafficLight (0 <= roadIdx < 4)
  //   * @return trafficLight on that road
  //   */
  //  uint8_t getTrafficLight(std::size_t roadIdx) const {
  //    assert(roadIdx < 4);
  //    return TrafficLightsPatterns[trafficLightsPatternId][roadIdx];
  //  }

  Json::Value retrieveInfo() override {
    return Json::Value(Json::UInt64(trafficLightsPatternId));
  }

private:
  int getRoadIdx(RoadId id) const;

private:
  // TODO(yifan): if a road is a trapezoid with four fixed points, OBB seems
  //  unnecessary.
  Obb obb;
  // TODO(yifan): If graph is built, ROADS is unnecessary.
  std::array<RoadId, 4> roads;
  std::size_t trafficLightsPatternId;
  std::size_t counter;
};

class Terminal : public Road {
public:
  Terminal(RoadId id, const Segment &segment) : Road(id), segment(segment) {
    nodeType = NodeType::Terminal;
  }

  const Segment &getSegment() const { return segment; }

  bool isIntersected(const Obb &obb) const {
    return ::ds::checkCollision(obb, segment);
  }

  bool onRoad(const VectorD &position) const override { return true; };
  LaneId locate(const VectorD &position) const override {
    return std::make_pair(getId(), 0);
  }

  VectorD getPosition() const override {
    return (segment.getEnd() + segment.getStart()) / 2.0;
  }

private:
  // TODO(yifan): if a road is a trapezoid with four fixed points, segment seems
  //  unnecessary
  Segment segment;
};

class Onramp : public Road {
public:
  Onramp(RoadId id, const Obb &obb, std::array<RoadId, 3> roads)
      : Road(id), roads(roads), obb(obb) {
    nodeType = NodeType::Onramp;
    counter.fill(0);
  }

  const std::array<RoadId, 3> &getLinkedRoads() { return roads; }
  const Obb &getObb() const { return obb; }
  bool onRoad(const VectorD &position) const override;
  LaneId locate(const VectorD &position) const override;

  VectorD getPosition() const override { return obb.getPosition(); }

  /**
   * update counter
   */
  void setCounter(int v0, int v1) {
    counter[0] = v0;
    counter[1] = v1;
  }

  /**
   * 	give VEHICLE a suggestion on whether it should slow down on ramp
   *
   * 	@param vehicle the vehicle approaching ramp
   * 	@return whether the vehicle should slow down and the position of
   * 		imaginary barrier
   */
  std::pair<bool, VectorD> needSlowDown(roadmap::RoadId from) const;

private:
  /**
   *  counter[0]: the number of vehicles from the straight road
   *  counter[1]: the number of vehicles from the merged road
   */
  std::array<int, 2> counter{};
  std::array<RoadId, 3> roads;
  Obb obb;
};

class Offramp : public Road {
public:
  Offramp(RoadId id, const Obb &obb, std::array<RoadId , 3> roads)
       : Road(id), roads(roads), obb(obb) {
    nodeType = NodeType::Offramp;
  }

  const std::array<RoadId, 3> &getLinkedRoads() { return roads; }
  const Obb &getObb() const { return obb; }
  bool onRoad(const VectorD &position) const override;
  LaneId locate(const VectorD &position) const override;

  VectorD getPosition() const override { return obb.getPosition(); }

private:
  std::array<RoadId, 3> roads;
  Obb obb;
};

} // namespace roadmap
} // namespace ds

namespace ds {
namespace roadmap {

class StraightRoad : public Road {
public:
  StraightRoad(RoadId id, double width, const Segment &axis,
               std::array<VectorD, 4> anchors, std::vector<LaneInfo> inlanes);

  const double getWidth() const { return width; }

  const double getLength() const {
    return (axis.getEnd() - axis.getStart()).getLength();
  }

  const double getLaneWidth() const { return width / lanes.size(); }

  const Segment &getAxis() const { return axis; }

  const std::vector<Segment> &getEdges() const {
    assert(edges.size() == 1 + lanes.size());
    return edges;
  }

  const std::vector<LaneInfo> &getLanes() const { return lanes; }

  std::vector<LaneInfo> &retrieveReverseLanes(std::vector<LaneInfo> &res,
                                              uint8_t direction) {
    for (auto lane : lanes) {
      if (lane.isReversed() && lane.getDirection() == direction)
        res.emplace_back(lane);
    }

    return res;
  }

  std::vector<LaneInfo> &retrieveNonReverseLanes(std::vector<LaneInfo> &res,
                                                 uint8_t direction) {
    for (auto lane : lanes) {
      if (!lane.isReversed() && lane.getDirection() == direction)
        res.emplace_back(lane);
    }

    return res;
  }

  // Return the lanes with which OBB intersects
  std::vector<std::size_t> occupiedLanes(const Obb &obb) const;

  // Return the lane to which POS belongs
  std::size_t locatedLane(const VectorD &pos) const;

  double distance2Start(const VectorD &pos) const {
    return signedDistance(edges.front(), pos);
  }

  double distance2Bottom(const VectorD &pos) const {
    return signedDistance(edges.back(), pos);
  }

  // Return true if OBB and the boundary intersect
  bool checkCollision(const Obb &obb) const;

  // Return the axis of the corresponding lane.
  // One may use signedDistance provided in segment.h to compute
  // the width when changing lane
  Segment getLaneAxis(std::size_t laneNum) const;

  VectorD getRoadCoord(const VectorD &position) const;

  bool onRoad(const VectorD &position) const override;

  LaneId locate(const VectorD &position) const override;

  VectorD getPosition() const override {
    return (axis.getStart() + axis.getEnd()) / 2.0;
  }

  const Segment &getFromBorder() const { return linkedBorders.at(0); }

  const Segment &getToBorder() const { return linkedBorders.at(1); }

  std::pair<double, VehicleId> getFirstVPair(LaneId, bool);

private:
  double width;
  Segment axis;
  std::vector<LaneInfo> lanes;
  std::vector<Segment> edges;
  std::vector<Segment> linkedBorders;
};

} // namespace roadmap
} // namespace ds

#endif // DRIVE_SINGULARITY_ROADMAP_ROADS_H
