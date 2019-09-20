#ifndef DRIVE_SINGULARITY_ROUTE_H
#define DRIVE_SINGULARITY_ROUTE_H

#include "DriveSingularity/Math/Segment.h"
#include "DriveSingularity/RoadMap/Common.h"
#include "DriveSingularity/RoadMap/Graph.h"
#include "DriveSingularity/RoadMap/RoadMap.h"
#include <list>

namespace ds {
namespace engine {

// std::pair<roadId, inverse>
using DirectedRoad = std::pair<roadmap::RoadId, bool>;

/**
 * Record route
 */
class Route {
public:
  Route() = default;
  Route(std::list<DirectedRoad> roadList,
        std::list<roadmap::LaneInfo> targetLanes)
      : roadList(std::move(roadList)), targetLanes(std::move(targetLanes)) {}

  const std::list<DirectedRoad> &getRoadList() const { return roadList; }

  std::list<DirectedRoad> &getRoadList() { return roadList; }

  const std::list<roadmap::LaneInfo> &getTargetLanes() const {
    return targetLanes;
  }

  std::list<roadmap::LaneInfo> &getTargetLanes() { return targetLanes; }

private:
  std::list<DirectedRoad> roadList;
  std::list<roadmap::LaneInfo> targetLanes;
};

/**
 * Generate route
 */
class RoutePlanner {
public:
  explicit RoutePlanner(const roadmap::Graph &roadGraph,
                        const roadmap::RoadMap &roadMap)
      : roadGraph(roadGraph), roadMap(roadMap) {}

  std::pair<Route, bool> findRoute(DirectedRoad from, DirectedRoad to) const;

  std::pair<Route, bool> reviseRoute(const Route &route,
                                     DirectedRoad start) const;

private:
  std::list<roadmap::LaneInfo>
  getTargetLanes(std::list<DirectedRoad> roadList) const;

  roadmap::LaneInfo getTargetLane(DirectedRoad directedRoad, uint8_t direction,
                                  roadmap::NodeId nodeId) const;

private:
  const roadmap::Graph &roadGraph;
  const roadmap::RoadMap &roadMap;
};

} // namespace engine
} // namespace ds

#endif // DRIVE_SINGULARITY_ROUTE_H
