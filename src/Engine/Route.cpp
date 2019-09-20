#include "Engine/Route.h"
#include <algorithm>
#include <chrono>
#include <map>
#include <queue>
#include <random>
#include <unordered_set>

namespace ds {
namespace engine {

struct PairHash {
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2> &p) const {
    auto h1 = std::hash<T1>{}(p.first);
    auto h2 = std::hash<T2>{}(p.second);
    return h1 ^ h2;
  }
};

// From node to node or road
std::pair<Route, bool> RoutePlanner::findRoute(DirectedRoad from, DirectedRoad to) const {
	std::unordered_map<DirectedRoad, DirectedRoad, PairHash> visitedMap;
	std::queue<DirectedRoad> queue;

	const auto &vertexes = roadGraph.getVertexes();
	const auto &edges = roadGraph.getEdges();

	visitedMap.insert({from, from});
	queue.push(from);

	while (!queue.empty()) {
		DirectedRoad current = queue.front();
		queue.pop();

		if (current == to) {
			std::list<DirectedRoad> roadList;
			roadList.push_front(current);
			while (current != from) {
				current = visitedMap.at(current);
				roadList.push_front(current);
			}
			return std::make_pair(Route(roadList, getTargetLanes(roadList)), true);
		}

		if (vertexes.find(current.first) != vertexes.end()) {  // current id related to a crossroad
			// TODO(ming): terminal or crossroad, default is crossroad
			auto node = vertexes.at(current.first);
			unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
			std::vector<roadmap::RoadId> copyRoads(node.linkedRoad);
			std::vector<DirectedRoad> linkedRoads;

			// check whether the next is reversed
			for (auto roadId: node.linkedRoad) {
				auto next = std::make_pair(roadId, edges.at(roadId).to == node.nodeId); // reverse
				if (visitedMap.find(next) == visitedMap.end()) linkedRoads.emplace_back(next);
			}

			// shuffle
			std::shuffle(std::begin(linkedRoads), std::end(linkedRoads), std::default_random_engine(seed));
			for (DirectedRoad next : linkedRoads) {
				visitedMap.insert({next, current});
				queue.push(next);
			}
		} else {  // current id related to a road
			auto road = edges.at(current.first);
			roadmap::RoadId next = current.second ? road.from : road.to;  // get next id (reverse or not)

			// determine the next one is road or crossroad
			if (edges.find(next) != edges.end()) { // if
				auto nextEdge = edges.at(next);
				DirectedRoad nextRoad = std::make_pair(next, nextEdge.to == current.first);
				visitedMap.insert({nextRoad, current});
				queue.push(nextRoad);
			} else {
				// cross road
				DirectedRoad nextRoad = DirectedRoad(next, false);
				visitedMap.insert({nextRoad, current});
				queue.push(nextRoad);
			}
		}
	}
	return std::make_pair(Route(), false);
}

//std::pair<Route, bool> RoutePlanner::reviseRoute(const Route &route, DirectedRoad start) const {
//  auto &oldRoadList = route.getRoadList();
//  std::unordered_set<DirectedRoad, PairHash> oldRoute;
//
//  for (auto directedRoad : oldRoadList) {
//    oldRoute.insert(directedRoad);
//  }
//
//  std::unordered_map<DirectedRoad, DirectedRoad, PairHash> visitedMap;
//  std::queue<DirectedRoad> queue;
//
//  visitedMap.insert({start, start});
//  queue.push(start);
//
//  while (!queue.empty()) {
//    DirectedRoad current = queue.front();
//    queue.pop();
//
//    if (oldRoute.find(current) != oldRoute.end()) {
//      std::list<DirectedRoad> roadList;
//      for (auto iter = oldRoadList.crbegin(); iter != oldRoadList.crend(); ++iter) {
//        roadList.push_front(*iter);
//        if (*iter == current) break;
//      }
//      while (current != start) {
//        current = visitedMap.at(current);
//        roadList.push_front(current);
//      }
//      return std::make_pair(Route(roadList, getTargetSegments(roadList)), true);
//    }
//
//    roadmap::NodeId nodeId =
//            current.second ? roadMap.getRoads().at(current.first)->getFrom()
//                           : roadMap.getRoads().at(current.first)->getTo();
//    auto node = roadMap.getNodes().at(nodeId);
//    auto crossRoads = std::dynamic_pointer_cast<roadmap::CrossRoads>(node);
//    // TODO: support other type of Node
//    if (!crossRoads)
//      continue;
//    auto &roadIds = roadInfo.getAssociatedRoads(nodeId);
//    for (auto roadId : roadIds) {
//      auto next = std::make_pair(roadId, roadMap.getRoads().at(roadId)->getTo() == nodeId);
//      if (visitedMap.find(next) == visitedMap.end()) {
//        visitedMap.insert({next, current});
//        queue.push(next);
//      }
//    }
//  }
//  return std::make_pair(Route(), false);
//}

/**
 * Retrieve available lanes.
 *
 * @param roadList list of DirectedRoad
 * @return
 */
std::list<roadmap::LaneInfo> RoutePlanner::getTargetLanes(std::list<DirectedRoad> roadList) const {
  std::list<roadmap::LaneInfo> targetLanes;
  const auto edges = roadGraph.getEdges();
  const auto vertexes = roadGraph.getVertexes();

  while (roadList.size() > 1) {
    DirectedRoad from = roadList.front();
    roadList.pop_front();

    DirectedRoad to = roadList.front();

	  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	  std::vector<roadmap::LaneInfo> candidateLanes;

    // retrieve lanes of current road
    if (edges.find(from.first) != edges.end()) {
    	// TODO(ming): add lanes
    	auto edge = edges.at(from.first);
    	auto road = roadMap.getRoad<roadmap::StraightRoad>(from.first);
    	auto lanes = road->getLanes();

    	// check next one is from or to
    	if (from.second) { // reverse
    		assert(edge.from == to.first);

		    uint8_t condition = roadmap::GoStraight; // TODO(ming): inference
		    road->retrieveReverseLanes(candidateLanes, condition);
    	} else { // non-reverse
    		assert(edge.to == to.first);

		    uint8_t condition = roadmap::GoStraight;
		    road->retrieveNonReverseLanes(candidateLanes, condition);
    	}
    } else { // current road is a crossroads
    	auto vertex = vertexes.at(from.first);

    	// judge last lane
    	roadmap::LaneInfo &lastLane = targetLanes.back();

    	// get last direction and road
    	auto lastDirection = lastLane.getDirection();
	    auto lastRoad = lastLane.getRoad();
	    auto nextRoad = to.first;

	    const auto &linkedRoads = vertex.linkedRoad;

	    // check direction of last road
	    auto lastIt = std::find(linkedRoads.begin(), linkedRoads.end(), lastRoad);
	    auto nextIt = std::find(linkedRoads.begin(), linkedRoads.end(), nextRoad);

	    int lastDir = std::distance(linkedRoads.begin(), lastIt);
	    int nextDir = std::distance(linkedRoads.begin(), nextIt);

	    uint8_t nextDirection = 0;

	    // TODO(ming): check relative position
	    int margin = lastDir - nextDir;
	    if (abs(margin) == 2) { // keep the same direction (naive)
	    	nextDirection = lastDirection;
	    } else if (margin == -1 || margin == 3) { // turn right
	    	// TODO(ming): modify or push new, current is
	    	nextDirection = roadmap::TurnRight;
	    } else if (margin == 1 || margin == -3) {
	    	nextDirection = roadmap::TurnLeft;
	    } else if (margin == 0) nextDirection = roadmap::TurnRound;

    	// retrieve next lanes
	    auto road = roadMap.getRoad<roadmap::StraightRoad>(to.first);
    	if (to.second) { // reverse
    		road->retrieveReverseLanes(candidateLanes, nextDirection);
    	} else { // non-reverse
    		road->retrieveNonReverseLanes(candidateLanes, nextDirection);
    	}
    }

	  // random select one, then push back
	  std::shuffle(std::begin(candidateLanes), std::end(candidateLanes), std::default_random_engine(seed));
	  if (!candidateLanes.empty()) targetLanes.emplace_back(candidateLanes[0]);
  }

  return targetLanes;
}

//roadmap::LaneInfo RoutePlanner::getTargetLane(DirectedRoad directedRoad, uint8_t direction, roadmap::NodeId nodeId) const {
////  auto &roadMap = roadInfo.getRoadMap();
//  auto road = roadMap.getRoads().at(directedRoad.first);
//  auto straightRoad = std::dynamic_pointer_cast<roadmap::StraightRoad>(road);
//  auto metaRoad = roadGraph.getEdges().at(straightRoad->getId());
//  assert(straightRoad); // TODO: other type of Node
//
//  // TODO(ming): straightRoad, retrieve available lanes
//  auto &laneInfos = straightRoad->getLanes();
//  size_t l, r;
//  for (l = 0; l < laneInfos.size(); ++l) {
//    if (laneInfos[l].isReversed() == directedRoad.second &&
//        (laneInfos[l].getDirection() & direction) != 0)
//      break;
//  }
//
//  assert(l < laneInfos.size());
//  for (r = l + 1; r < laneInfos.size(); ++r) {
//    if (laneInfos[r].isReversed() != directedRoad.second ||
//        (laneInfos[r].getDirection() & direction) == 0)
//      break;
//  }
//
//  // TODO(ming): retrieve available lanes
//  Segment lLine = road.getStraightRoadLane(straightRoad, l);
//  Segment rLine = roadInfo.getStraightRoadLane(straightRoad, r);
//
//  auto &lPoint = metaRoad.from == nodeId ? lLine.getStart() : lLine.getEnd();
//  auto &rPoint = metaRoad.from == nodeId ? rLine.getStart() : rLine.getEnd();
////      straightRoad->getFrom() == nodeId ? lLine.getStart() : lLine.getEnd();
////  auto &rPoint =
////      straightRoad->getFrom() == nodeId ? rLine.getStart() : rLine.getEnd();
//  return {lPoint, rPoint};
//
//}

} // namespace engine
} // namespace ds
