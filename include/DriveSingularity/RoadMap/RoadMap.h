#ifndef DRIVE_SINGULARITY_ROADMAP_ROADMAP_H
#define DRIVE_SINGULARITY_ROADMAP_ROADMAP_H

#include <memory>
#include <list>
#include <unordered_map>

#include "DriveSingularity/RoadMap/Common.h"
#include "DriveSingularity/RoadMap/Roads.h"
#include "DriveSingularity/Math/Vector.h"
//#include "DriveSingularity/Engine/Route.h"

namespace ds {
namespace roadmap {


class RoadMap {
public:
	/**
	 * Add road to road map
	 *
	 * @param road a shared_ptr
	 */
  void addRoad(std::shared_ptr<Road> road) {
    auto id = road->getId();
    auto res = roads.emplace(id, std::move(road));
    assert(res.second);
  }

  /**
   * Add a Node to road map
   *
   * @param node a shared_ptr of Area (Road)
   */
  void addNode(std::shared_ptr<Road> node) {
  	auto id = node->getId();
  	auto res = nodes.emplace(id, std::move(node));
  	assert(res.second);
  }

  /**
   * Retrieve road with given road id.
   *
   * @tparam RoadType road type
   * @param id road id
   * @return typed road with dynamic ptr cast
   */
  template <class RoadType> std::shared_ptr<RoadType> getRoad(RoadId id) const {
    return std::dynamic_pointer_cast<RoadType>(roads.at(id));
  }

  /**
   * Retrieve node with given node id.
   *
   * @tparam NodeType node type
   * @param id node id
   * @return typed node with dynamic ptr cast
   */
  template <class NodeType> std::shared_ptr<NodeType> getNode(NodeId id) const {
  	return std::dynamic_pointer_cast<NodeType>(nodes.at(id));
  }

  /**
   * Retrieve road map without nodes.
   *
   * @return unordered map
   */
  const std::unordered_map<RoadId, std::shared_ptr<Road>> &getRoads() const { return roads; }

  /**
   * Retrieve node map without roads
   *
   * @return unordered map
   */
  const std::unordered_map<NodeId, std::shared_ptr<Road>> &getNodes() const { return nodes; }

  LaneId locate(const VectorD &position) const;

  double calDistance(VectorD &startPos, VectorD &endPos, const std::list<LaneId> &route, const LaneId &endLane) const;
  int calOffset(bool reverse, double distance, const std::list<LaneId> &route) const;

  void reset() {
    roads.clear();
    nodes.clear();
  }

private:
  std::unordered_map<RoadId, std::shared_ptr<Road>> roads;
  std::unordered_map<NodeId, std::shared_ptr<Road>> nodes;
};

} // namespace roadmap
} // namespace ds

#endif // DRIVE_SINGULARITY_ROADMAP_ROADMAP_H
