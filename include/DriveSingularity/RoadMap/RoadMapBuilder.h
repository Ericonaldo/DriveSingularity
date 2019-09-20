/**
 * This file is part of include/DriveSingularity/RoadMap.
 *
 * RoadMapBuilder will build a road map and return a constructed graph for route planning
 */
#ifndef DRIVE_SINGULARITY_ROADMAP_ROADMAPFACTORY_H
#define DRIVE_SINGULARITY_ROADMAP_ROADMAPFACTORY_H

#include <array>
#include <cassert>
#include <unordered_map>
#include <utility>
#include <vector>

#include "DriveSingularity/Math/All.h"
#include "DriveSingularity/RoadMap/Common.h"
#include "DriveSingularity/RoadMap/RoadMap.h"
#include "DriveSingularity/RoadMap/Graph.h"
#include "DriveSingularity/RoadMap/Roads.h"

namespace ds {
namespace roadmap {

struct NodeInfo {
	NodeInfo(NodeType type, const VectorD &Position, double Rotation, NodeId id=-1)
			: id(id >= 0 ? id : makeRoadId()), type(type), position(Position), rotation(Rotation) {
	}

	RoadId id;
	NodeType type;
	VectorD position;
	double rotation;
};

struct RoadInfo {
	/**
	 * Road info.
	 *
	 * Basic road information includes its algebra description (i.e,
	 * {fromTop, fromBottom, toTop, toBottom}, and road Type.
	 *
	 * @param Type road type.
	 * @param From linked from road id.
	 * @param To linked to road id.
	 * @param Width axis length.
	 * @param Lanes meta lanes information.
	 * @param fromTop top vector of `from` line.
	 * @param fromBottom bottom vector of `from` line.
	 * @param toTop top vector of `to` line.
	 * @param toBottom bottom vector of `to` line
	 * @param id road id
	 */
	RoadInfo(RoadType Type, RoadId From, RoadId To, double Width, std::vector<LaneInfo> Lanes,
	         const VectorD &fromTop,
	         const VectorD &fromBottom,
	         const VectorD &toTop,
	         const VectorD &toBottom,
	         RoadId id=-1)
			: id(id >= 0 ? id : makeRoadId()), type(Type), from(From), to(To), width(Width),
			fromTop(fromTop), fromBottom(fromBottom), toTop(toTop), toBottom(toBottom),
			lanes(std::move(Lanes)) {}

	RoadId id;
	RoadType type;
	RoadId from, to;
	double width;

	VectorD fromTop, fromBottom, toTop, toBottom;
	std::vector<LaneInfo> lanes;
};

class RoadMapBuilder {

public:
  /**
   * Add a crossroad.
   *
   * @param center center position
   * @param rotation rotation
   * @return node id
   */
  RoadId addCrossroads(const VectorD &center, double rotation = 0, NodeId id = -1) {
    auto nodeId = addNode(NodeType::Crossroads, center, rotation, id);

    // create road recorder for cross
    roadsAtCross[nodeId];

    return nodeId;
  }

  /**
   * Add a terminal.
   *
   * The terminal added to road map is flagged by its center position and its rotation.
   *
   * @param center a VectorD instance describes the center position.
   * @param rotation rotation angle ranges from 0 to 2 * Pi which determines the direction.
   * @return RoadId of this terminal determined by a centralized RoadId producer.
   */
  RoadId addTerminal(const VectorD &center, double rotation = 0, NodeId id = -1) {
    auto nodeId = addNode(NodeType::Terminal, center, rotation, id);

    // create road recorder for terminal
    roadAtTerminal[nodeId];
    return nodeId;
  }

  RoadId addOnramp(const VectorD &center, double rotation = 0, NodeId id = -1) {
    auto nodeId = addNode(NodeType::Onramp, center, rotation, id);

    roadsAtOnramp[nodeId];
    return nodeId;
  }

  RoadId addOfframp(const VectorD &center, double rotation = 0, NodeId id = -1) {
    auto nodeId = addNode(NodeType::Offramp, center, rotation, id);

    roadsAtOfframp[nodeId];
    return nodeId;
  }

  /**
   * Add a straight road.
   *
   * ...
   *
   * @param from
   * @param to
   * @param width
   * @param lanes
   * @return RoadId of this straight road info entity determined by a centralized RoadId producer.
   */
  RoadId addStraightRoad(RoadId from, RoadId to, double width,
                         const std::vector<LaneInfo> &lanes,
                         const VectorD &fromTop, const VectorD &fromBottom,
                         const VectorD &toTop, const VectorD &toBottom, RoadId id = -1) {
    auto info = RoadInfo(RoadType::Straight, from, to, width, lanes,
    		fromTop, fromBottom, toTop, toBottom, id);
    roadsInfo.emplace(info.id, info);
    return info.id;
  }

  /**
   * Link road to given crossroad.
   *
   * @param roadId id of linked road.
   * @param crossId crossroad id.
   * @param dir linked direction.
   */
  void registerRoadAtCrossroads(RoadId roadId, NodeId crossId, std::size_t dir) {
    assert(isValidRoadId(roadId));
    roadsAtCross.at(crossId).at(dir) = roadId;
  }

  /**
   * Link a road to a terminal.
   *
   * This function will link a road and terminal with given ids, and register this road to a road map.
   *
   * @param roadId id of linked road.
   * @param terminalId id of linked terminal.
   */
  void registerRoadAtTerminal(RoadId roadId, NodeId terminalId) {
    roadAtTerminal[terminalId] = roadId;
  }

  void registerRoadAtOnramp(RoadId roadId, NodeId onrampId, std::size_t dir) {
    assert(isValidRoadId(roadId));
    roadsAtOnramp.at(onrampId).at(dir) = roadId;
  }

  void registerRoadAtOfframp(RoadId roadId, NodeId offrampId, std::size_t dir) {
    assert(isValidRoadId(roadId));
    roadsAtOfframp.at(offrampId).at(dir) = roadId;
  }

	void buildRoadMap();
	void parseJSON(const std::string &filename);
	void parseXML(const std::string &osmFileName, const std::string &jsonFileName);

	Graph getGraph() { return graph; }
	RoadMap getRoadMap() { return roadMap; }

private:
  /**
   * Create a node.
   *
   * This method will not create a node entity, but return a node info entity which parameterized by given attributions.
   *
   * @param type node type.
   * @param center a VectorD instance which describes the center position.
   * @param rotation angle ranges from 0 to 2 * Pi which describes the node rotation.
   * @package (optional) id distribute an id to node
   * @return the RoadId of this node determined by a centralized NodeId producer.
   */
  RoadId addNode(NodeType type, const VectorD &center, double rotation, NodeId id) {
    auto info = NodeInfo(type, center, rotation, id);
    nodesInfo.emplace(info.id, info);

    return info.id;
  }

private:

  std::shared_ptr<Crossroads> makeCrossroads(RoadId crossId);  // called by buildRoadMap
  std::shared_ptr<Terminal> makeTerminal(RoadId terminalId);  // called by buildRoadMap
  std::shared_ptr<Onramp> makeOnramp(RoadId onrampId);
  std::shared_ptr<Offramp> makeOfframp(RoadId offrampId);
  std::shared_ptr<StraightRoad> makeStraightRoad(RoadId roadId);  // called by buildRoadMap

private:
  std::unordered_map<RoadId, std::array<RoadId, 4>> roadsAtCross;
  std::unordered_map<RoadId, std::array<RoadId, 3>> roadsAtOnramp;
  std::unordered_map<RoadId, std::array<RoadId, 3>> roadsAtOfframp;
  std::unordered_map<RoadId, RoadId> roadAtTerminal;
  std::unordered_map<RoadId, NodeInfo> nodesInfo;
  std::unordered_map<RoadId, RoadInfo> roadsInfo;

private:
  Graph graph;
  RoadMap roadMap;
};

} // namespace roadmap
} // namespace ds

#endif // DRIVE_SINGULARITY_ROADMAP_ROADMAPFACTORY_H
