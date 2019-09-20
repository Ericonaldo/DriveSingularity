#include "RoadMap/RoadMapBuilder.h"
#include <fstream>
#include <iostream>
#include <set>
#include <json/json.h>
#include <json/value.h>

#include <pugixml.hpp>

namespace ds {
namespace roadmap {

/**
 * Make a crossroad entity.
 *
 * This function will return a shared ptr of Obb, the entity of a crossroad,
 * namely, a crossroad is described as a rectangle.
 *
 * NOTE: called when build a road map
 *
 * @param crossId node id
 * @return shared ptr of Obb like <nodeId, &Obb>
 */
std::shared_ptr<Crossroads> RoadMapBuilder::makeCrossroads(RoadId crossId) {
  auto nodeInfo = nodesInfo.at(crossId);
  auto &associateRoads = roadsAtCross.at(crossId);

  std::array<VectorD, 4> centers;
  for (std::size_t i = 0; i < 4; ++i) {
    const auto &roadInfo = roadsInfo.at(associateRoads.at(i));
    if (roadInfo.to == crossId) {
      centers[i] = (roadInfo.toTop + roadInfo.toBottom) / 2;
    } else {
      assert(roadInfo.from == crossId);
      centers[i] = (roadInfo.fromTop + roadInfo.fromBottom) / 2;
    }
  }

  double halfLength = (centers[0] - centers[2]).getLength() / 2;
  double halfWidth = (centers[1] - centers[3]).getLength() / 2;

  // rotate this Obb instance to match the neighboring roads
  auto obb = Obb(halfLength, halfWidth, nodeInfo.position, nodeInfo.rotation);
  // TODO(ming): cast type
  return std::make_shared<Crossroads>(nodeInfo.id, std::move(obb),
                                      associateRoads);
}

/**
 * Make a terminal entity.
 *
 * A terminal is described using Segment, and this function will return a shared
 * ptr of segment.
 *
 * NOTE: called when build a road map
 *
 * @param terminalId node id
 * @return shared ptr of Segment like <nodeId, Segment>
 */
std::shared_ptr<Terminal> RoadMapBuilder::makeTerminal(RoadId terminalId) {
  RoadId linkedRoad = roadAtTerminal.at(terminalId);
  RoadInfo roadInfo = roadsInfo.at(linkedRoad);

  assert(roadInfo.type != RoadType::UNKNOWN);

  // TODO(ming): refactor makeTerminal (overlap to marginal ?)
  Segment seg = Segment((roadInfo.fromTop + roadInfo.fromBottom) / 2,
                        (roadInfo.toTop + roadInfo.toBottom) / 2);
  return std::make_shared<Terminal>(terminalId, seg);
}

std::shared_ptr<Onramp> RoadMapBuilder::makeOnramp(RoadId onrampId) {
	auto &linkedRoads = roadsAtOnramp.at(onrampId);

	auto &road1 = roadsInfo.at(linkedRoads.at(1));
	auto &road0 = roadsInfo.at(linkedRoads.at(0));

	double halfLength;
	if (road1.from == onrampId) {
		halfLength = (road1.fromBottom - road1.fromTop).getLength() / 2.;
	} else {
		halfLength = (road1.toBottom - road1.toTop).getLength() / 2.;
	}

	double halfWidth = road0.width / 2.;

	auto& nodeInfo = nodesInfo.at(onrampId);

	// rotate this Obb instance to match the neighboring roads
	auto obb = Obb(halfLength, halfWidth, nodeInfo.position, nodeInfo.rotation);
  return std::make_shared<Onramp>(onrampId, std::move(obb), linkedRoads);
}

std::shared_ptr<Offramp> RoadMapBuilder::makeOfframp(RoadId offrampId) {
  auto &linkedRoads = roadsAtOfframp.at(offrampId);

  auto &road1 = roadsInfo.at(linkedRoads.at(1));
  auto &road0 = roadsInfo.at(linkedRoads.at(0));

  double halfLength;
  if (road1.from == offrampId) {
    halfLength = (road1.fromBottom - road1.fromTop).getLength() / 2.;
  } else {
    halfLength = (road1.toBottom - road1.toTop).getLength() / 2.;
  }

  double halfWidth = road0.width / 2.;

  auto& nodeInfo = nodesInfo.at(offrampId);

  auto obb = Obb(halfLength, halfWidth, nodeInfo.position, nodeInfo.rotation);
  return std::make_shared<Offramp>(offrampId, std::move(obb), linkedRoads);
}

/**
 * Make a straight road entity.
 *
 * This function will return a shared ptr of StraightRoad with recorded id,
 * namely, this road must be registered in current builder.
 *
 * NOTE: called when build a road map
 *
 * @param roadId id of registered road
 * @return shared ptr of StraightRoad like <roadId, ...
 */
std::shared_ptr<StraightRoad> RoadMapBuilder::makeStraightRoad(RoadId roadId) {
  RoadInfo roadInfo = roadsInfo.at(roadId);
  Segment seg = Segment((roadInfo.fromTop + roadInfo.fromBottom) / 2,
                        (roadInfo.toTop + roadInfo.toBottom) / 2);
  std::array<VectorD, 4> anchors = {roadInfo.fromTop, roadInfo.fromBottom,
                                    roadInfo.toTop, roadInfo.toBottom};
  return std::make_shared<StraightRoad>(roadId, roadInfo.width, seg, anchors,
                                        roadInfo.lanes);
}

/**
 * Build road map and graph.
 *
 * This method will build a road entity map and a abstract graph
 */
void RoadMapBuilder::buildRoadMap() {
  // dealing with node
  for (auto &kv : nodesInfo) {
    auto &node = kv.second;
    if (node.type == NodeType::Terminal) {
      roadMap.addRoad(makeTerminal(node.id));

      RoadId linkedRoad = roadAtTerminal.at(node.id);
      graph.addVertex(VertexId(node.id, NodeType::Terminal));

      auto &terminal = graph.getVertexes().at(node.id);
      terminal.linkedRoad.emplace_back(linkedRoad); // link road
    } else if (node.type == NodeType::Crossroads) {
      roadMap.addRoad(makeCrossroads(node.id));

      auto &linkedRoads = roadsAtCross.at(node.id);
      graph.addVertex(VertexId(node.id, NodeType::Crossroads));

      auto &crossNode = graph.getVertexes().at(node.id);
      for (RoadId road : linkedRoads)
        crossNode.linkedRoad.emplace_back(road); // link roads
    } else if (node.type == NodeType::Onramp) {
      roadMap.addRoad(makeOnramp(node.id));

      auto &linkedRoads = roadsAtOnramp.at(node.id);
      graph.addVertex(VertexId(node.id, NodeType::Onramp));

      auto &onrampNode = graph.getVertexes().at(node.id);
      for (RoadId road : linkedRoads)
        onrampNode.linkedRoad.emplace_back(road);
    } else if (node.type == NodeType::Offramp) {
      roadMap.addRoad(makeOfframp(node.id));

      auto &linkedRoads = roadsAtOfframp.at(node.id);
      graph.addVertex(VertexId(node.id, NodeType::Offramp));

      auto &offrampNode = graph.getVertexes().at(node.id);
      for (RoadId road : linkedRoads)
        offrampNode.linkedRoad.emplace_back(road);
    }
  }

  // dealing with roads
  for (auto &kv : roadsInfo) {
    auto &road = kv.second;
    if (road.type == RoadType::Straight) {
      roadMap.addRoad(makeStraightRoad(road.id));
      graph.addEdge(road.from, road.to, road.id);
    }
  }
}

/**
 * Build road map using outer JSON file.
 */
void RoadMapBuilder::parseJSON(const std::string &filename) {
  std::ifstream ifs(filename, std::ifstream::binary);
  Json::Reader reader;
  Json::Value jsonObj;

  if (!reader.parse(ifs, jsonObj, true)) {
    std::cout << "Failed to parse this json file.\n"
              << reader.getFormattedErrorMessages() << std::endl;
  }

  auto roadArray = jsonObj["roads"];
  auto intersectionArray = jsonObj["intersections"];

  // register road
  if (!roadArray.isNull()) {
    for (const auto &road : roadArray) {
      RoadId from = road["from"].asInt64();
      RoadId to = road["to"].asInt64();
      auto lanes = road["lanes"];
      auto ft = road["ft"], fb = road["fb"];
      auto tt = road["tt"], tb = road["tb"];

      if (road["type"] == "straight") {
        const double width = road["width"].asFloat();
        const RoadId id = road["id"].asInt64();
        const VectorD ft_vec = VectorD(ft[0].asFloat(), ft[1].asFloat());
        const VectorD fb_vec = VectorD(fb[0].asFloat(), fb[1].asFloat());
        const VectorD tt_vec = VectorD(tt[0].asFloat(), tt[1].asFloat());
        const VectorD tb_vec = VectorD(tb[0].asFloat(), tb[1].asFloat());

        std::vector<LaneInfo> laneInfos;
        for (const auto &lane : lanes) {
          bool reverse = lane["reverse"].asBool();
          uint8_t direction = 0;
          if (lane["goStraight"].asBool())
            direction |= GoStraight;
          if (lane["turnLeft"].asBool())
            direction |= TurnLeft;
          if (lane["turnRight"].asBool())
            direction |= TurnRight;
          if (lane["turnBack"].asBool())
            direction |= TurnRound;
          LaneInfo::LineType leftLineType = lane["leftLine"] == "continuous"
                                                ? LaneInfo::Continuous
                                                : LaneInfo::Striped;
          LaneInfo::LineType rightLineType = lane["rightLine"] == "continuous"
                                                 ? LaneInfo::Continuous
                                                 : LaneInfo::Striped;
          laneInfos.emplace_back(id, reverse, direction, leftLineType,
                                 rightLineType);
        }
        // TODO(ming): add lanes
        addStraightRoad(from, to, width, laneInfos, ft_vec, fb_vec, tt_vec,
                        tb_vec, id);
      } else {
        throw std::invalid_argument("Illegal road type.");
      }
    }
  }

  // register node
  if (!intersectionArray.isNull()) {
    for (auto &intersect : intersectionArray) {
      const NodeId id = intersect["id"].asInt64();
      const double rotation = intersect["rotation"].asDouble();
      const auto center = intersect["center"];

      if (intersect["type"] == "crossroad") {
        const auto linked = intersect["linkedRoads"];
        //     3
        // 0       2
        //     1
        addCrossroads(VectorD(center[0].asFloat(), center[1].asFloat()),
                      rotation, id);
        int a[4];
        for (int i = 0; i < 4; i++)
          a[i] = linked[i].asInt64();
        for (auto i = 0; i < 4; i++)
          registerRoadAtCrossroads(linked[i].asInt64(), id, i);
      } else if (intersect["type"] == "terminal") {
        const RoadId linked = intersect["linkedRoad"].asInt64();
        addTerminal(VectorD(center[0].asFloat(), center[1].asFloat()), rotation,
                    id);
        registerRoadAtTerminal(linked, id);
      } else if (intersect["type"] == "onramp") {
        const auto linked = intersect["linkedRoads"];
        addOnramp(VectorD(center[0].asFloat(), center[1].asFloat()),
                  rotation, id);
        int a[3];
        for (int i = 0; i < 3; i++)
          a[i] = linked[i].asInt64();
        for (auto i = 0; i < 3; i++)
          registerRoadAtOnramp(linked[i].asInt64(), id, i);
      } else if (intersect["type"] == "offramp") {
        const auto linked = intersect["linkedRoads"];
        addOfframp(VectorD(center[0].asFloat(), center[1].asFloat()),
                   rotation, id);
        int a[3];
        for (int i = 0; i < 3; i++)
          a[i] = linked[i].asInt64();
        for (auto i = 0; i < 3; i++)
          registerRoadAtOfframp(linked[i].asInt64(), id, i);
      } else {
        throw std::invalid_argument("Illegal intersection type.");
      }
    }
  }
}

static double DefaultLaneWidth = 40;
static size_t DefaultLaneNumber = 2; // each direction

std::string coordToString(VectorD position) {
  return "[" + std::to_string(position.getX()) + ", " + std::to_string(position.getY()) + "]";
}

std::string keyPairToString(const std::string &key, const std::string &value) {
  return "\"" + key + "\": " + value;
}

std::string roadToString(size_t id, size_t from, size_t to, double width,
                          const VectorD &ft, const VectorD &fb, const VectorD &tt, const VectorD &tb){
  std::string lanesString;
  for (size_t k = 0; k < DefaultLaneNumber << 1; ++k) {
    std::string laneString;
    if (k < DefaultLaneNumber) laneString = keyPairToString("reverse", "true") + ",";
    else laneString = keyPairToString("reverse", "false") + ",";
    laneString += keyPairToString("goStraight", "true") + ",";
    std::string leftFlag = k == DefaultLaneNumber || k == DefaultLaneNumber - 1 ? "true" : "false";
    std::string rightFlag = k == 0 || k == (DefaultLaneNumber << 1) - 1 ? "true" : "false";
    laneString += keyPairToString("turnLeft", leftFlag) + ",";
    laneString += keyPairToString("turnRight", rightFlag) + ",";
    laneString += keyPairToString("turnRound", "false") + ",";
    laneString += keyPairToString("leftLine", "\"continuous\"") + ",";
    laneString += keyPairToString("rightLine", "\"continuous\"");
    lanesString += "{" + laneString + "}";
    if (k < (DefaultLaneNumber << 1) - 1) lanesString += ",";
  }
  return "{" + keyPairToString("id", std::to_string(id)) + "," +
          keyPairToString("type", "\"straight\"") + "," +
          keyPairToString("from", std::to_string(from)) + "," +
          keyPairToString("to", std::to_string(to)) + "," +
          keyPairToString("width", std::to_string(width)) + "," +
          keyPairToString("ft", coordToString(ft)) + "," +
          keyPairToString("fb", coordToString(fb)) + "," +
          keyPairToString("tt", coordToString(tt)) + "," +
          keyPairToString("tb", coordToString(tb)) + "," +
          keyPairToString("lanes", "[" + lanesString + "]") + "}";
}


void RoadMapBuilder::parseXML(const std::string &osmFileName, const std::string &jsonFileName) {
  std::map<std::size_t, VectorD> nodeMap;
  pugi::xml_document doc;
  assert(doc.load_file(osmFileName.c_str()));
  auto osm = doc.child("osm");
  double minX = osm.child("bounds").attribute("minlon").as_float(),
  minY = osm.child("bounds").attribute("minlat").as_float();
  auto nodes = osm.children("node");
  size_t maxID = 0;
  for (const auto &node : nodes) {
    size_t id = node.attribute("id").as_int();
    nodeMap[id]= VectorD(node.attribute("lon").as_float() - minX,node.attribute("lat").as_float() - minY);
    maxID = std::max(id, maxID);
  }
  /*for (const auto &node: nodeMap) {
    std::cout << node.first << ":" << node.second.getX() << " " << node.second.getY() << std::endl;
  }*/
  auto ways = osm.children("way");
  std::map<size_t, std::set<size_t>> adjacentNodeMap;
  for (const auto &way : ways) {
    auto wayPoints = way.children("nd");
    size_t lastNode = wayPoints.begin()->attribute("ref").as_uint();
    for (const auto &wayPoint : wayPoints) {
      size_t currentNode = wayPoint.attribute("ref").as_uint();
      if (currentNode != lastNode) {
        adjacentNodeMap[currentNode].insert(lastNode);
        adjacentNodeMap[lastNode].insert(currentNode);
      }
      lastNode = currentNode;
    }
  }
  std::map<size_t, std::map<size_t, size_t >> crossingMap;
  //TODO(cyx): rotate crossing zone for better connection
  for (const auto &node : adjacentNodeMap) {
    //TODO(cyx): support for onramp and fork road
    if (node.second.size() != 4) continue;
    const auto &centerID = node.first;
    const auto &center = nodeMap[centerID];
    double minDis = (nodeMap[*adjacentNodeMap[centerID].begin()] - center).getLength();
    for (const auto &adjacentNodeID : adjacentNodeMap[centerID]) {
      minDis = std::min(minDis, (nodeMap[adjacentNodeID] - center).getLength());
    }
    minDis /= 2;
    auto tempAdjacentNodeSet = adjacentNodeMap[centerID];
    for (const auto &adjacentNodeID : tempAdjacentNodeSet) {
      const auto &adjacentNode = nodeMap[adjacentNodeID];
      auto delta = adjacentNode - center;
      ++maxID;
      auto mid = center;
      size_t directionType = 0;
      if (delta.getY() >= fabs(delta.getX())) {
        mid.getY() += minDis;
        directionType = 3;
      } else if (delta.getY() < -fabs(delta.getX())) {
        mid.getY() -= minDis;
        directionType = 1;
      } else if (delta.getX() > fabs(delta.getY())) {
        mid.getX() += minDis;
        directionType = 2;
      } else if (delta.getX() < -fabs(delta.getY())) {
        mid.getX() -= minDis;
        directionType = 0;
      }
      nodeMap[maxID] = mid;
      adjacentNodeMap[maxID].insert(centerID);
      adjacentNodeMap[maxID].insert(adjacentNodeID);
      adjacentNodeMap[centerID].erase(adjacentNodeID);
      adjacentNodeMap[centerID].insert(maxID);
      adjacentNodeMap[adjacentNodeID].erase(centerID);
      adjacentNodeMap[adjacentNodeID].insert(maxID);
      crossingMap[centerID][directionType] = maxID;
    }
  }
  /*for (const auto &node : adjacentNodeMap) {
    std::cout << "[" << node.first << "] (" << nodeMap[node.first].getX() << "," << nodeMap[node.first].getY() << ")";
    for (const auto &adjacentNode : node.second) {
      std::cout << " " << adjacentNode;
    }
    std::cout << std::endl;
  }*/
  // each node has 1, 2 or 4 degrees now
  std::map<size_t, std::pair<VectorD, VectorD>> anchorMap;
  for (const auto &node : nodeMap) {
    size_t centerID = node.first;
    const auto &center = nodeMap[centerID];
    if (adjacentNodeMap[centerID].size() == 2) {
      const auto &lastNode = nodeMap[*adjacentNodeMap[centerID].begin()];
      const auto &nextNode = nodeMap[*adjacentNodeMap[centerID].rbegin()];
      // calculate anchors
      // pay attention to directions
      auto delta = (rotate90(normalize(lastNode - center)) - rotate90(normalize(nextNode - center))) / 2;
      delta = normalize(delta) / delta.getLength() * DefaultLaneNumber * DefaultLaneWidth;
      anchorMap[centerID] = {center - delta, center + delta};
    }
  }
  std::map<std::pair<size_t, size_t >, size_t > roadIDMap;
  for (const auto &node : adjacentNodeMap) {
    size_t centerID = node.first;
    for (const auto &adjacentNodeID : node.second) {
      if (centerID < adjacentNodeID) {
        ++maxID;
        roadIDMap[std::make_pair(centerID, adjacentNodeID)] = maxID;
        roadIDMap[std::make_pair(adjacentNodeID, centerID)] = maxID;
      }
    }
  }
  std::string roadString;
  for (const auto &node : anchorMap) { // each node in anchorMap keys has 2 degrees
    size_t centerID = node.first;
    const auto &center = nodeMap[centerID];
    for (const auto &adjacentNodeID : adjacentNodeMap[centerID]) {
      if (adjacentNodeMap[adjacentNodeID].size() != 2) {
        auto adjacentNode = nodeMap[adjacentNodeID];
        auto delta = rotate90(normalize(adjacentNode - center));
        delta = delta * DefaultLaneWidth * DefaultLaneNumber;
        if (adjacentNodeMap[adjacentNodeID].size() == 4) {
          adjacentNode = adjacentNode - normalize(adjacentNode - center) * DefaultLaneWidth * DefaultLaneNumber * 1.15;
        }
        if (!roadString.empty()) roadString += ",";
        auto centerAnchor = anchorMap[centerID];
        size_t lastRoadID;
        if (adjacentNodeID == *adjacentNodeMap[centerID].rbegin()) {
          std::swap(centerAnchor.first, centerAnchor.second);
          lastRoadID = roadIDMap[std::make_pair(centerID, *adjacentNodeMap[centerID].begin())];
        } else {
          lastRoadID = roadIDMap[std::make_pair(centerID, *adjacentNodeMap[centerID].rbegin())];
        }
        roadString += roadToString(roadIDMap[std::make_pair(centerID, adjacentNodeID)], lastRoadID, adjacentNodeID, DefaultLaneNumber * DefaultLaneWidth * 2,
                                   centerAnchor.first, centerAnchor.second, adjacentNode - delta,
                                   adjacentNode + delta);
      } else if (adjacentNodeID > centerID) { // avoid add twice
        if (!roadString.empty()) roadString += ",";
        auto centerAnchor = anchorMap[centerID];
        size_t lastRoadID;
        if (adjacentNodeID == *adjacentNodeMap[centerID].rbegin()) {
          std::swap(centerAnchor.first, centerAnchor.second);
          lastRoadID = roadIDMap[std::make_pair(centerID, *adjacentNodeMap[centerID].begin())];
        } else {
          lastRoadID = roadIDMap[std::make_pair(centerID, *adjacentNodeMap[centerID].rbegin())];
        }
        size_t nextRoadID;
        auto adjacentAnchor = anchorMap[adjacentNodeID];
        if (centerID == *adjacentNodeMap[adjacentNodeID].rbegin()) {
          nextRoadID = roadIDMap[std::make_pair(adjacentNodeID, *adjacentNodeMap[adjacentNodeID].begin())];
        } else {
          std::swap(adjacentAnchor.first, adjacentAnchor.second);
          nextRoadID = roadIDMap[std::make_pair(adjacentNodeID, *adjacentNodeMap[adjacentNodeID].rbegin())];
        }
        roadString += roadToString(roadIDMap[std::make_pair(centerID, adjacentNodeID)], lastRoadID, nextRoadID, DefaultLaneNumber * DefaultLaneWidth * 2,
                                   centerAnchor.first, centerAnchor.second, adjacentAnchor.first,
                                   adjacentAnchor.second);
      }
    }
  }
  for (const auto &node : adjacentNodeMap) {
    if (node.second.size() == 1 && adjacentNodeMap[*node.second.begin()].size() == 1 &&
        node.first < *node.second.begin()) {
      const auto &centerID = node.first;
      const auto &center = nodeMap[centerID];
      const auto &adjacentNodeID = *node.second.begin();
      const auto &adjacentNode = nodeMap[adjacentNodeID];
      auto delta = rotate90(normalize(adjacentNode - center));
      delta = delta * DefaultLaneWidth * DefaultLaneNumber;
      if (!roadString.empty()) roadString += ",";
      roadString += roadToString(roadIDMap[std::make_pair(centerID, adjacentNodeID)], centerID, adjacentNodeID, DefaultLaneNumber * DefaultLaneWidth * 2,
                                 center - delta, center + delta, adjacentNode - delta, adjacentNode + delta);
    }
  }
  std::string roadsString = keyPairToString("roads", "[" + roadString + "]");
  std::string nodeString;
  for (const auto &node : adjacentNodeMap) {
    const auto &centerID = node.first;
    const auto &adjacentNodes = node.second;
    if (adjacentNodes.size() == 1) {
      if (!nodeString.empty()) {
        nodeString += ",";
      }
      nodeString += "{";
      nodeString += keyPairToString("id", std::to_string(centerID)) + ",";
      nodeString += keyPairToString("type", "\"terminal\"") + ",";
      nodeString += keyPairToString("linkedRoad", std::to_string(roadIDMap[std::make_pair(centerID, *adjacentNodes.begin())]));
      nodeString += "}";
    }
    if (adjacentNodes.size() == 4) {
      if (!nodeString.empty()) {
        nodeString += ",";
      }
      nodeString += "{";
      nodeString += keyPairToString("id", std::to_string(centerID)) + ",";
      nodeString += keyPairToString("type", "\"crossroad\"") + ",";
      std::string listString;
      for (size_t k = 0; k < 4; ++k) {
        listString += std::to_string(roadIDMap[std::make_pair(centerID, crossingMap[centerID][k])]);
        if (k != 3) {
          listString += ",";
        }
      }
      nodeString += keyPairToString("linkedRoads", "[" + listString + "]") + ",";
      nodeString += keyPairToString("center", coordToString(nodeMap[centerID]));
      nodeString += "}";
    }

  }
  std::string nodesString = keyPairToString("intersections", "[" + nodeString + "]");
  std::ofstream ofstream(jsonFileName, std::ofstream::binary);
  ofstream << "{" <<  roadsString << "," << nodesString << "}";
  ofstream.close();
}
} // namespace roadmap
} // namespace ds