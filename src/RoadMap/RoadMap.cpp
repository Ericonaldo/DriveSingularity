#include "RoadMap/RoadMap.h"


namespace ds {
namespace roadmap {

LaneId RoadMap::locate(const VectorD &position) const {
  for (auto &kv : roads) {
	  if (kv.second->getRoadType() == RoadType::UNKNOWN || kv.second->getNodeType() == NodeType::Terminal) continue;
//    auto straight = getRoad<StraightRoad>(kv.first);
    auto road = getRoads().at(kv.first);
//    assert(straight);
		assert(road);
//    if (straight->onRoad(position))
//      return straight->locate(position);
		if (road->onRoad(position)) return road->locate(position);
  }
  return std::make_pair(ErrorRoadId, 0);
}

double RoadMap::calDistance(ds::VectorD &startPos, ds::VectorD &endPos, const std::list<LaneId> &route,
		const LaneId &endLane) const {
	double dis = 0;

	auto lastPos = startPos;

	double temp = 0;
	bool pass = false;

	for (const auto &it : route) {
		RoadId _id = it.first;

		auto road = roads.at(_id);
		assert(road);

		if (endLane.first == _id) {
			pass = true;
			break;
		}

		temp += (road->getPosition() - lastPos).getLength();
		lastPos = road->getPosition();
	}

	dis += (lastPos - endPos).getLength();

	if (pass) dis += temp;

	return dis;
}

int RoadMap::calOffset(bool reverse, double distance, const std::list<LaneId> &route) const {
	// TODO(ming): implement
	return 0;
}

} // namespace roadmap
} // namespace ds