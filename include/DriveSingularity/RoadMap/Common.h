#ifndef DRIVE_SINGULARITY_ROADMAP_COMMON_H
#define DRIVE_SINGULARITY_ROADMAP_COMMON_H

#include <cstddef>
#include <cstdint>
#include <functional>

namespace ds {
namespace roadmap {

enum class NodeType { UNKNOWN = 0, Terminal, Crossroads, Onramp, Offramp };
enum class RoadType { UNKNOWN = 0, Straight };

using RoadId = std::int64_t;
using NodeId = std::int64_t;

const RoadId ErrorRoadId = 0;

using LaneId = std::pair<RoadId, std::size_t>;

inline RoadId makeRoadId() {
  static RoadId curId = 1;
  return curId++;
}

inline bool isValidRoadId(RoadId id) { return id != ErrorRoadId; }

constexpr uint8_t GoStraight = 0x1;
constexpr uint8_t TurnLeft = 0x2;
constexpr uint8_t TurnRight = 0x4;
constexpr uint8_t TurnRound = 0x8;

class LaneInfo {
public:
  enum LineType {
    Striped,
    Continuous,
  };

  LaneInfo(RoadId road, bool reverse, uint8_t direction, LineType leftLineType,
           LineType rightLineType)
      : road(road), reverse(reverse), direction(direction), leftLineType(leftLineType),
        rightLineType(rightLineType) {}

  bool isReversed() const {
    return reverse;
  }

  uint8_t getDirection() const { return direction; }

  RoadId getRoad() const { return road; }

  LineType getLeftLineType() { return leftLineType; }
  LineType getRightLineType() { return rightLineType; }

  double getMinVelocity() const {
    return minVelocity;
  }

  double getMaxVelocity() const {
    return maxVelocity;
  }

  void setMinVelocity(double minVelocity) {
    LaneInfo::minVelocity = minVelocity;
  }

  void setMaxVelocity(double maxVelocity) {
    LaneInfo::maxVelocity = maxVelocity;
  }

private:
	RoadId road;
  bool reverse;
  uint8_t direction;
  LineType leftLineType;
  LineType rightLineType;
  double minVelocity = 0;
  double maxVelocity = 30;
};

/**
 * VertexId record the node id and node type
 */
class VertexId {
public:
  VertexId(const NodeId id, const NodeType type) : nodeId(id), nodeType(type) {}

  const RoadId getId() const { return nodeId; }
  const NodeType getType() const { return nodeType; }

  bool operator == (VertexId &v) const {
  	return nodeId == v.nodeId && nodeType == v.nodeType;
  }

//  const std::int32_t getAux() const { return aux; }

//  bool operator==(VertexId &o) const {
//    return roadId == o.roadId && aux == o.aux;
//  }

private:
  NodeId nodeId;
  NodeType nodeType;
};

} // namespace roadmap
} // namespace ds

#endif // DRIVE_SINGULARITY_ROADMAP_COMMON_H
