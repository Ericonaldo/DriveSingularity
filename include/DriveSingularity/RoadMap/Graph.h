#ifndef DRIVE_SINGULARITY_ROADMAP_GRAPH_H
#define DRIVE_SINGULARITY_ROADMAP_GRAPH_H

#include <unordered_map>
#include <vector>

#include "DriveSingularity/RoadMap/Common.h"

namespace ds {
namespace roadmap {

using NodeId = RoadId;

class Graph {
public:
  struct Node {
    Node(const NodeId id, const NodeType type) : nodeId(id), nodeType(type) {}

    std::vector<RoadId> linkedRoad;
    NodeId nodeId;
    NodeType nodeType;
  };

  struct LinkedPair { // for road
    LinkedPair(const NodeId from, const NodeId to) : from(from), to(to) {}
    NodeId from;
    NodeId to;
  };

public:
  void addEdge(RoadId from, RoadId to, RoadId road);
  void addVertex(VertexId main);

  const std::unordered_map<RoadId, LinkedPair> &getEdges() const {
    return edges;
  }
  const std::unordered_map<NodeId, Node> &getVertexes() const {
    return vertexes;
  }

  std::unordered_map<RoadId, LinkedPair> &getEdges() { return edges; }
  std::unordered_map<NodeId, Node> &getVertexes() { return vertexes; }

  void reset() {
    vertexes.clear();
    edges.clear();
  }

private:
  std::unordered_map<NodeId, Node>
      vertexes; // node is described by id and Node (record the linked nodes or
                // roads)
  std::unordered_map<RoadId, LinkedPair>
      edges; // edge is described by id, and linked pair of nodes
};

} // namespace roadmap
} // namespace ds

#endif // DRIVE_SINGULARITY_ROADMAP_GRAPH_H
