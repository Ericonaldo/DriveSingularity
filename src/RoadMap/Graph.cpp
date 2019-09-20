//
// Created by ming on 7/17/19.
//

#include <DriveSingularity/RoadMap/Common.h>
#include "RoadMap/Graph.h"

namespace ds {
namespace roadmap {
	/**
	 * Add edge to graph.
	 *
	 * This method will create node that has not been added to the graph and add information about the linked road to a
	 * couple of roads.
	 *
	 * @param from road id
	 * @param to road id
	 */
	void Graph::addEdge(RoadId from, RoadId to, RoadId road) {
		edges.emplace(road, LinkedPair(from, to));
	}

	/**
	 * Add linked vertex.
	 *
	 * one-way add.
	 *
	 * @param main main node Vertex
	 * @param linked linked node Vertex
	 */
	void Graph::addVertex(ds::roadmap::VertexId vertex) {
		auto id = vertex.getId();

		if (vertexes.find(id) == vertexes.end())
			vertexes.emplace(id, Node(id, vertex.getType()));
	}
}
}
