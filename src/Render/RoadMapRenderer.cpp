#include "Render/RoadMapRenderer.h"

#include <array>
#include <vector>

namespace ds {
namespace render {

void renderStraightRoad(
    SDL_Window *window, SDL_Renderer *renderer,
    const std::shared_ptr<roadmap::StraightRoad> &straight) {
  int h;
  SDL_GetWindowSize(window, nullptr, &h);

  auto &edges = straight->getEdges();

  SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
  for (auto &edge : edges) {
    SDL_RenderDrawLine(renderer, edge.getStart().getX(),
                       h - edge.getStart().getY(), edge.getEnd().getX(),
                       h - edge.getEnd().getY());
  }

  //  SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
  //  SDL_RenderDrawLine(renderer, edges.first.getStart().getX(),
  //                     h - edges.first.getStart().getY(),
  //                     edges.first.getEnd().getX(),
  //                     h - edges.first.getEnd().getY());
  //  SDL_RenderDrawLine(renderer, edges.second.getStart().getX(),
  //                     h - edges.second.getStart().getY(),
  //                     edges.second.getEnd().getX(),
  //                     h - edges.second.getEnd().getY());
  //
  //  // lanes
  //  SDL_SetRenderDrawColor(renderer, 90, 90, 90, SDL_ALPHA_OPAQUE);
  //  auto lanes = roadInfo.getStraightRoadLanes(straight);
  //  for (auto &lane : lanes) {
  //    SDL_RenderDrawLine(renderer, lane.getStart().getX(),
  //                       h - lane.getStart().getY(), lane.getEnd().getX(),
  //                       h - lane.getEnd().getY());
  //  }
}

void renderRoadMap(SDL_Window *window, SDL_Renderer *renderer,
                   const roadmap::RoadMap &roadMap) {
  int w, h;
  SDL_GetWindowSize(window, &w, &h);

  // roads
  SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
  for (auto &kv : roadMap.getRoads()) {
  	if (roadMap.getRoads().at(kv.first)->getRoadType() != roadmap::RoadType::Straight) continue;
    auto straight = roadMap.getRoad<roadmap::StraightRoad>(kv.first);
    assert(straight);
    renderStraightRoad(window, renderer, straight);
  }

  // nodes
  //  SDL_SetRenderDrawColor(renderer, 100, 0, 0, SDL_ALPHA_OPAQUE);
  //  for (auto &kv : roadMap.getNodes()) {
  //    auto &node = kv.second;
  //    if (std::dynamic_pointer_cast<roadmap::TerminalPoint>(node)) {
  //      continue; // TODO
  //    }
  //
  //    auto cross = std::dynamic_pointer_cast<roadmap::CrossRoads>(node);
  //    assert(cross);
  //    auto crossObb = roadInfo.getCrossRoad(cross);
  //    auto vertices = crossObb.getVertices();
  //    for (std::size_t i = 0; i < 4; ++i) {
  //      SDL_RenderDrawLine(renderer, vertices[i].getX(), h -
  //      vertices[i].getY(),
  //                         vertices[(i + 1) % 4].getX(),
  //                         h - vertices[(i + 1) % 4].getY());
  //    }
  //  }
}

} // namespace render
} // namespace ds