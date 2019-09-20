#include "Render/EnvironmentRenderer.h"

#include "Render/RoadMapRenderer.h"

namespace ds {
namespace render {
namespace {

void renderVehicle(SDL_Window *window, SDL_Renderer *renderer,
                   const engine::Environment &env,
                   const std::shared_ptr<control::VehicleController> &vehicle) {
//  int r = 0, g = 0, b = 0;
//  auto roads = env.occupiedRoads(vehicle->getId());
//  std::size_t occupiedLaneCnt = 0;
//  for (auto &road : roads) {
//    occupiedLaneCnt += env.occupiedLanes(vehicle->getId(), road).size();
//  }
//  if (occupiedLaneCnt > 1) {
//    r = g = 0;
//    b = 255;
//  }
//  for (auto &road : roads) {
//    if (env.checkCollision(vehicle->getId(), road)) {
//      r = 255;
//      g = b = 0;
//      break;
//    }
//  }

  SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);

  int h;
  SDL_GetWindowSize(window, nullptr, &h);
  auto vertices = vehicle->getVertices();
  for (std::size_t i = 0; i < 4; ++i) {
//    if ((i == 0 && vehicle->isTurnLeft()) || (i == 2 && vehicle->isTurnRight()))
//      SDL_SetRenderDrawColor(renderer, 255, 165, 0, 0);
//    if (i == 1 && vehicle->isDecelerate()) SDL_SetRenderDrawColor(renderer, 255, 0, 0, 0);
    SDL_RenderDrawLine(renderer, vertices[i].getX(), h - vertices[i].getY(),
                       vertices[(i + 1) % 4].getX(),
                       h - vertices[(i + 1) % 4].getY());
    if (vehicle->getVehicleType() == ds::control::Agent) {
    	SDL_SetRenderDrawColor(renderer, 0, 255, 0, 0);
    }
    else {
    	SDL_SetRenderDrawColor(renderer, 0, 0, 255, 0);
    }
  }
}

} // namespace

void renderEnvironment(SDL_Window *window, SDL_Renderer *renderer,
                       const engine::Environment &env) {
  renderRoadMap(window, renderer, env.getRoadMap());
  for (auto &kv : env.getVehicles()) {
    auto &vehicle = kv.second;
    renderVehicle(window, renderer, env, vehicle);
  }
}

} // namespace render
} // namespace ds