#ifndef DRIVE_SINGULARITY_RENDER_ROADMAPRENDER_H
#define DRIVE_SINGULARITY_RENDER_ROADMAPRENDER_H

#include <SDL.h>

#include "DriveSingularity/RoadMap/RoadMap.h"

namespace ds {
namespace render {


void renderStraightRoad(SDL_Window *window, SDL_Renderer *renderer,
                        const std::shared_ptr<roadmap::StraightRoad> &straight);


void renderRoadMap(SDL_Window *window, SDL_Renderer *renderer,
                   const roadmap::RoadMap &roadMap);
}
} // namespace ds

#endif // DRIVE_SINGULARITY_RENDER_ROADMAPRENDER_H
