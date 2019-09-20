#ifndef DRIVE_SINGULARITY_RENDER_ENVIRONMENTRENDERER_H
#define DRIVE_SINGULARITY_RENDER_ENVIRONMENTRENDERER_H

#include <SDL.h>

#include "DriveSingularity/Engine/Environment.h"

namespace ds {
namespace render {

void renderEnvironment(SDL_Window *window, SDL_Renderer *renderer,
                       const engine::Environment &env);
}
} // namespace ds

#endif // DRIVE_SINGULARITY_RENDER_ENVIRONMENTRENDERER_H
