#include "mission_ctrl/sdl_utils.hpp"

void SDLWindowDestroyer::operator()(SDL_Window * w) const
{
    SDL_DestroyWindow(w);
}

void SDLRendererDestroyer::operator()(SDL_Renderer * w) const {
    SDL_DestroyRenderer(w);
}
