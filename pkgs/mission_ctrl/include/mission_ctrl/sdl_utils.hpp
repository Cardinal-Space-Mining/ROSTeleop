#ifndef MISSION_CTRL_SDL_UTILS_HPP_6_27_2024
#define MISSION_CTRL_SDL_UTILS_HPP_6_27_2024

#include <SDL2/SDL.h>

struct SDLWindowDestroyer
{
    void operator()(SDL_Window* w) const;
};

struct SDLRendererDestroyer
{
    void operator()(SDL_Renderer* w) const;
};

#endif