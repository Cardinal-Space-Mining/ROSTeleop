#include <algorithm>
#include <iostream>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>

#include <SDL2/SDL.h>

#include "mission_ctrl/app.hpp"

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600

int main(int argc, char * argv[])
{
    // Initialize SDL
    if(SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger(""),
                     "SDL could not be initialized! SDL_Error: %s",
                     SDL_GetError());
        std::exit(EXIT_FAILURE);
    }

    std::atexit(SDL_Quit);

#if defined linux && SDL_VERSION_ATLEAST(2, 0, 8)
    // Disable compositor bypass
    if(!SDL_SetHint(SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR, "0"))
    {
        RCLCPP_ERROR(rclcpp::get_logger(""),
                     "SDL can not disable compositor bypass!");
        std::exit(EXIT_FAILURE);
    }
#endif

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Application>(argc, argv);
    rclcpp::spin(node);
    rclcpp::shutdown();

    // because the application node uses RAII to hold stuff, we need to make
    // sure it is destroyed before calling SDL_Quit
    node = nullptr;

    return 0;
}