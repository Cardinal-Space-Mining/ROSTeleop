#ifndef MISSION_CTRL_APP_HPP_6_27_2024
#define MISSION_CTRL_APP_HPP_6_27_2024

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <SDL2/SDL.h>

#include "mission_ctrl/sdl_utils.hpp"

class Application : public rclcpp::Node
{
public:
    Application(int argc, char ** argv);

    bool is_running() const;

    void update();

private:
    const int WIDTH = 1028;
    const int HEIGHT = 940;

private:
    std::unique_ptr<SDL_Window, SDLWindowDestroyer> window;
    std::unique_ptr<SDL_Renderer, SDLRendererDestroyer> renderer;
    bool running = true;
};

#endif