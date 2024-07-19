#ifndef MISSION_CTRL_APP_HPP_6_27_2024
#define MISSION_CTRL_APP_HPP_6_27_2024

#include <memory>
#include <array>

#include <rclcpp/rclcpp.hpp>

#include <SDL.h>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include "mission_ctrl/sdl_utils.hpp"

class Application : public rclcpp::Node
{
public:
    Application(int argc, char ** argv);

    bool is_running() const;

    void update();

private:
    void handle_event(SDL_Event & e);

private:
    const int WIDTH = 1028;
    const int HEIGHT = 940;
    const 

private:
    std::unique_ptr<SDL_Window, SDLWindowDestroyer> window;
    std::unique_ptr<SDL_Renderer, SDLRendererDestroyer> renderer;
    bool running = true;
    ImGuiIO& io = ImGui::GetIO();
};

#endif