#ifndef MISSION_CTRL_APP_HPP_6_27_2024
#define MISSION_CTRL_APP_HPP_6_27_2024

#include <array>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <SDL.h>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include "mission_ctrl/sdl_utils.hpp"

#include "custom_types/msg/talon_ctrl.hpp"

class Application : public rclcpp::Node
{
public:
    Application(int argc, char ** argv);

    void update();

private:
    void handle_event(SDL_Event & e);

    void update_motors();

private:
    const int WIDTH = 1028;
    const int HEIGHT = 940;
    const std::array<uint8_t, 4> reset_color = {255, 255, 255, 255};
    const std::chrono::milliseconds frame_time = std::chrono::milliseconds(16);

private:
    float track_right_velo = 0;

private:
    std::unique_ptr<SDL_Window, SDLWindowDestroyer> window;
    std::unique_ptr<SDL_Renderer, SDLRendererDestroyer> renderer;

    ImGuiIO & io = ImGui::GetIO();

private:
    rclcpp::Publisher<custom_types::msg::TalonCtrl>::SharedPtr track_right_pub;

private:
    rclcpp::TimerBase::SharedPtr timer;
};

#endif