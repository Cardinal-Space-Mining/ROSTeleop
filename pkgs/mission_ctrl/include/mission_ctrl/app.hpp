#ifndef MISSION_CTRL_APP_HPP_6_27_2024
#define MISSION_CTRL_APP_HPP_6_27_2024

#include <array>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int32.hpp>

#include <SDL.h>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include "mission_ctrl/sdl_utils.hpp"
#include "mission_ctrl/states.hpp"

#include "custom_types/msg/talon_ctrl.hpp"

using namespace std::chrono_literals;

class RobotTeleopInterface
{
private:
    void update_motors();

public:
    RobotTeleopInterface(rclcpp::Node & parent);

private:
    RobotState robot_state;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub;
    sensor_msgs::msg::Joy joy;
    std::vector<
        std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>>>
        talon_ctrl_pubs;
    std::vector<
        std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonInfo>>>
        talon_info_subs;
    static const std::array<std::string, 5> motors;
    rclcpp::TimerBase::SharedPtr teleop_update_timer;
    TeleopStateMachine teleop_state;
};

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
    bool bot_enabled = false;

    static constexpr auto ENABLE_TIME = 250ms;
    static constexpr auto BEAT_TIME = 100ms;

private:
    std::unique_ptr<SDL_Window, SDLWindowDestroyer> window;
    std::unique_ptr<SDL_Renderer, SDLRendererDestroyer> renderer;

    ImGuiIO & io = ImGui::GetIO();

private:
    rclcpp::Publisher<custom_types::msg::TalonCtrl>::SharedPtr track_right_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr heartbeat;

private:
    rclcpp::TimerBase::SharedPtr frame_timer;
    rclcpp::TimerBase::SharedPtr heartbeat_timer;

private:
    RobotTeleopInterface teleop_interface;
};

#endif