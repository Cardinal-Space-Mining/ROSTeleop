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
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub;

    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>>
        right_track_ctrl;
    std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonInfo>>
        right_track_info;

    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>>
        left_track_ctrl;
    std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonInfo>>
        left_track_info;

    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>>
        trencher_ctrl;
    std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonInfo>>
        trencher_info;

    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>>
        hopper_belt_ctrl;
    std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonInfo>>
        hopper_belt_info;

    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>>
        hopper_actuator_ctrl;
    std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonInfo>>
        hopper_actuator_info;

    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>> talon_ctrl_pub(rclcpp::Node& parent, const std::string& name);

private:
    sensor_msgs::msg::Joy joy;
    RobotState robot_state;

private:
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