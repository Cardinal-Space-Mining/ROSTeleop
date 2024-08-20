#ifndef MISSION_CTRL_STATES_HPP_6_27_2024
#define MISSION_CTRL_STATES_HPP_6_27_2024

#include "sensor_msgs/msg/joy.hpp"

#include "custom_types/msg/talon_ctrl.hpp"
#include "custom_types/msg/talon_info.hpp"

using MotorSettings = std::vector<custom_types::msg::TalonCtrl>;
using RobotState = std::vector<custom_types::msg::TalonInfo>;

class TeleopStateMachine
{
public:
    MotorSettings update(const RobotState & robot,
                         const sensor_msgs::msg::Joy & ctrl);

private:
    enum class State
    {
        Normal = 0,
        Trench = 1,
        Offload = 2
    };
};

#endif