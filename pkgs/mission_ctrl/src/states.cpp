#include "mission_ctrl/states.hpp"
#include "mission_ctrl/logitech_map.hpp"

MotorSettings TeleopStateMachine::update(const RobotState & robot,
                                         const sensor_msgs::msg::Joy & ctrl)
{
    return MotorSettings();
}