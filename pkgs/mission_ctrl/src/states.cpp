#include "mission_ctrl/states.hpp"
#include "mission_ctrl/logitech_map.hpp"

MotorSettings TeleopStateMachine::update(const RobotState & robot,
                                         const sensor_msgs::msg::Joy & ctrl)
{
    if(ctrl.axes.size() < LogitechMapping::Axes::NUM_AXES ||
       ctrl.buttons.size() < LogitechMapping::Buttons::NUM_BUTTONS)
    {
        return MotorSettings{};
    }

    constexpr int right_track = 0;
    constexpr int left_track = 1;
    constexpr int trencher = 2;
    constexpr int hopper_belt = 3;
    constexpr int hopper_actuator = 4;

    if(ctrl.buttons[LogitechMapping::Buttons::A])
    {
        this->set_state(State::Normal);
        return MotorSettings{};
    }

    switch(this->current_state)
    {
    case State::Normal: return normal_state(robot, ctrl);

    case State::Trench: return trench_state(robot, ctrl);

    case State::Offload: return offload_state(robot, ctrl);

    default: break;
    }
}

MotorSettings
TeleopStateMachine::normal_state(const RobotState & robot,
                                 const sensor_msgs::msg::Joy & ctrl)
{
    if(ctrl.buttons[LogitechMapping::Buttons::L_STICK])
    {
        this->set_state(State::Trench);
        return MotorSettings{};
    }

    if(ctrl.buttons[LogitechMapping::Buttons::R_STICK])
    {
        this->set_state(State::Offload);
        return MotorSettings{};
    }
}

MotorSettings
TeleopStateMachine::offload_state(const RobotState & robot,
                                  const sensor_msgs::msg::Joy & ctrl)
{
    return MotorSettings();
}

MotorSettings
TeleopStateMachine::trench_state(const RobotState & robot,
                                 const sensor_msgs::msg::Joy & ctrl)
{
    return MotorSettings();
}

void TeleopStateMachine::set_state(State state)
{
    switch(state)
    {
    case State::Normal: state_info = NormalInfo(); break;

    case State::Trench: state_info = TrenchInfo(); break;

    case State::Offload: state_info = OffloadInfo(); break;

    default: break;
    }
    this->current_state = state;
}