#ifndef MISSION_CTRL_STATES_HPP_6_27_2024
#define MISSION_CTRL_STATES_HPP_6_27_2024


enum class ControlType{
    KEYBOARD_CTRL = 0,
    GAMEPAD_CTRL = 1
};

enum class ControlState{
    Drive,
    Mine,
    Offload
};


#endif