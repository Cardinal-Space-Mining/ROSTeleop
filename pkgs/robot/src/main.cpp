
#include <memory>
#include <span>
#include <utility>
#include <vector>

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "custom_types/msg/talon_ctrl.hpp"
#include "custom_types/msg/talon_info.hpp"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

using namespace std::chrono_literals;

struct Gains
{
    double P, I, D, F;

    void apply_gains(BaseTalon & talon) const
    {
        talon.Config_kD(0, P);
        talon.Config_kP(0, I);
        talon.Config_kI(0, D);
        talon.Config_kF(0, F);
    }
};

namespace constants
{
static const std::string INTERFACE = "can0";
static constexpr Gains DEFAULT_GAINS{0.0, 0.0, 0.0, 0.2};
} // namespace constants

void apply_msg(BaseTalon & motor, const custom_types::msg::TalonCtrl & msg)
{
    printf("Applying: {Mode: %d, Value: %f}", msg.mode, msg.value );
    motor.Set(static_cast<ctre::phoenix::motorcontrol::ControlMode>(msg.mode),
              msg.value);
}

custom_types::msg::TalonInfo get_info(BaseTalon & talon)
{
    custom_types::msg::TalonInfo info;
    info.temperature = talon.GetTemperature();
    info.bus_voltage = talon.GetBusVoltage();
    info.output_percent = talon.GetMotorOutputPercent();
    info.output_voltage = talon.GetMotorOutputVoltage();
    info.output_current = talon.GetOutputCurrent();
    info.position = talon.GetSelectedSensorPosition();
    info.velocity = talon.GetSelectedSensorVelocity();
    return info;
}

class Robot : public rclcpp::Node
{
private:
    void info_periodic()
    {
        track_right_info->publish(get_info(track_right));
        track_left_info->publish(get_info(track_left));
        trencher_info->publish(get_info(trencher));
        hopper_actuator_info->publish(get_info(hopper_actuator));
        hopper_belt_info->publish(get_info(hopper_belt));
    }

    auto create_motor_info_pub(std::string name)
    {
        return this->create_publisher<custom_types::msg::TalonInfo>(name, 10);
    }

public:
    Robot()
    : rclcpp::Node("robot")
    , heartbeat_sub(this->create_subscription<std_msgs::msg::Int32>(
          "heartbeat", 10, [](const std_msgs::msg::Int32 & msg)
          { ctre::phoenix::unmanaged::Unmanaged::FeedEnable(msg.data); }))
    , info_timer(
          this->create_wall_timer(100ms, [this]() { this->info_periodic(); }))
    , track_right_ctrl(this->create_subscription<custom_types::msg::TalonCtrl>(
          "track_right_ctrl", 10,
          [this](const custom_types::msg::TalonCtrl & msg)
          { apply_msg(this->track_right, msg); }))
    , track_right_info(create_motor_info_pub("track_right_info"))
    , track_left_ctrl(this->create_subscription<custom_types::msg::TalonCtrl>(
          "track_left_ctrl", 10,
          [this](const custom_types::msg::TalonCtrl & msg)
          { apply_msg(this->track_left, msg); }))
    , track_left_info(create_motor_info_pub("track_left_info"))
    , trencher_ctrl(this->create_subscription<custom_types::msg::TalonCtrl>(
          "trencher_ctrl", 10, [this](const custom_types::msg::TalonCtrl & msg)
          { apply_msg(this->trencher, msg); }))
    ,

    trencher_info(create_motor_info_pub("trencher_info"))
    , hopper_belt_ctrl(this->create_subscription<custom_types::msg::TalonCtrl>(
          "hopper_belt_ctrl", 10,
          [this](const custom_types::msg::TalonCtrl & msg)
          { apply_msg(this->hopper_belt, msg); }))
    , hopper_belt_info(create_motor_info_pub("hopper_belt_info"))
    , hopper_actuator_ctrl(
          this->create_subscription<custom_types::msg::TalonCtrl>(
              "hopper_actuator_ctrl", 10,
              [this](const custom_types::msg::TalonCtrl & msg)
              { apply_msg(this->hopper_actuator, msg); }))
    , hopper_actuator_info(create_motor_info_pub("hopper_actuator_info"))
    {
        RCLCPP_DEBUG(this->get_logger(), "Starting Node INIT");

        // TODO
        std::array<std::reference_wrapper<BaseTalon>, 5> motors = {
            {track_right, track_left, trencher, hopper_belt, hopper_actuator}};

        for(auto & motor : motors)
        {
            constants::DEFAULT_GAINS.apply_gains(motor);
        }

        RCLCPP_DEBUG(this->get_logger(), "Initialized Node");
    }

private:
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> heartbeat_sub;
    rclcpp::TimerBase::SharedPtr info_timer;

private:
    TalonFX track_right{0, constants::INTERFACE};
    TalonFX track_left{1, constants::INTERFACE};
    TalonFX trencher{2, constants::INTERFACE};
    TalonFX hopper_belt{3, constants::INTERFACE};
    TalonSRX hopper_actuator{4, constants::INTERFACE};

private:
    using TalonCtrlSub =
        rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr;
    using TalonInfoPub =
        rclcpp::Publisher<custom_types::msg::TalonInfo>::SharedPtr;

    TalonCtrlSub track_right_ctrl;
    TalonInfoPub track_right_info;

    TalonCtrlSub track_left_ctrl;
    TalonInfoPub track_left_info;

    TalonCtrlSub trencher_ctrl;
    TalonInfoPub trencher_info;

    TalonCtrlSub hopper_belt_ctrl;
    TalonInfoPub hopper_belt_info;

    TalonCtrlSub hopper_actuator_ctrl;
    rclcpp::Publisher<custom_types::msg::TalonInfo>::SharedPtr
        hopper_actuator_info;

    // Set the can interface for pheonix5
};

int main(int argc, char ** argv)
{
    ctre::phoenix::unmanaged::Unmanaged::LoadPhoenix();

    std::cout << "Loaded Pheonix" << std::endl;

    // Init ROS2 for logging capabilities
    rclcpp::init(argc, argv);

    std::cout << "Loaded rclcpp" << std::endl;

    auto node = std::make_shared<Robot>();

    std::cout << "Loaded Bot Node" << std::endl;

    rclcpp::spin(node);

    std::cout << "Spun Node" << std::endl;
    rclcpp::shutdown();

    std::cout << "Loaded Done" << std::endl;
    return EXIT_SUCCESS;
}