
#include <memory>
#include <span>
#include <utility>
#include <vector>

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "custom_types/msg/talon_ctrl.hpp"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class Robot : public rclcpp::Node
{
public:
    template <size_t V>
    Robot(std::span<const std::pair<const char *, int>, V> motors)
    : rclcpp::Node("robot")
    {
        for(const auto & motor : motors)
        {
            auto & motor_ref =
                m_motors.emplace_back(std::make_unique<TalonSRX>(motor.second));
            m_motor_subs.emplace_back(this->create_subscription<
                                      custom_types::msg::TalonCtrl>(
                motor.first, 10,
                [&motor_ref](const custom_types::msg::TalonCtrl & msg)
                {
                    motor_ref->Set(
                        static_cast<ctre::phoenix::motorcontrol::ControlMode>(
                            msg.mode),
                        msg.value);
                }));
        }
    }

private:
    std::vector<std::unique_ptr<TalonSRX>> m_motors;
    std::vector<
        std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonCtrl>>>
        m_motor_subs;
};

int main(int argc, char ** argv)
{
    // Init ROS2 for logging capabilities
    rclcpp::init(argc, argv);

    // Set the can interface for pheonix5
    const char* interface = "can0";
    if(ctre::phoenix::platform::can::SetCANInterface(interface) != 0)
    {

        RCLCPP_ERROR(rclcpp::get_logger("robot"),
                     "Failed to set the CAN interface to %s", interface);
        std::exit(EXIT_FAILURE);
    }

    // Create the node
    static constexpr std::pair<const char *, int> motors[] = {
        {"track_right", 0},
        {"track_left", 1},
        {"trencher", 2},
        {"hopper_belt", 3},
        {"hopper_actuator", 4}};

    auto node = std::make_shared<Robot>(
        std::span{std::begin(motors), std::end(motors)});

    // Run the node
    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}