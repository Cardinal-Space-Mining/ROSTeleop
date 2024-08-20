
#include <memory>
#include <span>
#include <utility>
#include <vector>

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "custom_types/msg/talon_ctrl.hpp"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

namespace
{
const char * to_string(ctre::phoenix::motorcontrol::ControlMode mode)
{
    using namespace ctre::phoenix::motorcontrol;
    switch(mode)
    {
    case ControlMode::Position: return "Position";

    case ControlMode::Velocity: return "Velocity";

    case ControlMode::Current: return "Current";

    case ControlMode::Follower: return "Follower";

    case ControlMode::MotionProfile: return "MotionProfile";

    case ControlMode::MotionMagic: return "MotionMagic";

    case ControlMode::MotionProfileArc: return "MotionProfileArc";

    case ControlMode::MusicTone: return "MusicTone";
    case ControlMode::Disabled: return "Disabled";
    case ControlMode::PercentOutput: return "PercentOutput";
    }
    return "";
}
} // namespace

using namespace std::chrono_literals;

struct MotorInfo
{

    const char * motor_name;
    int motor_id;
    enum class MotorType
    {
        TalonFX,
        TalonSRX
    };
    MotorType type;
};

class Robot : public rclcpp::Node
{
public:
    template <size_t V>
    Robot(std::span<const MotorInfo, V> motors, const std::string & interface)
    : rclcpp::Node("robot")
    , timer(this->create_wall_timer(
          1000ms,
          []() { ctre::phoenix::unmanaged::Unmanaged::FeedEnable(2000); }))
    {
        for(const auto & motor : motors)
        {
            switch(motor.type)
            {
            case MotorInfo::MotorType::TalonFX:
                m_motors.emplace_back(
                    std::make_unique<TalonFX>(motor.motor_id, interface));
                break;

            case MotorInfo::MotorType::TalonSRX:
                m_motors.emplace_back(
                    std::make_unique<TalonSRX>(motor.motor_id, interface));
                break;

            default: break;
            }
        }

        for(std::size_t i = 0; i < motors.size(); i++)
        {
            auto motor_info = motors[i];
            std::unique_ptr<BaseMotorController> & motor_ref = m_motors[i];
            m_motor_subs.emplace_back(
                this->create_subscription<custom_types::msg::TalonCtrl>(
                    motors[i].motor_name, 10,
                    [&motor_ref, this,
                     motor_info](const custom_types::msg::TalonCtrl & msg)
                    {
                        auto ctrl_mode = static_cast<
                            ctre::phoenix::motorcontrol::ControlMode>(msg.mode);
                        RCLCPP_DEBUG_THROTTLE(this->get_logger(),
                                              *this->get_clock(), 1000,
                                              "Configured Motor %s: {%s,%f}",
                                              motor_info.motor_name,
                                              to_string(ctrl_mode), msg.value);

                        motor_ref->Set(ctrl_mode, msg.value);
                    }));
        }

        RCLCPP_DEBUG(this->get_logger(), "Initialized Node");
    }

private:
    std::vector<std::unique_ptr<BaseMotorController>> m_motors;
    std::vector<
        std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonCtrl>>>
        m_motor_subs;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char ** argv)
{
    // Init ROS2 for logging capabilities
    rclcpp::init(argc, argv);

    // Set the can interface for pheonix5
    std::string interface = "can0";

    // Create the node
    static constexpr MotorInfo motors[] = {
        {"track_right", 0, MotorInfo::MotorType::TalonFX},
        {"track_left", 1, MotorInfo::MotorType::TalonFX},
        {"trencher", 2, MotorInfo::MotorType::TalonFX},
        {"hopper_belt", 3, MotorInfo::MotorType::TalonFX},
        {"hopper_actuator", 4, MotorInfo::MotorType::TalonSRX}};

    auto motor_span = std::span{std::begin(motors), std::end(motors)};
    auto node = std::make_shared<Robot>(motor_span, interface);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}