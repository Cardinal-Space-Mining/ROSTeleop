
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

struct MotorInfo
{

    std::string motor_name;
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
private:
    void info_periodic()
    {
        for(size_t i = 0; i < m_motors.size(); i++)
        {

            custom_types::msg::TalonInfo info;
            auto& motor = m_motors[i];

            info.temperature = motor->GetTemperature();
            info.bus_voltage = motor->GetBusVoltage();

            info.output_percent = motor->GetMotorOutputPercent();
            info.output_voltage = motor->GetMotorOutputVoltage();
            info.output_current = motor->GetOutputCurrent();

            info.position = motor->GetSelectedSensorPosition();
            info.velocity = motor->GetSelectedSensorVelocity();

            this->talon_info_pubs[i]->publish(info);
        }
    }

public:
    Robot()
    : rclcpp::Node("robot")
    , heartbeat_sub(this->create_subscription<std_msgs::msg::Int32>(
          "heartbeat", 10, [](const std_msgs::msg::Int32 & msg)
          { ctre::phoenix::unmanaged::Unmanaged::FeedEnable(msg.data); }))
    , info_timer(
          this->create_wall_timer(100ms, [this]() { this->info_periodic(); }))
    {
        RCLCPP_DEBUG(this->get_logger(), "Starting Node INIT");


        // Create the node
        static const MotorInfo motors[] = {
            {"track_right", 0, MotorInfo::MotorType::TalonFX},
            {"track_left", 1, MotorInfo::MotorType::TalonFX},
            {"trencher", 2, MotorInfo::MotorType::TalonFX},
            {"hopper_belt", 3, MotorInfo::MotorType::TalonFX},
            {"hopper_actuator", 4, MotorInfo::MotorType::TalonSRX}};

        for(const auto & motor : motors)
        {
            std::unique_ptr<BaseTalon> talon =
                motor.type == MotorInfo::MotorType::TalonFX
                    ? static_cast<std::unique_ptr<BaseTalon>>(
                          std::make_unique<TalonFX>(motor.motor_id,
                                                    m_interface))
                    : static_cast<std::unique_ptr<BaseTalon>>(
                          std::make_unique<TalonSRX>(motor.motor_id,
                                                     m_interface));
            m_motors.emplace_back(std::move(talon));
        }

        // Create the pubs and subs AFTER all the motors are created so m_motors
        // does not reallocate and invalidate all the Callbacks that hold
        //  references to the motors
        for(std::size_t i = 0; i < std::size(motors); i++)
        {
            auto & motor = motors[i];
            auto pub = this->create_publisher<custom_types::msg::TalonInfo>(
                motor.motor_name + "_info", 10);
            auto sub = this->create_subscription<custom_types::msg::TalonCtrl>(
                motor.motor_name + "_ctrl", 10,
                [this, i](const custom_types::msg::TalonCtrl & msg)
                {
                    this->m_motors[i]->Set(
                        static_cast<ctre::phoenix::motorcontrol::ControlMode>(
                            msg.mode),
                        msg.value);
                });
            talon_ctrl_subs.emplace_back(sub);
            talon_info_pubs.emplace_back(pub);
        }

        RCLCPP_DEBUG(this->get_logger(), "Initialized Node");
    }

private:
    const std::string m_interface = "can0";

private:
    std::vector<std::unique_ptr<BaseTalon>> m_motors;
    std::vector<
        std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonCtrl>>>
        talon_ctrl_subs;
    std::vector<
        std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonInfo>>>
        talon_info_pubs;

private:
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> heartbeat_sub;
    rclcpp::TimerBase::SharedPtr info_timer;

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