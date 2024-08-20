
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

    const char * motor_name;
    int motor_id;
    enum class MotorType
    {
        TalonFX,
        TalonSRX
    };
    MotorType type;
};

class InstantiatedMotor
{
private:
    static std::unique_ptr<BaseTalon>
    create_motor(const MotorInfo & motor_info, const std::string & interface)
    {
        switch(motor_info.type)
        {
        case MotorInfo::MotorType::TalonSRX:
            return std::make_unique<TalonSRX>(motor_info.motor_id, interface);
        case MotorInfo::MotorType::TalonFX:
            return std::make_unique<TalonFX>(motor_info.motor_id, interface);

        default: throw std::runtime_error("Unacounted Path");
        }
    }

    void on_msg(const custom_types::msg::TalonCtrl & msg)
    {
        m_motor->Set(
            static_cast<ctre::phoenix::motorcontrol::ControlMode>(msg.mode),
            msg.value);
    }

public:
    custom_types::msg::TalonInfo get_info()
    {
        custom_types::msg::TalonInfo info;

        info.temperature = m_motor->GetTemperature();
        info.bus_voltage = m_motor->GetBusVoltage();

        info.output_percent = m_motor->GetMotorOutputPercent();
        info.output_voltage = m_motor->GetMotorOutputVoltage();
        info.output_current = m_motor->GetOutputCurrent();

        info.position = m_motor->GetSelectedSensorPosition();
        info.velocity = m_motor->GetSelectedSensorVelocity();

        return info;
    }

public:
    InstantiatedMotor(const MotorInfo & motor_info,
                      const std::string & interface, rclcpp::Node & parent)
    : m_motor(InstantiatedMotor::create_motor(motor_info, interface))
    , m_ctrl_sub(parent.create_subscription<custom_types::msg::TalonCtrl>(
          std::string{motor_info.motor_name} + "_ctrl", 10,
          std::bind(&InstantiatedMotor::on_msg, this, std::placeholders::_1)))
    , m_info_pub(parent.create_publisher<custom_types::msg::TalonInfo>(
          std::string{motor_info.motor_name} + "_info", 10))

    {
    }

public:
    std::unique_ptr<BaseTalon> m_motor;
    std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonCtrl>>
        m_ctrl_sub;
    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonInfo>> m_info_pub;
};

class Robot : public rclcpp::Node
{
private:
    void info_periodic()
    {
        for(auto & motor : m_motors)
        {
            auto info = motor.get_info();
            motor.m_info_pub->publish(info);
        }
    }

public:
    Robot()
    : rclcpp::Node("robot")
    , heartbeat_sub(this->create_subscription<std_msgs::msg::Int32>(
          "heartbeat", 10, [](const std_msgs::msg::Int32 & msg)
          { ctre::phoenix::unmanaged::Unmanaged::FeedEnable(msg.data); }))
    , info_timer(this->create_wall_timer(
          100ms, std::bind(&Robot::info_periodic, this)))
    {

        // Create the node
        static constexpr MotorInfo motors[] = {
            {"track_right", 0, MotorInfo::MotorType::TalonFX},
            {"track_left", 1, MotorInfo::MotorType::TalonFX},
            {"trencher", 2, MotorInfo::MotorType::TalonFX},
            {"hopper_belt", 3, MotorInfo::MotorType::TalonFX},
            {"hopper_actuator", 4, MotorInfo::MotorType::TalonSRX}};

        for(const auto & motor : motors)
        {
            m_motors.emplace_back(motor, "can0", *this);
        }

        RCLCPP_DEBUG(this->get_logger(), "Initialized Node");
    }

private:
    const std::string m_interface = "can0";

private:
    std::vector<InstantiatedMotor> m_motors;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> heartbeat_sub;
    rclcpp::TimerBase::SharedPtr info_timer;

    // Set the can interface for pheonix5
};

int main(int argc, char ** argv)
{
    // Init ROS2 for logging capabilities
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Robot>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}