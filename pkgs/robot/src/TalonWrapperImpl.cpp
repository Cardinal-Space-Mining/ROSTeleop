#include "TalonWrapper.hpp"

namespace Robot
{
auto P6_MOTOR_SETPOINT_ACC = 5_tr_per_s_sq;
};

// ----------- Talon FX5 -----------

template <>
TalonWrapper<TalonFX5>::TalonWrapper(rclcpp::Node & parent_node,
                                     std::string name, int id)
: m_motor(id)
, m_sub(this->create_ctrl_sub(parent_node, name))
, m_pub(this->create_info_pub(parent_node, name))
, m_info_timer(this->create_info_timer(parent_node))
{
}

template <>
void TalonWrapper<TalonFX5>::config(const Gains & gains, BrakeMode break_mode)
{

    m_motor.Config_kD(0, gains.P);
    m_motor.Config_kP(0, gains.I);
    m_motor.Config_kI(0, gains.D);
    m_motor.Config_kF(0, gains.F);
    switch(break_mode)
    {
    case BrakeMode::BRAKE:
        m_motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

        /* code */
        break;
    case BrakeMode::COAST:
        m_motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
        break;

    default: break;
    }
}

template <>
void TalonWrapper<TalonFX5>::on_ctrl_msg(
    const custom_types::msg::TalonCtrl & msg)
{
    this->m_motor.Set(
        static_cast<ctre::phoenix::motorcontrol::ControlMode>(msg.mode),
        msg.value);
}

template <>
custom_types::msg::TalonInfo TalonWrapper<TalonFX5>::get_info()
{
    custom_types::msg::TalonInfo info;
    info.temperature = this->m_motor.GetTemperature();
    info.bus_voltage = this->m_motor.GetBusVoltage();
    info.output_percent = this->m_motor.GetMotorOutputPercent();
    info.output_voltage = this->m_motor.GetMotorOutputVoltage();
    info.output_current = this->m_motor.GetOutputCurrent();
    info.position = this->m_motor.GetSelectedSensorPosition();
    info.velocity = this->m_motor.GetSelectedSensorVelocity();
    return info;
}

// ----------- TalonSRX -----------

template <>
TalonWrapper<TalonSRX>::TalonWrapper(rclcpp::Node & parent_node,
                                     std::string name, int id)
: m_motor(id)
, m_sub(this->create_ctrl_sub(parent_node, name))
, m_pub(this->create_info_pub(parent_node, name))
, m_info_timer(this->create_info_timer(parent_node))
{
}

template <>
void TalonWrapper<TalonSRX>::config(const Gains & gains, BrakeMode break_mode)
{

    m_motor.Config_kD(0, gains.P);
    m_motor.Config_kP(0, gains.I);
    m_motor.Config_kI(0, gains.D);
    m_motor.Config_kF(0, gains.F);

    switch(break_mode)
    {
    case BrakeMode::BRAKE:
        m_motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

        /* code */
        break;
    case BrakeMode::COAST:
        m_motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
        break;

    default: break;
    }
}

template <>
void TalonWrapper<TalonSRX>::on_ctrl_msg(
    const custom_types::msg::TalonCtrl & msg)
{
    this->m_motor.Set(
        static_cast<ctre::phoenix::motorcontrol::ControlMode>(msg.mode),
        msg.value);
}

template <>
custom_types::msg::TalonInfo TalonWrapper<TalonSRX>::get_info()
{
    custom_types::msg::TalonInfo info;
    info.temperature = this->m_motor.GetTemperature();
    info.bus_voltage = this->m_motor.GetBusVoltage();
    info.output_percent = this->m_motor.GetMotorOutputPercent();
    info.output_voltage = this->m_motor.GetMotorOutputVoltage();
    info.output_current = this->m_motor.GetOutputCurrent();
    info.position = this->m_motor.GetSelectedSensorPosition();
    info.velocity = this->m_motor.GetSelectedSensorVelocity();
    return info;
}

// ----------- TalonFX6 -----------

template <>
TalonWrapper<TalonFX6>::TalonWrapper(rclcpp::Node & parent_node,
                                     std::string name, int id)
: m_motor(id)
, m_sub(this->create_ctrl_sub(parent_node, name))
, m_pub(this->create_info_pub(parent_node, name))
, m_info_timer(this->create_info_timer(parent_node))
{
}

template <>
void TalonWrapper<TalonFX6>::config(const Gains & gains, BrakeMode break_mode)
{
    phoenix6::configs::TalonFXConfiguration
        generic_config{} /*, tracks_config{}*/;

    /* Voltage-based velocity requires a feed forward to account for the
     * back-emf of the motor */
    generic_config.Slot0.kP = gains.P;
    generic_config.Slot0.kI = gains.I;
    generic_config.Slot0.kD = gains.F;

    generic_config.MotorOutput.NeutralMode =
        break_mode == BrakeMode::BRAKE
            ? ctre::phoenix6::signals::NeutralModeValue::Brake
            : ctre::phoenix6::signals::NeutralModeValue::Coast;

    generic_config.CurrentLimits.StatorCurrentLimitEnable = false;

    m_motor.GetConfigurator().Apply(generic_config);
}

template <>
void TalonWrapper<TalonFX6>::on_ctrl_msg(
    const custom_types::msg::TalonCtrl & msg)
{
    switch(msg.mode)
    {
    case custom_types::msg::TalonCtrl::PERCENT_OUTPUT:
        m_motor.SetControl(ctre::phoenix6::controls::DutyCycleOut(msg.value));
        break;

    case custom_types::msg::TalonCtrl::VELOCITY:
        m_motor.SetControl(ctre::phoenix6::controls::VelocityVoltage{
            units::angular_velocity::turns_per_second_t{msg.value},
            Robot::P6_MOTOR_SETPOINT_ACC, false});
        break;
    default:
        throw std::runtime_error(
            "Not Implemented TalonWrapper<TalonFX6>::on_ctrl_msg");
    }
}

template <>
custom_types::msg::TalonInfo TalonWrapper<TalonFX6>::get_info()
{
    custom_types::msg::TalonInfo info;
    info.temperature = this->m_motor.GetDeviceTemp().GetValueAsDouble();
    info.bus_voltage = this->m_motor.GetMotorVoltage().GetValueAsDouble();
    info.output_percent = this->m_motor.GetDutyCycle().GetValueAsDouble();
    info.output_voltage = this->m_motor.GetMotorVoltage().GetValueAsDouble();
    info.output_current = this->m_motor.GetStatorCurrent().GetValueAsDouble();
    info.position = this->m_motor.GetPosition().GetValueAsDouble();
    info.velocity = this->m_motor.GetVelocity().GetValueAsDouble();
    return info;
}
