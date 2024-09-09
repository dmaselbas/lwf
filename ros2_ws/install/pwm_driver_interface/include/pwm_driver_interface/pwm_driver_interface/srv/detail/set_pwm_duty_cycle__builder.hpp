// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pwm_driver_interface:srv/SetPwmDutyCycle.idl
// generated code does not contain a copyright notice

#ifndef PWM_DRIVER_INTERFACE__SRV__DETAIL__SET_PWM_DUTY_CYCLE__BUILDER_HPP_
#define PWM_DRIVER_INTERFACE__SRV__DETAIL__SET_PWM_DUTY_CYCLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pwm_driver_interface/srv/detail/set_pwm_duty_cycle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pwm_driver_interface
{

namespace srv
{

namespace builder
{

class Init_SetPwmDutyCycle_Request_duty_cycle
{
public:
  explicit Init_SetPwmDutyCycle_Request_duty_cycle(::pwm_driver_interface::srv::SetPwmDutyCycle_Request & msg)
  : msg_(msg)
  {}
  ::pwm_driver_interface::srv::SetPwmDutyCycle_Request duty_cycle(::pwm_driver_interface::srv::SetPwmDutyCycle_Request::_duty_cycle_type arg)
  {
    msg_.duty_cycle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pwm_driver_interface::srv::SetPwmDutyCycle_Request msg_;
};

class Init_SetPwmDutyCycle_Request_channel
{
public:
  Init_SetPwmDutyCycle_Request_channel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPwmDutyCycle_Request_duty_cycle channel(::pwm_driver_interface::srv::SetPwmDutyCycle_Request::_channel_type arg)
  {
    msg_.channel = std::move(arg);
    return Init_SetPwmDutyCycle_Request_duty_cycle(msg_);
  }

private:
  ::pwm_driver_interface::srv::SetPwmDutyCycle_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pwm_driver_interface::srv::SetPwmDutyCycle_Request>()
{
  return pwm_driver_interface::srv::builder::Init_SetPwmDutyCycle_Request_channel();
}

}  // namespace pwm_driver_interface


namespace pwm_driver_interface
{

namespace srv
{

namespace builder
{

class Init_SetPwmDutyCycle_Response_duty_cycle
{
public:
  explicit Init_SetPwmDutyCycle_Response_duty_cycle(::pwm_driver_interface::srv::SetPwmDutyCycle_Response & msg)
  : msg_(msg)
  {}
  ::pwm_driver_interface::srv::SetPwmDutyCycle_Response duty_cycle(::pwm_driver_interface::srv::SetPwmDutyCycle_Response::_duty_cycle_type arg)
  {
    msg_.duty_cycle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pwm_driver_interface::srv::SetPwmDutyCycle_Response msg_;
};

class Init_SetPwmDutyCycle_Response_channel
{
public:
  Init_SetPwmDutyCycle_Response_channel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPwmDutyCycle_Response_duty_cycle channel(::pwm_driver_interface::srv::SetPwmDutyCycle_Response::_channel_type arg)
  {
    msg_.channel = std::move(arg);
    return Init_SetPwmDutyCycle_Response_duty_cycle(msg_);
  }

private:
  ::pwm_driver_interface::srv::SetPwmDutyCycle_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pwm_driver_interface::srv::SetPwmDutyCycle_Response>()
{
  return pwm_driver_interface::srv::builder::Init_SetPwmDutyCycle_Response_channel();
}

}  // namespace pwm_driver_interface

#endif  // PWM_DRIVER_INTERFACE__SRV__DETAIL__SET_PWM_DUTY_CYCLE__BUILDER_HPP_
