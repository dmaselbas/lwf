// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pwm_driver_interface:msg/Pwm.idl
// generated code does not contain a copyright notice

#ifndef PWM_DRIVER_INTERFACE__MSG__DETAIL__PWM__BUILDER_HPP_
#define PWM_DRIVER_INTERFACE__MSG__DETAIL__PWM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pwm_driver_interface/msg/detail/pwm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pwm_driver_interface
{

namespace msg
{

namespace builder
{

class Init_Pwm_duty_cycle
{
public:
  explicit Init_Pwm_duty_cycle(::pwm_driver_interface::msg::Pwm & msg)
  : msg_(msg)
  {}
  ::pwm_driver_interface::msg::Pwm duty_cycle(::pwm_driver_interface::msg::Pwm::_duty_cycle_type arg)
  {
    msg_.duty_cycle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pwm_driver_interface::msg::Pwm msg_;
};

class Init_Pwm_channel
{
public:
  Init_Pwm_channel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pwm_duty_cycle channel(::pwm_driver_interface::msg::Pwm::_channel_type arg)
  {
    msg_.channel = std::move(arg);
    return Init_Pwm_duty_cycle(msg_);
  }

private:
  ::pwm_driver_interface::msg::Pwm msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pwm_driver_interface::msg::Pwm>()
{
  return pwm_driver_interface::msg::builder::Init_Pwm_channel();
}

}  // namespace pwm_driver_interface

#endif  // PWM_DRIVER_INTERFACE__MSG__DETAIL__PWM__BUILDER_HPP_
