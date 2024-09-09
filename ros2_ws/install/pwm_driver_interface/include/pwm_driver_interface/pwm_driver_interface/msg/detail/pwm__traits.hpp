// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pwm_driver_interface:msg/Pwm.idl
// generated code does not contain a copyright notice

#ifndef PWM_DRIVER_INTERFACE__MSG__DETAIL__PWM__TRAITS_HPP_
#define PWM_DRIVER_INTERFACE__MSG__DETAIL__PWM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pwm_driver_interface/msg/detail/pwm__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pwm_driver_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const Pwm & msg,
  std::ostream & out)
{
  out << "{";
  // member: channel
  {
    out << "channel: ";
    rosidl_generator_traits::value_to_yaml(msg.channel, out);
    out << ", ";
  }

  // member: duty_cycle
  {
    out << "duty_cycle: ";
    rosidl_generator_traits::value_to_yaml(msg.duty_cycle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Pwm & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: channel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel: ";
    rosidl_generator_traits::value_to_yaml(msg.channel, out);
    out << "\n";
  }

  // member: duty_cycle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duty_cycle: ";
    rosidl_generator_traits::value_to_yaml(msg.duty_cycle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Pwm & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace pwm_driver_interface

namespace rosidl_generator_traits
{

[[deprecated("use pwm_driver_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pwm_driver_interface::msg::Pwm & msg,
  std::ostream & out, size_t indentation = 0)
{
  pwm_driver_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pwm_driver_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const pwm_driver_interface::msg::Pwm & msg)
{
  return pwm_driver_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<pwm_driver_interface::msg::Pwm>()
{
  return "pwm_driver_interface::msg::Pwm";
}

template<>
inline const char * name<pwm_driver_interface::msg::Pwm>()
{
  return "pwm_driver_interface/msg/Pwm";
}

template<>
struct has_fixed_size<pwm_driver_interface::msg::Pwm>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pwm_driver_interface::msg::Pwm>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pwm_driver_interface::msg::Pwm>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PWM_DRIVER_INTERFACE__MSG__DETAIL__PWM__TRAITS_HPP_
