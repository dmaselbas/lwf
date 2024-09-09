// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pwm_driver_interface:srv/SetPwmDutyCycle.idl
// generated code does not contain a copyright notice

#ifndef PWM_DRIVER_INTERFACE__SRV__DETAIL__SET_PWM_DUTY_CYCLE__TRAITS_HPP_
#define PWM_DRIVER_INTERFACE__SRV__DETAIL__SET_PWM_DUTY_CYCLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pwm_driver_interface/srv/detail/set_pwm_duty_cycle__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pwm_driver_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetPwmDutyCycle_Request & msg,
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
  const SetPwmDutyCycle_Request & msg,
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

inline std::string to_yaml(const SetPwmDutyCycle_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace pwm_driver_interface

namespace rosidl_generator_traits
{

[[deprecated("use pwm_driver_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pwm_driver_interface::srv::SetPwmDutyCycle_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  pwm_driver_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pwm_driver_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const pwm_driver_interface::srv::SetPwmDutyCycle_Request & msg)
{
  return pwm_driver_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pwm_driver_interface::srv::SetPwmDutyCycle_Request>()
{
  return "pwm_driver_interface::srv::SetPwmDutyCycle_Request";
}

template<>
inline const char * name<pwm_driver_interface::srv::SetPwmDutyCycle_Request>()
{
  return "pwm_driver_interface/srv/SetPwmDutyCycle_Request";
}

template<>
struct has_fixed_size<pwm_driver_interface::srv::SetPwmDutyCycle_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pwm_driver_interface::srv::SetPwmDutyCycle_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pwm_driver_interface::srv::SetPwmDutyCycle_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace pwm_driver_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetPwmDutyCycle_Response & msg,
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
  const SetPwmDutyCycle_Response & msg,
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

inline std::string to_yaml(const SetPwmDutyCycle_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace pwm_driver_interface

namespace rosidl_generator_traits
{

[[deprecated("use pwm_driver_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pwm_driver_interface::srv::SetPwmDutyCycle_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  pwm_driver_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pwm_driver_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const pwm_driver_interface::srv::SetPwmDutyCycle_Response & msg)
{
  return pwm_driver_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pwm_driver_interface::srv::SetPwmDutyCycle_Response>()
{
  return "pwm_driver_interface::srv::SetPwmDutyCycle_Response";
}

template<>
inline const char * name<pwm_driver_interface::srv::SetPwmDutyCycle_Response>()
{
  return "pwm_driver_interface/srv/SetPwmDutyCycle_Response";
}

template<>
struct has_fixed_size<pwm_driver_interface::srv::SetPwmDutyCycle_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pwm_driver_interface::srv::SetPwmDutyCycle_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pwm_driver_interface::srv::SetPwmDutyCycle_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pwm_driver_interface::srv::SetPwmDutyCycle>()
{
  return "pwm_driver_interface::srv::SetPwmDutyCycle";
}

template<>
inline const char * name<pwm_driver_interface::srv::SetPwmDutyCycle>()
{
  return "pwm_driver_interface/srv/SetPwmDutyCycle";
}

template<>
struct has_fixed_size<pwm_driver_interface::srv::SetPwmDutyCycle>
  : std::integral_constant<
    bool,
    has_fixed_size<pwm_driver_interface::srv::SetPwmDutyCycle_Request>::value &&
    has_fixed_size<pwm_driver_interface::srv::SetPwmDutyCycle_Response>::value
  >
{
};

template<>
struct has_bounded_size<pwm_driver_interface::srv::SetPwmDutyCycle>
  : std::integral_constant<
    bool,
    has_bounded_size<pwm_driver_interface::srv::SetPwmDutyCycle_Request>::value &&
    has_bounded_size<pwm_driver_interface::srv::SetPwmDutyCycle_Response>::value
  >
{
};

template<>
struct is_service<pwm_driver_interface::srv::SetPwmDutyCycle>
  : std::true_type
{
};

template<>
struct is_service_request<pwm_driver_interface::srv::SetPwmDutyCycle_Request>
  : std::true_type
{
};

template<>
struct is_service_response<pwm_driver_interface::srv::SetPwmDutyCycle_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PWM_DRIVER_INTERFACE__SRV__DETAIL__SET_PWM_DUTY_CYCLE__TRAITS_HPP_
