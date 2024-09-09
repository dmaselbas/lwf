// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pwm_driver_interface:msg/Pwm.idl
// generated code does not contain a copyright notice

#ifndef PWM_DRIVER_INTERFACE__MSG__DETAIL__PWM__STRUCT_HPP_
#define PWM_DRIVER_INTERFACE__MSG__DETAIL__PWM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pwm_driver_interface__msg__Pwm __attribute__((deprecated))
#else
# define DEPRECATED__pwm_driver_interface__msg__Pwm __declspec(deprecated)
#endif

namespace pwm_driver_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Pwm_
{
  using Type = Pwm_<ContainerAllocator>;

  explicit Pwm_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->channel = 0;
      this->duty_cycle = 0l;
    }
  }

  explicit Pwm_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->channel = 0;
      this->duty_cycle = 0l;
    }
  }

  // field types and members
  using _channel_type =
    int8_t;
  _channel_type channel;
  using _duty_cycle_type =
    int32_t;
  _duty_cycle_type duty_cycle;

  // setters for named parameter idiom
  Type & set__channel(
    const int8_t & _arg)
  {
    this->channel = _arg;
    return *this;
  }
  Type & set__duty_cycle(
    const int32_t & _arg)
  {
    this->duty_cycle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pwm_driver_interface::msg::Pwm_<ContainerAllocator> *;
  using ConstRawPtr =
    const pwm_driver_interface::msg::Pwm_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pwm_driver_interface::msg::Pwm_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pwm_driver_interface::msg::Pwm_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pwm_driver_interface::msg::Pwm_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pwm_driver_interface::msg::Pwm_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pwm_driver_interface::msg::Pwm_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pwm_driver_interface::msg::Pwm_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pwm_driver_interface::msg::Pwm_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pwm_driver_interface::msg::Pwm_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pwm_driver_interface__msg__Pwm
    std::shared_ptr<pwm_driver_interface::msg::Pwm_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pwm_driver_interface__msg__Pwm
    std::shared_ptr<pwm_driver_interface::msg::Pwm_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pwm_ & other) const
  {
    if (this->channel != other.channel) {
      return false;
    }
    if (this->duty_cycle != other.duty_cycle) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pwm_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pwm_

// alias to use template instance with default allocator
using Pwm =
  pwm_driver_interface::msg::Pwm_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace pwm_driver_interface

#endif  // PWM_DRIVER_INTERFACE__MSG__DETAIL__PWM__STRUCT_HPP_
