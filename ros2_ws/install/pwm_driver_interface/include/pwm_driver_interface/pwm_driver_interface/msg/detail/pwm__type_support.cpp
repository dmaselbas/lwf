// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from pwm_driver_interface:msg/Pwm.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "pwm_driver_interface/msg/detail/pwm__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace pwm_driver_interface
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Pwm_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) pwm_driver_interface::msg::Pwm(_init);
}

void Pwm_fini_function(void * message_memory)
{
  auto typed_message = static_cast<pwm_driver_interface::msg::Pwm *>(message_memory);
  typed_message->~Pwm();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Pwm_message_member_array[2] = {
  {
    "channel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pwm_driver_interface::msg::Pwm, channel),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "duty_cycle",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pwm_driver_interface::msg::Pwm, duty_cycle),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Pwm_message_members = {
  "pwm_driver_interface::msg",  // message namespace
  "Pwm",  // message name
  2,  // number of fields
  sizeof(pwm_driver_interface::msg::Pwm),
  Pwm_message_member_array,  // message members
  Pwm_init_function,  // function to initialize message memory (memory has to be allocated)
  Pwm_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Pwm_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Pwm_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace pwm_driver_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pwm_driver_interface::msg::Pwm>()
{
  return &::pwm_driver_interface::msg::rosidl_typesupport_introspection_cpp::Pwm_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pwm_driver_interface, msg, Pwm)() {
  return &::pwm_driver_interface::msg::rosidl_typesupport_introspection_cpp::Pwm_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
