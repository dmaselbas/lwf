// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pwm_driver_interface:msg/Pwm.idl
// generated code does not contain a copyright notice

#ifndef PWM_DRIVER_INTERFACE__MSG__DETAIL__PWM__STRUCT_H_
#define PWM_DRIVER_INTERFACE__MSG__DETAIL__PWM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Pwm in the package pwm_driver_interface.
typedef struct pwm_driver_interface__msg__Pwm
{
  int8_t channel;
  int32_t duty_cycle;
} pwm_driver_interface__msg__Pwm;

// Struct for a sequence of pwm_driver_interface__msg__Pwm.
typedef struct pwm_driver_interface__msg__Pwm__Sequence
{
  pwm_driver_interface__msg__Pwm * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pwm_driver_interface__msg__Pwm__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PWM_DRIVER_INTERFACE__MSG__DETAIL__PWM__STRUCT_H_
