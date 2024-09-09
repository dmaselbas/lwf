// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pwm_driver_interface:srv/SetPwmDutyCycle.idl
// generated code does not contain a copyright notice

#ifndef PWM_DRIVER_INTERFACE__SRV__DETAIL__SET_PWM_DUTY_CYCLE__STRUCT_H_
#define PWM_DRIVER_INTERFACE__SRV__DETAIL__SET_PWM_DUTY_CYCLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetPwmDutyCycle in the package pwm_driver_interface.
typedef struct pwm_driver_interface__srv__SetPwmDutyCycle_Request
{
  int8_t channel;
  int32_t duty_cycle;
} pwm_driver_interface__srv__SetPwmDutyCycle_Request;

// Struct for a sequence of pwm_driver_interface__srv__SetPwmDutyCycle_Request.
typedef struct pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence
{
  pwm_driver_interface__srv__SetPwmDutyCycle_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetPwmDutyCycle in the package pwm_driver_interface.
typedef struct pwm_driver_interface__srv__SetPwmDutyCycle_Response
{
  int8_t channel;
  int32_t duty_cycle;
} pwm_driver_interface__srv__SetPwmDutyCycle_Response;

// Struct for a sequence of pwm_driver_interface__srv__SetPwmDutyCycle_Response.
typedef struct pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence
{
  pwm_driver_interface__srv__SetPwmDutyCycle_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PWM_DRIVER_INTERFACE__SRV__DETAIL__SET_PWM_DUTY_CYCLE__STRUCT_H_
