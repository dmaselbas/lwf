// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pwm_driver_interface:srv/SetPwmDutyCycle.idl
// generated code does not contain a copyright notice
#include "pwm_driver_interface/srv/detail/set_pwm_duty_cycle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
pwm_driver_interface__srv__SetPwmDutyCycle_Request__init(pwm_driver_interface__srv__SetPwmDutyCycle_Request * msg)
{
  if (!msg) {
    return false;
  }
  // channel
  // duty_cycle
  return true;
}

void
pwm_driver_interface__srv__SetPwmDutyCycle_Request__fini(pwm_driver_interface__srv__SetPwmDutyCycle_Request * msg)
{
  if (!msg) {
    return;
  }
  // channel
  // duty_cycle
}

bool
pwm_driver_interface__srv__SetPwmDutyCycle_Request__are_equal(const pwm_driver_interface__srv__SetPwmDutyCycle_Request * lhs, const pwm_driver_interface__srv__SetPwmDutyCycle_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // channel
  if (lhs->channel != rhs->channel) {
    return false;
  }
  // duty_cycle
  if (lhs->duty_cycle != rhs->duty_cycle) {
    return false;
  }
  return true;
}

bool
pwm_driver_interface__srv__SetPwmDutyCycle_Request__copy(
  const pwm_driver_interface__srv__SetPwmDutyCycle_Request * input,
  pwm_driver_interface__srv__SetPwmDutyCycle_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // channel
  output->channel = input->channel;
  // duty_cycle
  output->duty_cycle = input->duty_cycle;
  return true;
}

pwm_driver_interface__srv__SetPwmDutyCycle_Request *
pwm_driver_interface__srv__SetPwmDutyCycle_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pwm_driver_interface__srv__SetPwmDutyCycle_Request * msg = (pwm_driver_interface__srv__SetPwmDutyCycle_Request *)allocator.allocate(sizeof(pwm_driver_interface__srv__SetPwmDutyCycle_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pwm_driver_interface__srv__SetPwmDutyCycle_Request));
  bool success = pwm_driver_interface__srv__SetPwmDutyCycle_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pwm_driver_interface__srv__SetPwmDutyCycle_Request__destroy(pwm_driver_interface__srv__SetPwmDutyCycle_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pwm_driver_interface__srv__SetPwmDutyCycle_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence__init(pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pwm_driver_interface__srv__SetPwmDutyCycle_Request * data = NULL;

  if (size) {
    data = (pwm_driver_interface__srv__SetPwmDutyCycle_Request *)allocator.zero_allocate(size, sizeof(pwm_driver_interface__srv__SetPwmDutyCycle_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pwm_driver_interface__srv__SetPwmDutyCycle_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pwm_driver_interface__srv__SetPwmDutyCycle_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence__fini(pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pwm_driver_interface__srv__SetPwmDutyCycle_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence *
pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence * array = (pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence *)allocator.allocate(sizeof(pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence__destroy(pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence__are_equal(const pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence * lhs, const pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pwm_driver_interface__srv__SetPwmDutyCycle_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence__copy(
  const pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence * input,
  pwm_driver_interface__srv__SetPwmDutyCycle_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pwm_driver_interface__srv__SetPwmDutyCycle_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pwm_driver_interface__srv__SetPwmDutyCycle_Request * data =
      (pwm_driver_interface__srv__SetPwmDutyCycle_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pwm_driver_interface__srv__SetPwmDutyCycle_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pwm_driver_interface__srv__SetPwmDutyCycle_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pwm_driver_interface__srv__SetPwmDutyCycle_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
pwm_driver_interface__srv__SetPwmDutyCycle_Response__init(pwm_driver_interface__srv__SetPwmDutyCycle_Response * msg)
{
  if (!msg) {
    return false;
  }
  // channel
  // duty_cycle
  return true;
}

void
pwm_driver_interface__srv__SetPwmDutyCycle_Response__fini(pwm_driver_interface__srv__SetPwmDutyCycle_Response * msg)
{
  if (!msg) {
    return;
  }
  // channel
  // duty_cycle
}

bool
pwm_driver_interface__srv__SetPwmDutyCycle_Response__are_equal(const pwm_driver_interface__srv__SetPwmDutyCycle_Response * lhs, const pwm_driver_interface__srv__SetPwmDutyCycle_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // channel
  if (lhs->channel != rhs->channel) {
    return false;
  }
  // duty_cycle
  if (lhs->duty_cycle != rhs->duty_cycle) {
    return false;
  }
  return true;
}

bool
pwm_driver_interface__srv__SetPwmDutyCycle_Response__copy(
  const pwm_driver_interface__srv__SetPwmDutyCycle_Response * input,
  pwm_driver_interface__srv__SetPwmDutyCycle_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // channel
  output->channel = input->channel;
  // duty_cycle
  output->duty_cycle = input->duty_cycle;
  return true;
}

pwm_driver_interface__srv__SetPwmDutyCycle_Response *
pwm_driver_interface__srv__SetPwmDutyCycle_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pwm_driver_interface__srv__SetPwmDutyCycle_Response * msg = (pwm_driver_interface__srv__SetPwmDutyCycle_Response *)allocator.allocate(sizeof(pwm_driver_interface__srv__SetPwmDutyCycle_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pwm_driver_interface__srv__SetPwmDutyCycle_Response));
  bool success = pwm_driver_interface__srv__SetPwmDutyCycle_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pwm_driver_interface__srv__SetPwmDutyCycle_Response__destroy(pwm_driver_interface__srv__SetPwmDutyCycle_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pwm_driver_interface__srv__SetPwmDutyCycle_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence__init(pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pwm_driver_interface__srv__SetPwmDutyCycle_Response * data = NULL;

  if (size) {
    data = (pwm_driver_interface__srv__SetPwmDutyCycle_Response *)allocator.zero_allocate(size, sizeof(pwm_driver_interface__srv__SetPwmDutyCycle_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pwm_driver_interface__srv__SetPwmDutyCycle_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pwm_driver_interface__srv__SetPwmDutyCycle_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence__fini(pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pwm_driver_interface__srv__SetPwmDutyCycle_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence *
pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence * array = (pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence *)allocator.allocate(sizeof(pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence__destroy(pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence__are_equal(const pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence * lhs, const pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pwm_driver_interface__srv__SetPwmDutyCycle_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence__copy(
  const pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence * input,
  pwm_driver_interface__srv__SetPwmDutyCycle_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pwm_driver_interface__srv__SetPwmDutyCycle_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pwm_driver_interface__srv__SetPwmDutyCycle_Response * data =
      (pwm_driver_interface__srv__SetPwmDutyCycle_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pwm_driver_interface__srv__SetPwmDutyCycle_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pwm_driver_interface__srv__SetPwmDutyCycle_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pwm_driver_interface__srv__SetPwmDutyCycle_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
