// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robp_interfaces:msg/DutyCycles.idl
// generated code does not contain a copyright notice
#include "robp_interfaces/msg/detail/duty_cycles__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
robp_interfaces__msg__DutyCycles__init(robp_interfaces__msg__DutyCycles * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    robp_interfaces__msg__DutyCycles__fini(msg);
    return false;
  }
  // duty_cycle_left
  // duty_cycle_right
  return true;
}

void
robp_interfaces__msg__DutyCycles__fini(robp_interfaces__msg__DutyCycles * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // duty_cycle_left
  // duty_cycle_right
}

bool
robp_interfaces__msg__DutyCycles__are_equal(const robp_interfaces__msg__DutyCycles * lhs, const robp_interfaces__msg__DutyCycles * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // duty_cycle_left
  if (lhs->duty_cycle_left != rhs->duty_cycle_left) {
    return false;
  }
  // duty_cycle_right
  if (lhs->duty_cycle_right != rhs->duty_cycle_right) {
    return false;
  }
  return true;
}

bool
robp_interfaces__msg__DutyCycles__copy(
  const robp_interfaces__msg__DutyCycles * input,
  robp_interfaces__msg__DutyCycles * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // duty_cycle_left
  output->duty_cycle_left = input->duty_cycle_left;
  // duty_cycle_right
  output->duty_cycle_right = input->duty_cycle_right;
  return true;
}

robp_interfaces__msg__DutyCycles *
robp_interfaces__msg__DutyCycles__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robp_interfaces__msg__DutyCycles * msg = (robp_interfaces__msg__DutyCycles *)allocator.allocate(sizeof(robp_interfaces__msg__DutyCycles), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robp_interfaces__msg__DutyCycles));
  bool success = robp_interfaces__msg__DutyCycles__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robp_interfaces__msg__DutyCycles__destroy(robp_interfaces__msg__DutyCycles * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robp_interfaces__msg__DutyCycles__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robp_interfaces__msg__DutyCycles__Sequence__init(robp_interfaces__msg__DutyCycles__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robp_interfaces__msg__DutyCycles * data = NULL;

  if (size) {
    data = (robp_interfaces__msg__DutyCycles *)allocator.zero_allocate(size, sizeof(robp_interfaces__msg__DutyCycles), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robp_interfaces__msg__DutyCycles__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robp_interfaces__msg__DutyCycles__fini(&data[i - 1]);
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
robp_interfaces__msg__DutyCycles__Sequence__fini(robp_interfaces__msg__DutyCycles__Sequence * array)
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
      robp_interfaces__msg__DutyCycles__fini(&array->data[i]);
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

robp_interfaces__msg__DutyCycles__Sequence *
robp_interfaces__msg__DutyCycles__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robp_interfaces__msg__DutyCycles__Sequence * array = (robp_interfaces__msg__DutyCycles__Sequence *)allocator.allocate(sizeof(robp_interfaces__msg__DutyCycles__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robp_interfaces__msg__DutyCycles__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robp_interfaces__msg__DutyCycles__Sequence__destroy(robp_interfaces__msg__DutyCycles__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robp_interfaces__msg__DutyCycles__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robp_interfaces__msg__DutyCycles__Sequence__are_equal(const robp_interfaces__msg__DutyCycles__Sequence * lhs, const robp_interfaces__msg__DutyCycles__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robp_interfaces__msg__DutyCycles__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robp_interfaces__msg__DutyCycles__Sequence__copy(
  const robp_interfaces__msg__DutyCycles__Sequence * input,
  robp_interfaces__msg__DutyCycles__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robp_interfaces__msg__DutyCycles);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robp_interfaces__msg__DutyCycles * data =
      (robp_interfaces__msg__DutyCycles *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robp_interfaces__msg__DutyCycles__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robp_interfaces__msg__DutyCycles__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robp_interfaces__msg__DutyCycles__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
