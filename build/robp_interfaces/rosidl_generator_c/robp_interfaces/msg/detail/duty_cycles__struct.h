// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robp_interfaces:msg/DutyCycles.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/msg/duty_cycles.h"


#ifndef ROBP_INTERFACES__MSG__DETAIL__DUTY_CYCLES__STRUCT_H_
#define ROBP_INTERFACES__MSG__DETAIL__DUTY_CYCLES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/DutyCycles in the package robp_interfaces.
typedef struct robp_interfaces__msg__DutyCycles
{
  std_msgs__msg__Header header;
  /// Value should be in [-1, 1], negative is backwards, positive forwards
  double duty_cycle_left;
  double duty_cycle_right;
} robp_interfaces__msg__DutyCycles;

// Struct for a sequence of robp_interfaces__msg__DutyCycles.
typedef struct robp_interfaces__msg__DutyCycles__Sequence
{
  robp_interfaces__msg__DutyCycles * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robp_interfaces__msg__DutyCycles__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBP_INTERFACES__MSG__DETAIL__DUTY_CYCLES__STRUCT_H_
