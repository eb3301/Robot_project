// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robp_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/msg/encoders.h"


#ifndef ROBP_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_H_
#define ROBP_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_H_

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

/// Struct defined in msg/Encoders in the package robp_interfaces.
typedef struct robp_interfaces__msg__Encoders
{
  std_msgs__msg__Header header;
  /// Total number of ticks
  int64_t encoder_left;
  int64_t encoder_right;
  /// The number of ticks since the last reading
  int32_t delta_encoder_left;
  int32_t delta_encoder_right;
} robp_interfaces__msg__Encoders;

// Struct for a sequence of robp_interfaces__msg__Encoders.
typedef struct robp_interfaces__msg__Encoders__Sequence
{
  robp_interfaces__msg__Encoders * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robp_interfaces__msg__Encoders__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBP_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_H_
