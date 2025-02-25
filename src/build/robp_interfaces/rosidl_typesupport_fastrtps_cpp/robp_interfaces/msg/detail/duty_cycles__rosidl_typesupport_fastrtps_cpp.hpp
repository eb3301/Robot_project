// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from robp_interfaces:msg/DutyCycles.idl
// generated code does not contain a copyright notice

#ifndef ROBP_INTERFACES__MSG__DETAIL__DUTY_CYCLES__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ROBP_INTERFACES__MSG__DETAIL__DUTY_CYCLES__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "robp_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "robp_interfaces/msg/detail/duty_cycles__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace robp_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robp_interfaces
cdr_serialize(
  const robp_interfaces::msg::DutyCycles & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robp_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  robp_interfaces::msg::DutyCycles & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robp_interfaces
get_serialized_size(
  const robp_interfaces::msg::DutyCycles & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robp_interfaces
max_serialized_size_DutyCycles(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robp_interfaces
cdr_serialize_key(
  const robp_interfaces::msg::DutyCycles & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robp_interfaces
get_serialized_size_key(
  const robp_interfaces::msg::DutyCycles & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robp_interfaces
max_serialized_size_key_DutyCycles(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace robp_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robp_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robp_interfaces, msg, DutyCycles)();

#ifdef __cplusplus
}
#endif

#endif  // ROBP_INTERFACES__MSG__DETAIL__DUTY_CYCLES__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
