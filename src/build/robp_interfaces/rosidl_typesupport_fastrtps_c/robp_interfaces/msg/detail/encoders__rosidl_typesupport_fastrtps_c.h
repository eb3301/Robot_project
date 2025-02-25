// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from robp_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice
#ifndef ROBP_INTERFACES__MSG__DETAIL__ENCODERS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define ROBP_INTERFACES__MSG__DETAIL__ENCODERS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "robp_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robp_interfaces/msg/detail/encoders__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robp_interfaces
bool cdr_serialize_robp_interfaces__msg__Encoders(
  const robp_interfaces__msg__Encoders * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robp_interfaces
bool cdr_deserialize_robp_interfaces__msg__Encoders(
  eprosima::fastcdr::Cdr &,
  robp_interfaces__msg__Encoders * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robp_interfaces
size_t get_serialized_size_robp_interfaces__msg__Encoders(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robp_interfaces
size_t max_serialized_size_robp_interfaces__msg__Encoders(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robp_interfaces
bool cdr_serialize_key_robp_interfaces__msg__Encoders(
  const robp_interfaces__msg__Encoders * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robp_interfaces
size_t get_serialized_size_key_robp_interfaces__msg__Encoders(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robp_interfaces
size_t max_serialized_size_key_robp_interfaces__msg__Encoders(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robp_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robp_interfaces, msg, Encoders)();

#ifdef __cplusplus
}
#endif

#endif  // ROBP_INTERFACES__MSG__DETAIL__ENCODERS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
