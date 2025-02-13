// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from robp_interfaces:msg/DutyCycles.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "robp_interfaces/msg/detail/duty_cycles__functions.h"
#include "robp_interfaces/msg/detail/duty_cycles__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace robp_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void DutyCycles_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) robp_interfaces::msg::DutyCycles(_init);
}

void DutyCycles_fini_function(void * message_memory)
{
  auto typed_message = static_cast<robp_interfaces::msg::DutyCycles *>(message_memory);
  typed_message->~DutyCycles();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DutyCycles_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robp_interfaces::msg::DutyCycles, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "duty_cycle_left",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robp_interfaces::msg::DutyCycles, duty_cycle_left),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "duty_cycle_right",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robp_interfaces::msg::DutyCycles, duty_cycle_right),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DutyCycles_message_members = {
  "robp_interfaces::msg",  // message namespace
  "DutyCycles",  // message name
  3,  // number of fields
  sizeof(robp_interfaces::msg::DutyCycles),
  false,  // has_any_key_member_
  DutyCycles_message_member_array,  // message members
  DutyCycles_init_function,  // function to initialize message memory (memory has to be allocated)
  DutyCycles_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DutyCycles_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DutyCycles_message_members,
  get_message_typesupport_handle_function,
  &robp_interfaces__msg__DutyCycles__get_type_hash,
  &robp_interfaces__msg__DutyCycles__get_type_description,
  &robp_interfaces__msg__DutyCycles__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace robp_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robp_interfaces::msg::DutyCycles>()
{
  return &::robp_interfaces::msg::rosidl_typesupport_introspection_cpp::DutyCycles_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robp_interfaces, msg, DutyCycles)() {
  return &::robp_interfaces::msg::rosidl_typesupport_introspection_cpp::DutyCycles_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
