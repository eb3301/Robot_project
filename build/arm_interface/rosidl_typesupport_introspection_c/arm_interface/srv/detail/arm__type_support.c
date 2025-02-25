// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from arm_interface:srv/Arm.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "arm_interface/srv/detail/arm__rosidl_typesupport_introspection_c.h"
#include "arm_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "arm_interface/srv/detail/arm__functions.h"
#include "arm_interface/srv/detail/arm__struct.h"


// Include directives for member types
// Member `obj_class`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__Arm_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arm_interface__srv__Arm_Request__init(message_memory);
}

void arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__Arm_Request_fini_function(void * message_memory)
{
  arm_interface__srv__Arm_Request__fini(message_memory);
}

size_t arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__size_function__Arm_Request__xy(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__get_const_function__Arm_Request__xy(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__get_function__Arm_Request__xy(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__fetch_function__Arm_Request__xy(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__get_const_function__Arm_Request__xy(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__assign_function__Arm_Request__xy(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__get_function__Arm_Request__xy(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__Arm_Request_message_member_array[2] = {
  {
    "xy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(arm_interface__srv__Arm_Request, xy),  // bytes offset in struct
    NULL,  // default value
    arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__size_function__Arm_Request__xy,  // size() function pointer
    arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__get_const_function__Arm_Request__xy,  // get_const(index) function pointer
    arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__get_function__Arm_Request__xy,  // get(index) function pointer
    arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__fetch_function__Arm_Request__xy,  // fetch(index, &value) function pointer
    arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__assign_function__Arm_Request__xy,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "obj_class",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_interface__srv__Arm_Request, obj_class),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__Arm_Request_message_members = {
  "arm_interface__srv",  // message namespace
  "Arm_Request",  // message name
  2,  // number of fields
  sizeof(arm_interface__srv__Arm_Request),
  false,  // has_any_key_member_
  arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__Arm_Request_message_member_array,  // message members
  arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__Arm_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__Arm_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__Arm_Request_message_type_support_handle = {
  0,
  &arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__Arm_Request_message_members,
  get_message_typesupport_handle_function,
  &arm_interface__srv__Arm_Request__get_type_hash,
  &arm_interface__srv__Arm_Request__get_type_description,
  &arm_interface__srv__Arm_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arm_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_interface, srv, Arm_Request)() {
  if (!arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__Arm_Request_message_type_support_handle.typesupport_identifier) {
    arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__Arm_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__Arm_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "arm_interface/srv/detail/arm__rosidl_typesupport_introspection_c.h"
// already included above
// #include "arm_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "arm_interface/srv/detail/arm__functions.h"
// already included above
// #include "arm_interface/srv/detail/arm__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void arm_interface__srv__Arm_Response__rosidl_typesupport_introspection_c__Arm_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arm_interface__srv__Arm_Response__init(message_memory);
}

void arm_interface__srv__Arm_Response__rosidl_typesupport_introspection_c__Arm_Response_fini_function(void * message_memory)
{
  arm_interface__srv__Arm_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember arm_interface__srv__Arm_Response__rosidl_typesupport_introspection_c__Arm_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_interface__srv__Arm_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arm_interface__srv__Arm_Response__rosidl_typesupport_introspection_c__Arm_Response_message_members = {
  "arm_interface__srv",  // message namespace
  "Arm_Response",  // message name
  1,  // number of fields
  sizeof(arm_interface__srv__Arm_Response),
  false,  // has_any_key_member_
  arm_interface__srv__Arm_Response__rosidl_typesupport_introspection_c__Arm_Response_message_member_array,  // message members
  arm_interface__srv__Arm_Response__rosidl_typesupport_introspection_c__Arm_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  arm_interface__srv__Arm_Response__rosidl_typesupport_introspection_c__Arm_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arm_interface__srv__Arm_Response__rosidl_typesupport_introspection_c__Arm_Response_message_type_support_handle = {
  0,
  &arm_interface__srv__Arm_Response__rosidl_typesupport_introspection_c__Arm_Response_message_members,
  get_message_typesupport_handle_function,
  &arm_interface__srv__Arm_Response__get_type_hash,
  &arm_interface__srv__Arm_Response__get_type_description,
  &arm_interface__srv__Arm_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arm_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_interface, srv, Arm_Response)() {
  if (!arm_interface__srv__Arm_Response__rosidl_typesupport_introspection_c__Arm_Response_message_type_support_handle.typesupport_identifier) {
    arm_interface__srv__Arm_Response__rosidl_typesupport_introspection_c__Arm_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arm_interface__srv__Arm_Response__rosidl_typesupport_introspection_c__Arm_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "arm_interface/srv/detail/arm__rosidl_typesupport_introspection_c.h"
// already included above
// #include "arm_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "arm_interface/srv/detail/arm__functions.h"
// already included above
// #include "arm_interface/srv/detail/arm__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "arm_interface/srv/arm.h"
// Member `request`
// Member `response`
// already included above
// #include "arm_interface/srv/detail/arm__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arm_interface__srv__Arm_Event__init(message_memory);
}

void arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_fini_function(void * message_memory)
{
  arm_interface__srv__Arm_Event__fini(message_memory);
}

size_t arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__size_function__Arm_Event__request(
  const void * untyped_member)
{
  const arm_interface__srv__Arm_Request__Sequence * member =
    (const arm_interface__srv__Arm_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__get_const_function__Arm_Event__request(
  const void * untyped_member, size_t index)
{
  const arm_interface__srv__Arm_Request__Sequence * member =
    (const arm_interface__srv__Arm_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__get_function__Arm_Event__request(
  void * untyped_member, size_t index)
{
  arm_interface__srv__Arm_Request__Sequence * member =
    (arm_interface__srv__Arm_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__fetch_function__Arm_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const arm_interface__srv__Arm_Request * item =
    ((const arm_interface__srv__Arm_Request *)
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__get_const_function__Arm_Event__request(untyped_member, index));
  arm_interface__srv__Arm_Request * value =
    (arm_interface__srv__Arm_Request *)(untyped_value);
  *value = *item;
}

void arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__assign_function__Arm_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  arm_interface__srv__Arm_Request * item =
    ((arm_interface__srv__Arm_Request *)
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__get_function__Arm_Event__request(untyped_member, index));
  const arm_interface__srv__Arm_Request * value =
    (const arm_interface__srv__Arm_Request *)(untyped_value);
  *item = *value;
}

bool arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__resize_function__Arm_Event__request(
  void * untyped_member, size_t size)
{
  arm_interface__srv__Arm_Request__Sequence * member =
    (arm_interface__srv__Arm_Request__Sequence *)(untyped_member);
  arm_interface__srv__Arm_Request__Sequence__fini(member);
  return arm_interface__srv__Arm_Request__Sequence__init(member, size);
}

size_t arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__size_function__Arm_Event__response(
  const void * untyped_member)
{
  const arm_interface__srv__Arm_Response__Sequence * member =
    (const arm_interface__srv__Arm_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__get_const_function__Arm_Event__response(
  const void * untyped_member, size_t index)
{
  const arm_interface__srv__Arm_Response__Sequence * member =
    (const arm_interface__srv__Arm_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__get_function__Arm_Event__response(
  void * untyped_member, size_t index)
{
  arm_interface__srv__Arm_Response__Sequence * member =
    (arm_interface__srv__Arm_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__fetch_function__Arm_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const arm_interface__srv__Arm_Response * item =
    ((const arm_interface__srv__Arm_Response *)
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__get_const_function__Arm_Event__response(untyped_member, index));
  arm_interface__srv__Arm_Response * value =
    (arm_interface__srv__Arm_Response *)(untyped_value);
  *value = *item;
}

void arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__assign_function__Arm_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  arm_interface__srv__Arm_Response * item =
    ((arm_interface__srv__Arm_Response *)
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__get_function__Arm_Event__response(untyped_member, index));
  const arm_interface__srv__Arm_Response * value =
    (const arm_interface__srv__Arm_Response *)(untyped_value);
  *item = *value;
}

bool arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__resize_function__Arm_Event__response(
  void * untyped_member, size_t size)
{
  arm_interface__srv__Arm_Response__Sequence * member =
    (arm_interface__srv__Arm_Response__Sequence *)(untyped_member);
  arm_interface__srv__Arm_Response__Sequence__fini(member);
  return arm_interface__srv__Arm_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_interface__srv__Arm_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(arm_interface__srv__Arm_Event, request),  // bytes offset in struct
    NULL,  // default value
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__size_function__Arm_Event__request,  // size() function pointer
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__get_const_function__Arm_Event__request,  // get_const(index) function pointer
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__get_function__Arm_Event__request,  // get(index) function pointer
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__fetch_function__Arm_Event__request,  // fetch(index, &value) function pointer
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__assign_function__Arm_Event__request,  // assign(index, value) function pointer
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__resize_function__Arm_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(arm_interface__srv__Arm_Event, response),  // bytes offset in struct
    NULL,  // default value
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__size_function__Arm_Event__response,  // size() function pointer
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__get_const_function__Arm_Event__response,  // get_const(index) function pointer
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__get_function__Arm_Event__response,  // get(index) function pointer
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__fetch_function__Arm_Event__response,  // fetch(index, &value) function pointer
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__assign_function__Arm_Event__response,  // assign(index, value) function pointer
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__resize_function__Arm_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_message_members = {
  "arm_interface__srv",  // message namespace
  "Arm_Event",  // message name
  3,  // number of fields
  sizeof(arm_interface__srv__Arm_Event),
  false,  // has_any_key_member_
  arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_message_member_array,  // message members
  arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_message_type_support_handle = {
  0,
  &arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_message_members,
  get_message_typesupport_handle_function,
  &arm_interface__srv__Arm_Event__get_type_hash,
  &arm_interface__srv__Arm_Event__get_type_description,
  &arm_interface__srv__Arm_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arm_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_interface, srv, Arm_Event)() {
  arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_interface, srv, Arm_Request)();
  arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_interface, srv, Arm_Response)();
  if (!arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_message_type_support_handle.typesupport_identifier) {
    arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "arm_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "arm_interface/srv/detail/arm__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers arm_interface__srv__detail__arm__rosidl_typesupport_introspection_c__Arm_service_members = {
  "arm_interface__srv",  // service namespace
  "Arm",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // arm_interface__srv__detail__arm__rosidl_typesupport_introspection_c__Arm_Request_message_type_support_handle,
  NULL,  // response message
  // arm_interface__srv__detail__arm__rosidl_typesupport_introspection_c__Arm_Response_message_type_support_handle
  NULL  // event_message
  // arm_interface__srv__detail__arm__rosidl_typesupport_introspection_c__Arm_Response_message_type_support_handle
};


static rosidl_service_type_support_t arm_interface__srv__detail__arm__rosidl_typesupport_introspection_c__Arm_service_type_support_handle = {
  0,
  &arm_interface__srv__detail__arm__rosidl_typesupport_introspection_c__Arm_service_members,
  get_service_typesupport_handle_function,
  &arm_interface__srv__Arm_Request__rosidl_typesupport_introspection_c__Arm_Request_message_type_support_handle,
  &arm_interface__srv__Arm_Response__rosidl_typesupport_introspection_c__Arm_Response_message_type_support_handle,
  &arm_interface__srv__Arm_Event__rosidl_typesupport_introspection_c__Arm_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    arm_interface,
    srv,
    Arm
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    arm_interface,
    srv,
    Arm
  ),
  &arm_interface__srv__Arm__get_type_hash,
  &arm_interface__srv__Arm__get_type_description,
  &arm_interface__srv__Arm__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_interface, srv, Arm_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_interface, srv, Arm_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_interface, srv, Arm_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arm_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_interface, srv, Arm)(void) {
  if (!arm_interface__srv__detail__arm__rosidl_typesupport_introspection_c__Arm_service_type_support_handle.typesupport_identifier) {
    arm_interface__srv__detail__arm__rosidl_typesupport_introspection_c__Arm_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)arm_interface__srv__detail__arm__rosidl_typesupport_introspection_c__Arm_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_interface, srv, Arm_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_interface, srv, Arm_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_interface, srv, Arm_Event)()->data;
  }

  return &arm_interface__srv__detail__arm__rosidl_typesupport_introspection_c__Arm_service_type_support_handle;
}
