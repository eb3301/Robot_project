// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arm_interface:srv/Arm.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arm_interface/srv/arm.h"


#ifndef ARM_INTERFACE__SRV__DETAIL__ARM__STRUCT_H_
#define ARM_INTERFACE__SRV__DETAIL__ARM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'obj_class'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Arm in the package arm_interface.
typedef struct arm_interface__srv__Arm_Request
{
  float xy[2];
  rosidl_runtime_c__String obj_class;
} arm_interface__srv__Arm_Request;

// Struct for a sequence of arm_interface__srv__Arm_Request.
typedef struct arm_interface__srv__Arm_Request__Sequence
{
  arm_interface__srv__Arm_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_interface__srv__Arm_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/Arm in the package arm_interface.
typedef struct arm_interface__srv__Arm_Response
{
  bool success;
} arm_interface__srv__Arm_Response;

// Struct for a sequence of arm_interface__srv__Arm_Response.
typedef struct arm_interface__srv__Arm_Response__Sequence
{
  arm_interface__srv__Arm_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_interface__srv__Arm_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  arm_interface__srv__Arm_Event__request__MAX_SIZE = 1
};
// response
enum
{
  arm_interface__srv__Arm_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Arm in the package arm_interface.
typedef struct arm_interface__srv__Arm_Event
{
  service_msgs__msg__ServiceEventInfo info;
  arm_interface__srv__Arm_Request__Sequence request;
  arm_interface__srv__Arm_Response__Sequence response;
} arm_interface__srv__Arm_Event;

// Struct for a sequence of arm_interface__srv__Arm_Event.
typedef struct arm_interface__srv__Arm_Event__Sequence
{
  arm_interface__srv__Arm_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_interface__srv__Arm_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARM_INTERFACE__SRV__DETAIL__ARM__STRUCT_H_
