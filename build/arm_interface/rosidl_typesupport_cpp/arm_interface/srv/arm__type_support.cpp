// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from arm_interface:srv/Arm.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "arm_interface/srv/detail/arm__functions.h"
#include "arm_interface/srv/detail/arm__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace arm_interface
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Arm_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Arm_Request_type_support_ids_t;

static const _Arm_Request_type_support_ids_t _Arm_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Arm_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Arm_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Arm_Request_type_support_symbol_names_t _Arm_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, arm_interface, srv, Arm_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, arm_interface, srv, Arm_Request)),
  }
};

typedef struct _Arm_Request_type_support_data_t
{
  void * data[2];
} _Arm_Request_type_support_data_t;

static _Arm_Request_type_support_data_t _Arm_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Arm_Request_message_typesupport_map = {
  2,
  "arm_interface",
  &_Arm_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Arm_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Arm_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Arm_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Arm_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &arm_interface__srv__Arm_Request__get_type_hash,
  &arm_interface__srv__Arm_Request__get_type_description,
  &arm_interface__srv__Arm_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace arm_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<arm_interface::srv::Arm_Request>()
{
  return &::arm_interface::srv::rosidl_typesupport_cpp::Arm_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, arm_interface, srv, Arm_Request)() {
  return get_message_type_support_handle<arm_interface::srv::Arm_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "arm_interface/srv/detail/arm__functions.h"
// already included above
// #include "arm_interface/srv/detail/arm__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace arm_interface
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Arm_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Arm_Response_type_support_ids_t;

static const _Arm_Response_type_support_ids_t _Arm_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Arm_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Arm_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Arm_Response_type_support_symbol_names_t _Arm_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, arm_interface, srv, Arm_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, arm_interface, srv, Arm_Response)),
  }
};

typedef struct _Arm_Response_type_support_data_t
{
  void * data[2];
} _Arm_Response_type_support_data_t;

static _Arm_Response_type_support_data_t _Arm_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Arm_Response_message_typesupport_map = {
  2,
  "arm_interface",
  &_Arm_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Arm_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Arm_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Arm_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Arm_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &arm_interface__srv__Arm_Response__get_type_hash,
  &arm_interface__srv__Arm_Response__get_type_description,
  &arm_interface__srv__Arm_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace arm_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<arm_interface::srv::Arm_Response>()
{
  return &::arm_interface::srv::rosidl_typesupport_cpp::Arm_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, arm_interface, srv, Arm_Response)() {
  return get_message_type_support_handle<arm_interface::srv::Arm_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "arm_interface/srv/detail/arm__functions.h"
// already included above
// #include "arm_interface/srv/detail/arm__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace arm_interface
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Arm_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Arm_Event_type_support_ids_t;

static const _Arm_Event_type_support_ids_t _Arm_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Arm_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Arm_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Arm_Event_type_support_symbol_names_t _Arm_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, arm_interface, srv, Arm_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, arm_interface, srv, Arm_Event)),
  }
};

typedef struct _Arm_Event_type_support_data_t
{
  void * data[2];
} _Arm_Event_type_support_data_t;

static _Arm_Event_type_support_data_t _Arm_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Arm_Event_message_typesupport_map = {
  2,
  "arm_interface",
  &_Arm_Event_message_typesupport_ids.typesupport_identifier[0],
  &_Arm_Event_message_typesupport_symbol_names.symbol_name[0],
  &_Arm_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Arm_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Arm_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &arm_interface__srv__Arm_Event__get_type_hash,
  &arm_interface__srv__Arm_Event__get_type_description,
  &arm_interface__srv__Arm_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace arm_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<arm_interface::srv::Arm_Event>()
{
  return &::arm_interface::srv::rosidl_typesupport_cpp::Arm_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, arm_interface, srv, Arm_Event)() {
  return get_message_type_support_handle<arm_interface::srv::Arm_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "arm_interface/srv/detail/arm__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace arm_interface
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Arm_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Arm_type_support_ids_t;

static const _Arm_type_support_ids_t _Arm_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Arm_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Arm_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Arm_type_support_symbol_names_t _Arm_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, arm_interface, srv, Arm)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, arm_interface, srv, Arm)),
  }
};

typedef struct _Arm_type_support_data_t
{
  void * data[2];
} _Arm_type_support_data_t;

static _Arm_type_support_data_t _Arm_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Arm_service_typesupport_map = {
  2,
  "arm_interface",
  &_Arm_service_typesupport_ids.typesupport_identifier[0],
  &_Arm_service_typesupport_symbol_names.symbol_name[0],
  &_Arm_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Arm_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Arm_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<arm_interface::srv::Arm_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<arm_interface::srv::Arm_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<arm_interface::srv::Arm_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<arm_interface::srv::Arm>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<arm_interface::srv::Arm>,
  &arm_interface__srv__Arm__get_type_hash,
  &arm_interface__srv__Arm__get_type_description,
  &arm_interface__srv__Arm__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace arm_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<arm_interface::srv::Arm>()
{
  return &::arm_interface::srv::rosidl_typesupport_cpp::Arm_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, arm_interface, srv, Arm)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<arm_interface::srv::Arm>();
}

#ifdef __cplusplus
}
#endif
