// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arm_interface:srv/Arm.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arm_interface/srv/arm.hpp"


#ifndef ARM_INTERFACE__SRV__DETAIL__ARM__TRAITS_HPP_
#define ARM_INTERFACE__SRV__DETAIL__ARM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arm_interface/srv/detail/arm__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace arm_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Arm_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: xy
  {
    if (msg.xy.size() == 0) {
      out << "xy: []";
    } else {
      out << "xy: [";
      size_t pending_items = msg.xy.size();
      for (auto item : msg.xy) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: obj_class
  {
    out << "obj_class: ";
    rosidl_generator_traits::value_to_yaml(msg.obj_class, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Arm_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: xy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.xy.size() == 0) {
      out << "xy: []\n";
    } else {
      out << "xy:\n";
      for (auto item : msg.xy) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: obj_class
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obj_class: ";
    rosidl_generator_traits::value_to_yaml(msg.obj_class, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Arm_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace arm_interface

namespace rosidl_generator_traits
{

[[deprecated("use arm_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_interface::srv::Arm_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const arm_interface::srv::Arm_Request & msg)
{
  return arm_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<arm_interface::srv::Arm_Request>()
{
  return "arm_interface::srv::Arm_Request";
}

template<>
inline const char * name<arm_interface::srv::Arm_Request>()
{
  return "arm_interface/srv/Arm_Request";
}

template<>
struct has_fixed_size<arm_interface::srv::Arm_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arm_interface::srv::Arm_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arm_interface::srv::Arm_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace arm_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Arm_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Arm_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Arm_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace arm_interface

namespace rosidl_generator_traits
{

[[deprecated("use arm_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_interface::srv::Arm_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const arm_interface::srv::Arm_Response & msg)
{
  return arm_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<arm_interface::srv::Arm_Response>()
{
  return "arm_interface::srv::Arm_Response";
}

template<>
inline const char * name<arm_interface::srv::Arm_Response>()
{
  return "arm_interface/srv/Arm_Response";
}

template<>
struct has_fixed_size<arm_interface::srv::Arm_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<arm_interface::srv::Arm_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<arm_interface::srv::Arm_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace arm_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Arm_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Arm_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Arm_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace arm_interface

namespace rosidl_generator_traits
{

[[deprecated("use arm_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_interface::srv::Arm_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const arm_interface::srv::Arm_Event & msg)
{
  return arm_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<arm_interface::srv::Arm_Event>()
{
  return "arm_interface::srv::Arm_Event";
}

template<>
inline const char * name<arm_interface::srv::Arm_Event>()
{
  return "arm_interface/srv/Arm_Event";
}

template<>
struct has_fixed_size<arm_interface::srv::Arm_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arm_interface::srv::Arm_Event>
  : std::integral_constant<bool, has_bounded_size<arm_interface::srv::Arm_Request>::value && has_bounded_size<arm_interface::srv::Arm_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<arm_interface::srv::Arm_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<arm_interface::srv::Arm>()
{
  return "arm_interface::srv::Arm";
}

template<>
inline const char * name<arm_interface::srv::Arm>()
{
  return "arm_interface/srv/Arm";
}

template<>
struct has_fixed_size<arm_interface::srv::Arm>
  : std::integral_constant<
    bool,
    has_fixed_size<arm_interface::srv::Arm_Request>::value &&
    has_fixed_size<arm_interface::srv::Arm_Response>::value
  >
{
};

template<>
struct has_bounded_size<arm_interface::srv::Arm>
  : std::integral_constant<
    bool,
    has_bounded_size<arm_interface::srv::Arm_Request>::value &&
    has_bounded_size<arm_interface::srv::Arm_Response>::value
  >
{
};

template<>
struct is_service<arm_interface::srv::Arm>
  : std::true_type
{
};

template<>
struct is_service_request<arm_interface::srv::Arm_Request>
  : std::true_type
{
};

template<>
struct is_service_response<arm_interface::srv::Arm_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ARM_INTERFACE__SRV__DETAIL__ARM__TRAITS_HPP_
