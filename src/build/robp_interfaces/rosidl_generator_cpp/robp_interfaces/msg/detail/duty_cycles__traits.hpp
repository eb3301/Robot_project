// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robp_interfaces:msg/DutyCycles.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/msg/duty_cycles.hpp"


#ifndef ROBP_INTERFACES__MSG__DETAIL__DUTY_CYCLES__TRAITS_HPP_
#define ROBP_INTERFACES__MSG__DETAIL__DUTY_CYCLES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robp_interfaces/msg/detail/duty_cycles__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace robp_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const DutyCycles & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: duty_cycle_left
  {
    out << "duty_cycle_left: ";
    rosidl_generator_traits::value_to_yaml(msg.duty_cycle_left, out);
    out << ", ";
  }

  // member: duty_cycle_right
  {
    out << "duty_cycle_right: ";
    rosidl_generator_traits::value_to_yaml(msg.duty_cycle_right, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DutyCycles & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: duty_cycle_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duty_cycle_left: ";
    rosidl_generator_traits::value_to_yaml(msg.duty_cycle_left, out);
    out << "\n";
  }

  // member: duty_cycle_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duty_cycle_right: ";
    rosidl_generator_traits::value_to_yaml(msg.duty_cycle_right, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DutyCycles & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robp_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robp_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robp_interfaces::msg::DutyCycles & msg,
  std::ostream & out, size_t indentation = 0)
{
  robp_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robp_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const robp_interfaces::msg::DutyCycles & msg)
{
  return robp_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robp_interfaces::msg::DutyCycles>()
{
  return "robp_interfaces::msg::DutyCycles";
}

template<>
inline const char * name<robp_interfaces::msg::DutyCycles>()
{
  return "robp_interfaces/msg/DutyCycles";
}

template<>
struct has_fixed_size<robp_interfaces::msg::DutyCycles>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<robp_interfaces::msg::DutyCycles>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<robp_interfaces::msg::DutyCycles>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBP_INTERFACES__MSG__DETAIL__DUTY_CYCLES__TRAITS_HPP_
