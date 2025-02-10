// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robp_interfaces:msg/DutyCycles.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/msg/duty_cycles.hpp"


#ifndef ROBP_INTERFACES__MSG__DETAIL__DUTY_CYCLES__BUILDER_HPP_
#define ROBP_INTERFACES__MSG__DETAIL__DUTY_CYCLES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robp_interfaces/msg/detail/duty_cycles__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robp_interfaces
{

namespace msg
{

namespace builder
{

class Init_DutyCycles_duty_cycle_right
{
public:
  explicit Init_DutyCycles_duty_cycle_right(::robp_interfaces::msg::DutyCycles & msg)
  : msg_(msg)
  {}
  ::robp_interfaces::msg::DutyCycles duty_cycle_right(::robp_interfaces::msg::DutyCycles::_duty_cycle_right_type arg)
  {
    msg_.duty_cycle_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robp_interfaces::msg::DutyCycles msg_;
};

class Init_DutyCycles_duty_cycle_left
{
public:
  explicit Init_DutyCycles_duty_cycle_left(::robp_interfaces::msg::DutyCycles & msg)
  : msg_(msg)
  {}
  Init_DutyCycles_duty_cycle_right duty_cycle_left(::robp_interfaces::msg::DutyCycles::_duty_cycle_left_type arg)
  {
    msg_.duty_cycle_left = std::move(arg);
    return Init_DutyCycles_duty_cycle_right(msg_);
  }

private:
  ::robp_interfaces::msg::DutyCycles msg_;
};

class Init_DutyCycles_header
{
public:
  Init_DutyCycles_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DutyCycles_duty_cycle_left header(::robp_interfaces::msg::DutyCycles::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DutyCycles_duty_cycle_left(msg_);
  }

private:
  ::robp_interfaces::msg::DutyCycles msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robp_interfaces::msg::DutyCycles>()
{
  return robp_interfaces::msg::builder::Init_DutyCycles_header();
}

}  // namespace robp_interfaces

#endif  // ROBP_INTERFACES__MSG__DETAIL__DUTY_CYCLES__BUILDER_HPP_
