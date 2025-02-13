// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robp_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/msg/encoders.hpp"


#ifndef ROBP_INTERFACES__MSG__DETAIL__ENCODERS__BUILDER_HPP_
#define ROBP_INTERFACES__MSG__DETAIL__ENCODERS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robp_interfaces/msg/detail/encoders__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robp_interfaces
{

namespace msg
{

namespace builder
{

class Init_Encoders_delta_encoder_right
{
public:
  explicit Init_Encoders_delta_encoder_right(::robp_interfaces::msg::Encoders & msg)
  : msg_(msg)
  {}
  ::robp_interfaces::msg::Encoders delta_encoder_right(::robp_interfaces::msg::Encoders::_delta_encoder_right_type arg)
  {
    msg_.delta_encoder_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robp_interfaces::msg::Encoders msg_;
};

class Init_Encoders_delta_encoder_left
{
public:
  explicit Init_Encoders_delta_encoder_left(::robp_interfaces::msg::Encoders & msg)
  : msg_(msg)
  {}
  Init_Encoders_delta_encoder_right delta_encoder_left(::robp_interfaces::msg::Encoders::_delta_encoder_left_type arg)
  {
    msg_.delta_encoder_left = std::move(arg);
    return Init_Encoders_delta_encoder_right(msg_);
  }

private:
  ::robp_interfaces::msg::Encoders msg_;
};

class Init_Encoders_encoder_right
{
public:
  explicit Init_Encoders_encoder_right(::robp_interfaces::msg::Encoders & msg)
  : msg_(msg)
  {}
  Init_Encoders_delta_encoder_left encoder_right(::robp_interfaces::msg::Encoders::_encoder_right_type arg)
  {
    msg_.encoder_right = std::move(arg);
    return Init_Encoders_delta_encoder_left(msg_);
  }

private:
  ::robp_interfaces::msg::Encoders msg_;
};

class Init_Encoders_encoder_left
{
public:
  explicit Init_Encoders_encoder_left(::robp_interfaces::msg::Encoders & msg)
  : msg_(msg)
  {}
  Init_Encoders_encoder_right encoder_left(::robp_interfaces::msg::Encoders::_encoder_left_type arg)
  {
    msg_.encoder_left = std::move(arg);
    return Init_Encoders_encoder_right(msg_);
  }

private:
  ::robp_interfaces::msg::Encoders msg_;
};

class Init_Encoders_header
{
public:
  Init_Encoders_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Encoders_encoder_left header(::robp_interfaces::msg::Encoders::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Encoders_encoder_left(msg_);
  }

private:
  ::robp_interfaces::msg::Encoders msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robp_interfaces::msg::Encoders>()
{
  return robp_interfaces::msg::builder::Init_Encoders_header();
}

}  // namespace robp_interfaces

#endif  // ROBP_INTERFACES__MSG__DETAIL__ENCODERS__BUILDER_HPP_
