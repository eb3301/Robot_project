// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arm_interface:srv/Arm.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arm_interface/srv/arm.hpp"


#ifndef ARM_INTERFACE__SRV__DETAIL__ARM__BUILDER_HPP_
#define ARM_INTERFACE__SRV__DETAIL__ARM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arm_interface/srv/detail/arm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arm_interface
{

namespace srv
{

namespace builder
{

class Init_Arm_Request_obj_class
{
public:
  explicit Init_Arm_Request_obj_class(::arm_interface::srv::Arm_Request & msg)
  : msg_(msg)
  {}
  ::arm_interface::srv::Arm_Request obj_class(::arm_interface::srv::Arm_Request::_obj_class_type arg)
  {
    msg_.obj_class = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_interface::srv::Arm_Request msg_;
};

class Init_Arm_Request_xy
{
public:
  Init_Arm_Request_xy()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Arm_Request_obj_class xy(::arm_interface::srv::Arm_Request::_xy_type arg)
  {
    msg_.xy = std::move(arg);
    return Init_Arm_Request_obj_class(msg_);
  }

private:
  ::arm_interface::srv::Arm_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_interface::srv::Arm_Request>()
{
  return arm_interface::srv::builder::Init_Arm_Request_xy();
}

}  // namespace arm_interface


namespace arm_interface
{

namespace srv
{

namespace builder
{

class Init_Arm_Response_success
{
public:
  Init_Arm_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::arm_interface::srv::Arm_Response success(::arm_interface::srv::Arm_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_interface::srv::Arm_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_interface::srv::Arm_Response>()
{
  return arm_interface::srv::builder::Init_Arm_Response_success();
}

}  // namespace arm_interface


namespace arm_interface
{

namespace srv
{

namespace builder
{

class Init_Arm_Event_response
{
public:
  explicit Init_Arm_Event_response(::arm_interface::srv::Arm_Event & msg)
  : msg_(msg)
  {}
  ::arm_interface::srv::Arm_Event response(::arm_interface::srv::Arm_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_interface::srv::Arm_Event msg_;
};

class Init_Arm_Event_request
{
public:
  explicit Init_Arm_Event_request(::arm_interface::srv::Arm_Event & msg)
  : msg_(msg)
  {}
  Init_Arm_Event_response request(::arm_interface::srv::Arm_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Arm_Event_response(msg_);
  }

private:
  ::arm_interface::srv::Arm_Event msg_;
};

class Init_Arm_Event_info
{
public:
  Init_Arm_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Arm_Event_request info(::arm_interface::srv::Arm_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Arm_Event_request(msg_);
  }

private:
  ::arm_interface::srv::Arm_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_interface::srv::Arm_Event>()
{
  return arm_interface::srv::builder::Init_Arm_Event_info();
}

}  // namespace arm_interface

#endif  // ARM_INTERFACE__SRV__DETAIL__ARM__BUILDER_HPP_
