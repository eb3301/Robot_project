// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robp_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/msg/encoders.hpp"


#ifndef ROBP_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_HPP_
#define ROBP_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robp_interfaces__msg__Encoders __attribute__((deprecated))
#else
# define DEPRECATED__robp_interfaces__msg__Encoders __declspec(deprecated)
#endif

namespace robp_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Encoders_
{
  using Type = Encoders_<ContainerAllocator>;

  explicit Encoders_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->encoder_left = 0ll;
      this->encoder_right = 0ll;
      this->delta_encoder_left = 0l;
      this->delta_encoder_right = 0l;
    }
  }

  explicit Encoders_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->encoder_left = 0ll;
      this->encoder_right = 0ll;
      this->delta_encoder_left = 0l;
      this->delta_encoder_right = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _encoder_left_type =
    int64_t;
  _encoder_left_type encoder_left;
  using _encoder_right_type =
    int64_t;
  _encoder_right_type encoder_right;
  using _delta_encoder_left_type =
    int32_t;
  _delta_encoder_left_type delta_encoder_left;
  using _delta_encoder_right_type =
    int32_t;
  _delta_encoder_right_type delta_encoder_right;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__encoder_left(
    const int64_t & _arg)
  {
    this->encoder_left = _arg;
    return *this;
  }
  Type & set__encoder_right(
    const int64_t & _arg)
  {
    this->encoder_right = _arg;
    return *this;
  }
  Type & set__delta_encoder_left(
    const int32_t & _arg)
  {
    this->delta_encoder_left = _arg;
    return *this;
  }
  Type & set__delta_encoder_right(
    const int32_t & _arg)
  {
    this->delta_encoder_right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robp_interfaces::msg::Encoders_<ContainerAllocator> *;
  using ConstRawPtr =
    const robp_interfaces::msg::Encoders_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robp_interfaces::msg::Encoders_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robp_interfaces::msg::Encoders_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robp_interfaces::msg::Encoders_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robp_interfaces::msg::Encoders_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robp_interfaces::msg::Encoders_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robp_interfaces::msg::Encoders_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robp_interfaces::msg::Encoders_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robp_interfaces::msg::Encoders_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robp_interfaces__msg__Encoders
    std::shared_ptr<robp_interfaces::msg::Encoders_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robp_interfaces__msg__Encoders
    std::shared_ptr<robp_interfaces::msg::Encoders_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Encoders_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->encoder_left != other.encoder_left) {
      return false;
    }
    if (this->encoder_right != other.encoder_right) {
      return false;
    }
    if (this->delta_encoder_left != other.delta_encoder_left) {
      return false;
    }
    if (this->delta_encoder_right != other.delta_encoder_right) {
      return false;
    }
    return true;
  }
  bool operator!=(const Encoders_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Encoders_

// alias to use template instance with default allocator
using Encoders =
  robp_interfaces::msg::Encoders_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robp_interfaces

#endif  // ROBP_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_HPP_
