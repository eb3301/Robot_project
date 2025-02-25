// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robp_interfaces:msg/DutyCycles.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robp_interfaces/msg/duty_cycles.hpp"


#ifndef ROBP_INTERFACES__MSG__DETAIL__DUTY_CYCLES__STRUCT_HPP_
#define ROBP_INTERFACES__MSG__DETAIL__DUTY_CYCLES__STRUCT_HPP_

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
# define DEPRECATED__robp_interfaces__msg__DutyCycles __attribute__((deprecated))
#else
# define DEPRECATED__robp_interfaces__msg__DutyCycles __declspec(deprecated)
#endif

namespace robp_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DutyCycles_
{
  using Type = DutyCycles_<ContainerAllocator>;

  explicit DutyCycles_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->duty_cycle_left = 0.0;
      this->duty_cycle_right = 0.0;
    }
  }

  explicit DutyCycles_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->duty_cycle_left = 0.0;
      this->duty_cycle_right = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _duty_cycle_left_type =
    double;
  _duty_cycle_left_type duty_cycle_left;
  using _duty_cycle_right_type =
    double;
  _duty_cycle_right_type duty_cycle_right;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__duty_cycle_left(
    const double & _arg)
  {
    this->duty_cycle_left = _arg;
    return *this;
  }
  Type & set__duty_cycle_right(
    const double & _arg)
  {
    this->duty_cycle_right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robp_interfaces::msg::DutyCycles_<ContainerAllocator> *;
  using ConstRawPtr =
    const robp_interfaces::msg::DutyCycles_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robp_interfaces::msg::DutyCycles_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robp_interfaces::msg::DutyCycles_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robp_interfaces::msg::DutyCycles_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robp_interfaces::msg::DutyCycles_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robp_interfaces::msg::DutyCycles_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robp_interfaces::msg::DutyCycles_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robp_interfaces::msg::DutyCycles_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robp_interfaces::msg::DutyCycles_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robp_interfaces__msg__DutyCycles
    std::shared_ptr<robp_interfaces::msg::DutyCycles_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robp_interfaces__msg__DutyCycles
    std::shared_ptr<robp_interfaces::msg::DutyCycles_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DutyCycles_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->duty_cycle_left != other.duty_cycle_left) {
      return false;
    }
    if (this->duty_cycle_right != other.duty_cycle_right) {
      return false;
    }
    return true;
  }
  bool operator!=(const DutyCycles_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DutyCycles_

// alias to use template instance with default allocator
using DutyCycles =
  robp_interfaces::msg::DutyCycles_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robp_interfaces

#endif  // ROBP_INTERFACES__MSG__DETAIL__DUTY_CYCLES__STRUCT_HPP_
