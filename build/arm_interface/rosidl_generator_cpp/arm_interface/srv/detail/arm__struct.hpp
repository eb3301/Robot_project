// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from arm_interface:srv/Arm.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arm_interface/srv/arm.hpp"


#ifndef ARM_INTERFACE__SRV__DETAIL__ARM__STRUCT_HPP_
#define ARM_INTERFACE__SRV__DETAIL__ARM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__arm_interface__srv__Arm_Request __attribute__((deprecated))
#else
# define DEPRECATED__arm_interface__srv__Arm_Request __declspec(deprecated)
#endif

namespace arm_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Arm_Request_
{
  using Type = Arm_Request_<ContainerAllocator>;

  explicit Arm_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 2>::iterator, float>(this->xy.begin(), this->xy.end(), 0.0f);
      this->obj_class = "";
    }
  }

  explicit Arm_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : xy(_alloc),
    obj_class(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 2>::iterator, float>(this->xy.begin(), this->xy.end(), 0.0f);
      this->obj_class = "";
    }
  }

  // field types and members
  using _xy_type =
    std::array<float, 2>;
  _xy_type xy;
  using _obj_class_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _obj_class_type obj_class;

  // setters for named parameter idiom
  Type & set__xy(
    const std::array<float, 2> & _arg)
  {
    this->xy = _arg;
    return *this;
  }
  Type & set__obj_class(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->obj_class = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_interface::srv::Arm_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_interface::srv::Arm_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_interface::srv::Arm_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_interface::srv::Arm_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_interface::srv::Arm_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_interface::srv::Arm_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_interface::srv::Arm_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_interface::srv::Arm_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_interface::srv::Arm_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_interface::srv::Arm_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_interface__srv__Arm_Request
    std::shared_ptr<arm_interface::srv::Arm_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_interface__srv__Arm_Request
    std::shared_ptr<arm_interface::srv::Arm_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Arm_Request_ & other) const
  {
    if (this->xy != other.xy) {
      return false;
    }
    if (this->obj_class != other.obj_class) {
      return false;
    }
    return true;
  }
  bool operator!=(const Arm_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Arm_Request_

// alias to use template instance with default allocator
using Arm_Request =
  arm_interface::srv::Arm_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace arm_interface


#ifndef _WIN32
# define DEPRECATED__arm_interface__srv__Arm_Response __attribute__((deprecated))
#else
# define DEPRECATED__arm_interface__srv__Arm_Response __declspec(deprecated)
#endif

namespace arm_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Arm_Response_
{
  using Type = Arm_Response_<ContainerAllocator>;

  explicit Arm_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit Arm_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_interface::srv::Arm_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_interface::srv::Arm_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_interface::srv::Arm_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_interface::srv::Arm_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_interface::srv::Arm_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_interface::srv::Arm_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_interface::srv::Arm_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_interface::srv::Arm_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_interface::srv::Arm_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_interface::srv::Arm_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_interface__srv__Arm_Response
    std::shared_ptr<arm_interface::srv::Arm_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_interface__srv__Arm_Response
    std::shared_ptr<arm_interface::srv::Arm_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Arm_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const Arm_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Arm_Response_

// alias to use template instance with default allocator
using Arm_Response =
  arm_interface::srv::Arm_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace arm_interface


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_interface__srv__Arm_Event __attribute__((deprecated))
#else
# define DEPRECATED__arm_interface__srv__Arm_Event __declspec(deprecated)
#endif

namespace arm_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Arm_Event_
{
  using Type = Arm_Event_<ContainerAllocator>;

  explicit Arm_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit Arm_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<arm_interface::srv::Arm_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<arm_interface::srv::Arm_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<arm_interface::srv::Arm_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<arm_interface::srv::Arm_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<arm_interface::srv::Arm_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<arm_interface::srv::Arm_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<arm_interface::srv::Arm_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<arm_interface::srv::Arm_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_interface::srv::Arm_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_interface::srv::Arm_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_interface::srv::Arm_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_interface::srv::Arm_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_interface::srv::Arm_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_interface::srv::Arm_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_interface::srv::Arm_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_interface::srv::Arm_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_interface::srv::Arm_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_interface::srv::Arm_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_interface__srv__Arm_Event
    std::shared_ptr<arm_interface::srv::Arm_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_interface__srv__Arm_Event
    std::shared_ptr<arm_interface::srv::Arm_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Arm_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const Arm_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Arm_Event_

// alias to use template instance with default allocator
using Arm_Event =
  arm_interface::srv::Arm_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace arm_interface

namespace arm_interface
{

namespace srv
{

struct Arm
{
  using Request = arm_interface::srv::Arm_Request;
  using Response = arm_interface::srv::Arm_Response;
  using Event = arm_interface::srv::Arm_Event;
};

}  // namespace srv

}  // namespace arm_interface

#endif  // ARM_INTERFACE__SRV__DETAIL__ARM__STRUCT_HPP_
