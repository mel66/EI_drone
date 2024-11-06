// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from behavior_interface:msg/BehaviorStatus.idl
// generated code does not contain a copyright notice

#ifndef BEHAVIOR_INTERFACE__MSG__DETAIL__BEHAVIOR_STATUS__STRUCT_HPP_
#define BEHAVIOR_INTERFACE__MSG__DETAIL__BEHAVIOR_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__behavior_interface__msg__BehaviorStatus __attribute__((deprecated))
#else
# define DEPRECATED__behavior_interface__msg__BehaviorStatus __declspec(deprecated)
#endif

namespace behavior_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BehaviorStatus_
{
  using Type = BehaviorStatus_<ContainerAllocator>;

  explicit BehaviorStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->status = false;
    }
  }

  explicit BehaviorStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->status = false;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _status_type =
    bool;
  _status_type status;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__status(
    const bool & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    behavior_interface::msg::BehaviorStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const behavior_interface::msg::BehaviorStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<behavior_interface::msg::BehaviorStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<behavior_interface::msg::BehaviorStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      behavior_interface::msg::BehaviorStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<behavior_interface::msg::BehaviorStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      behavior_interface::msg::BehaviorStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<behavior_interface::msg::BehaviorStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<behavior_interface::msg::BehaviorStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<behavior_interface::msg::BehaviorStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__behavior_interface__msg__BehaviorStatus
    std::shared_ptr<behavior_interface::msg::BehaviorStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__behavior_interface__msg__BehaviorStatus
    std::shared_ptr<behavior_interface::msg::BehaviorStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BehaviorStatus_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const BehaviorStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BehaviorStatus_

// alias to use template instance with default allocator
using BehaviorStatus =
  behavior_interface::msg::BehaviorStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace behavior_interface

#endif  // BEHAVIOR_INTERFACE__MSG__DETAIL__BEHAVIOR_STATUS__STRUCT_HPP_
