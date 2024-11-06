// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from behavior_interface:msg/BehaviorStatus.idl
// generated code does not contain a copyright notice

#ifndef BEHAVIOR_INTERFACE__MSG__DETAIL__BEHAVIOR_STATUS__BUILDER_HPP_
#define BEHAVIOR_INTERFACE__MSG__DETAIL__BEHAVIOR_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "behavior_interface/msg/detail/behavior_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace behavior_interface
{

namespace msg
{

namespace builder
{

class Init_BehaviorStatus_status
{
public:
  explicit Init_BehaviorStatus_status(::behavior_interface::msg::BehaviorStatus & msg)
  : msg_(msg)
  {}
  ::behavior_interface::msg::BehaviorStatus status(::behavior_interface::msg::BehaviorStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::behavior_interface::msg::BehaviorStatus msg_;
};

class Init_BehaviorStatus_name
{
public:
  Init_BehaviorStatus_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BehaviorStatus_status name(::behavior_interface::msg::BehaviorStatus::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_BehaviorStatus_status(msg_);
  }

private:
  ::behavior_interface::msg::BehaviorStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::behavior_interface::msg::BehaviorStatus>()
{
  return behavior_interface::msg::builder::Init_BehaviorStatus_name();
}

}  // namespace behavior_interface

#endif  // BEHAVIOR_INTERFACE__MSG__DETAIL__BEHAVIOR_STATUS__BUILDER_HPP_
