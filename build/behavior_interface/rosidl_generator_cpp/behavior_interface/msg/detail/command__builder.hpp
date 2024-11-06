// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from behavior_interface:msg/Command.idl
// generated code does not contain a copyright notice

#ifndef BEHAVIOR_INTERFACE__MSG__DETAIL__COMMAND__BUILDER_HPP_
#define BEHAVIOR_INTERFACE__MSG__DETAIL__COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "behavior_interface/msg/detail/command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace behavior_interface
{

namespace msg
{

namespace builder
{

class Init_Command_command
{
public:
  Init_Command_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::behavior_interface::msg::Command command(::behavior_interface::msg::Command::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::behavior_interface::msg::Command msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::behavior_interface::msg::Command>()
{
  return behavior_interface::msg::builder::Init_Command_command();
}

}  // namespace behavior_interface

#endif  // BEHAVIOR_INTERFACE__MSG__DETAIL__COMMAND__BUILDER_HPP_
