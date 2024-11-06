// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from behavior_interface:msg/BehaviorStatus.idl
// generated code does not contain a copyright notice

#ifndef BEHAVIOR_INTERFACE__MSG__DETAIL__BEHAVIOR_STATUS__TRAITS_HPP_
#define BEHAVIOR_INTERFACE__MSG__DETAIL__BEHAVIOR_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "behavior_interface/msg/detail/behavior_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace behavior_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const BehaviorStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BehaviorStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BehaviorStatus & msg, bool use_flow_style = false)
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

}  // namespace behavior_interface

namespace rosidl_generator_traits
{

[[deprecated("use behavior_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const behavior_interface::msg::BehaviorStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  behavior_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use behavior_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const behavior_interface::msg::BehaviorStatus & msg)
{
  return behavior_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<behavior_interface::msg::BehaviorStatus>()
{
  return "behavior_interface::msg::BehaviorStatus";
}

template<>
inline const char * name<behavior_interface::msg::BehaviorStatus>()
{
  return "behavior_interface/msg/BehaviorStatus";
}

template<>
struct has_fixed_size<behavior_interface::msg::BehaviorStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<behavior_interface::msg::BehaviorStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<behavior_interface::msg::BehaviorStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BEHAVIOR_INTERFACE__MSG__DETAIL__BEHAVIOR_STATUS__TRAITS_HPP_
