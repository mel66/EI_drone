// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from behavior_interface:msg/Command.idl
// generated code does not contain a copyright notice

#ifndef BEHAVIOR_INTERFACE__MSG__DETAIL__COMMAND__STRUCT_H_
#define BEHAVIOR_INTERFACE__MSG__DETAIL__COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Command in the package behavior_interface.
typedef struct behavior_interface__msg__Command
{
  rosidl_runtime_c__String command;
} behavior_interface__msg__Command;

// Struct for a sequence of behavior_interface__msg__Command.
typedef struct behavior_interface__msg__Command__Sequence
{
  behavior_interface__msg__Command * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} behavior_interface__msg__Command__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BEHAVIOR_INTERFACE__MSG__DETAIL__COMMAND__STRUCT_H_
