// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from behavior_interface:msg/BehaviorStatus.idl
// generated code does not contain a copyright notice

#ifndef BEHAVIOR_INTERFACE__MSG__DETAIL__BEHAVIOR_STATUS__STRUCT_H_
#define BEHAVIOR_INTERFACE__MSG__DETAIL__BEHAVIOR_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BehaviorStatus in the package behavior_interface.
typedef struct behavior_interface__msg__BehaviorStatus
{
  rosidl_runtime_c__String name;
  bool status;
} behavior_interface__msg__BehaviorStatus;

// Struct for a sequence of behavior_interface__msg__BehaviorStatus.
typedef struct behavior_interface__msg__BehaviorStatus__Sequence
{
  behavior_interface__msg__BehaviorStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} behavior_interface__msg__BehaviorStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BEHAVIOR_INTERFACE__MSG__DETAIL__BEHAVIOR_STATUS__STRUCT_H_
