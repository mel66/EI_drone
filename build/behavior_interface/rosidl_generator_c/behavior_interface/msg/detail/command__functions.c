// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from behavior_interface:msg/Command.idl
// generated code does not contain a copyright notice
#include "behavior_interface/msg/detail/command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `command`
#include "rosidl_runtime_c/string_functions.h"

bool
behavior_interface__msg__Command__init(behavior_interface__msg__Command * msg)
{
  if (!msg) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__init(&msg->command)) {
    behavior_interface__msg__Command__fini(msg);
    return false;
  }
  return true;
}

void
behavior_interface__msg__Command__fini(behavior_interface__msg__Command * msg)
{
  if (!msg) {
    return;
  }
  // command
  rosidl_runtime_c__String__fini(&msg->command);
}

bool
behavior_interface__msg__Command__are_equal(const behavior_interface__msg__Command * lhs, const behavior_interface__msg__Command * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->command), &(rhs->command)))
  {
    return false;
  }
  return true;
}

bool
behavior_interface__msg__Command__copy(
  const behavior_interface__msg__Command * input,
  behavior_interface__msg__Command * output)
{
  if (!input || !output) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__copy(
      &(input->command), &(output->command)))
  {
    return false;
  }
  return true;
}

behavior_interface__msg__Command *
behavior_interface__msg__Command__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  behavior_interface__msg__Command * msg = (behavior_interface__msg__Command *)allocator.allocate(sizeof(behavior_interface__msg__Command), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(behavior_interface__msg__Command));
  bool success = behavior_interface__msg__Command__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
behavior_interface__msg__Command__destroy(behavior_interface__msg__Command * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    behavior_interface__msg__Command__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
behavior_interface__msg__Command__Sequence__init(behavior_interface__msg__Command__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  behavior_interface__msg__Command * data = NULL;

  if (size) {
    data = (behavior_interface__msg__Command *)allocator.zero_allocate(size, sizeof(behavior_interface__msg__Command), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = behavior_interface__msg__Command__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        behavior_interface__msg__Command__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
behavior_interface__msg__Command__Sequence__fini(behavior_interface__msg__Command__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      behavior_interface__msg__Command__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

behavior_interface__msg__Command__Sequence *
behavior_interface__msg__Command__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  behavior_interface__msg__Command__Sequence * array = (behavior_interface__msg__Command__Sequence *)allocator.allocate(sizeof(behavior_interface__msg__Command__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = behavior_interface__msg__Command__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
behavior_interface__msg__Command__Sequence__destroy(behavior_interface__msg__Command__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    behavior_interface__msg__Command__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
behavior_interface__msg__Command__Sequence__are_equal(const behavior_interface__msg__Command__Sequence * lhs, const behavior_interface__msg__Command__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!behavior_interface__msg__Command__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
behavior_interface__msg__Command__Sequence__copy(
  const behavior_interface__msg__Command__Sequence * input,
  behavior_interface__msg__Command__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(behavior_interface__msg__Command);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    behavior_interface__msg__Command * data =
      (behavior_interface__msg__Command *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!behavior_interface__msg__Command__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          behavior_interface__msg__Command__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!behavior_interface__msg__Command__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
