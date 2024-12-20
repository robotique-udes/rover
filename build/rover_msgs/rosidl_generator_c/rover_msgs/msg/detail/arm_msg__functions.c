// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rover_msgs:msg/ArmMsg.idl
// generated code does not contain a copyright notice
#include "rover_msgs/msg/detail/arm_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
rover_msgs__msg__ArmMsg__init(rover_msgs__msg__ArmMsg * msg)
{
  if (!msg) {
    return false;
  }
  // data
  return true;
}

void
rover_msgs__msg__ArmMsg__fini(rover_msgs__msg__ArmMsg * msg)
{
  if (!msg) {
    return;
  }
  // data
}

bool
rover_msgs__msg__ArmMsg__are_equal(const rover_msgs__msg__ArmMsg * lhs, const rover_msgs__msg__ArmMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->data[i] != rhs->data[i]) {
      return false;
    }
  }
  return true;
}

bool
rover_msgs__msg__ArmMsg__copy(
  const rover_msgs__msg__ArmMsg * input,
  rover_msgs__msg__ArmMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // data
  for (size_t i = 0; i < 7; ++i) {
    output->data[i] = input->data[i];
  }
  return true;
}

rover_msgs__msg__ArmMsg *
rover_msgs__msg__ArmMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__ArmMsg * msg = (rover_msgs__msg__ArmMsg *)allocator.allocate(sizeof(rover_msgs__msg__ArmMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_msgs__msg__ArmMsg));
  bool success = rover_msgs__msg__ArmMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_msgs__msg__ArmMsg__destroy(rover_msgs__msg__ArmMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_msgs__msg__ArmMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_msgs__msg__ArmMsg__Sequence__init(rover_msgs__msg__ArmMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__ArmMsg * data = NULL;

  if (size) {
    data = (rover_msgs__msg__ArmMsg *)allocator.zero_allocate(size, sizeof(rover_msgs__msg__ArmMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_msgs__msg__ArmMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_msgs__msg__ArmMsg__fini(&data[i - 1]);
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
rover_msgs__msg__ArmMsg__Sequence__fini(rover_msgs__msg__ArmMsg__Sequence * array)
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
      rover_msgs__msg__ArmMsg__fini(&array->data[i]);
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

rover_msgs__msg__ArmMsg__Sequence *
rover_msgs__msg__ArmMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__ArmMsg__Sequence * array = (rover_msgs__msg__ArmMsg__Sequence *)allocator.allocate(sizeof(rover_msgs__msg__ArmMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_msgs__msg__ArmMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_msgs__msg__ArmMsg__Sequence__destroy(rover_msgs__msg__ArmMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_msgs__msg__ArmMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_msgs__msg__ArmMsg__Sequence__are_equal(const rover_msgs__msg__ArmMsg__Sequence * lhs, const rover_msgs__msg__ArmMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_msgs__msg__ArmMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_msgs__msg__ArmMsg__Sequence__copy(
  const rover_msgs__msg__ArmMsg__Sequence * input,
  rover_msgs__msg__ArmMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_msgs__msg__ArmMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_msgs__msg__ArmMsg * data =
      (rover_msgs__msg__ArmMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_msgs__msg__ArmMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_msgs__msg__ArmMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_msgs__msg__ArmMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
