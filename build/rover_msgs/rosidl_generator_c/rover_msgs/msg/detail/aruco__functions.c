// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rover_msgs:msg/Aruco.idl
// generated code does not contain a copyright notice
#include "rover_msgs/msg/detail/aruco__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `id`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
rover_msgs__msg__Aruco__init(rover_msgs__msg__Aruco * msg)
{
  if (!msg) {
    return false;
  }
  // valid
  // id
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->id, 0)) {
    rover_msgs__msg__Aruco__fini(msg);
    return false;
  }
  return true;
}

void
rover_msgs__msg__Aruco__fini(rover_msgs__msg__Aruco * msg)
{
  if (!msg) {
    return;
  }
  // valid
  // id
  rosidl_runtime_c__uint16__Sequence__fini(&msg->id);
}

bool
rover_msgs__msg__Aruco__are_equal(const rover_msgs__msg__Aruco * lhs, const rover_msgs__msg__Aruco * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // valid
  if (lhs->valid != rhs->valid) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->id), &(rhs->id)))
  {
    return false;
  }
  return true;
}

bool
rover_msgs__msg__Aruco__copy(
  const rover_msgs__msg__Aruco * input,
  rover_msgs__msg__Aruco * output)
{
  if (!input || !output) {
    return false;
  }
  // valid
  output->valid = input->valid;
  // id
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->id), &(output->id)))
  {
    return false;
  }
  return true;
}

rover_msgs__msg__Aruco *
rover_msgs__msg__Aruco__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__Aruco * msg = (rover_msgs__msg__Aruco *)allocator.allocate(sizeof(rover_msgs__msg__Aruco), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_msgs__msg__Aruco));
  bool success = rover_msgs__msg__Aruco__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_msgs__msg__Aruco__destroy(rover_msgs__msg__Aruco * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_msgs__msg__Aruco__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_msgs__msg__Aruco__Sequence__init(rover_msgs__msg__Aruco__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__Aruco * data = NULL;

  if (size) {
    data = (rover_msgs__msg__Aruco *)allocator.zero_allocate(size, sizeof(rover_msgs__msg__Aruco), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_msgs__msg__Aruco__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_msgs__msg__Aruco__fini(&data[i - 1]);
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
rover_msgs__msg__Aruco__Sequence__fini(rover_msgs__msg__Aruco__Sequence * array)
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
      rover_msgs__msg__Aruco__fini(&array->data[i]);
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

rover_msgs__msg__Aruco__Sequence *
rover_msgs__msg__Aruco__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__Aruco__Sequence * array = (rover_msgs__msg__Aruco__Sequence *)allocator.allocate(sizeof(rover_msgs__msg__Aruco__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_msgs__msg__Aruco__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_msgs__msg__Aruco__Sequence__destroy(rover_msgs__msg__Aruco__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_msgs__msg__Aruco__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_msgs__msg__Aruco__Sequence__are_equal(const rover_msgs__msg__Aruco__Sequence * lhs, const rover_msgs__msg__Aruco__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_msgs__msg__Aruco__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_msgs__msg__Aruco__Sequence__copy(
  const rover_msgs__msg__Aruco__Sequence * input,
  rover_msgs__msg__Aruco__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_msgs__msg__Aruco);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_msgs__msg__Aruco * data =
      (rover_msgs__msg__Aruco *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_msgs__msg__Aruco__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_msgs__msg__Aruco__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_msgs__msg__Aruco__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
