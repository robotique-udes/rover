// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rover_msgs:msg/Joy.idl
// generated code does not contain a copyright notice
#include "rover_msgs/msg/detail/joy__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
rover_msgs__msg__Joy__init(rover_msgs__msg__Joy * msg)
{
  if (!msg) {
    return false;
  }
  // joy_data
  return true;
}

void
rover_msgs__msg__Joy__fini(rover_msgs__msg__Joy * msg)
{
  if (!msg) {
    return;
  }
  // joy_data
}

bool
rover_msgs__msg__Joy__are_equal(const rover_msgs__msg__Joy * lhs, const rover_msgs__msg__Joy * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // joy_data
  for (size_t i = 0; i < 20; ++i) {
    if (lhs->joy_data[i] != rhs->joy_data[i]) {
      return false;
    }
  }
  return true;
}

bool
rover_msgs__msg__Joy__copy(
  const rover_msgs__msg__Joy * input,
  rover_msgs__msg__Joy * output)
{
  if (!input || !output) {
    return false;
  }
  // joy_data
  for (size_t i = 0; i < 20; ++i) {
    output->joy_data[i] = input->joy_data[i];
  }
  return true;
}

rover_msgs__msg__Joy *
rover_msgs__msg__Joy__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__Joy * msg = (rover_msgs__msg__Joy *)allocator.allocate(sizeof(rover_msgs__msg__Joy), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_msgs__msg__Joy));
  bool success = rover_msgs__msg__Joy__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_msgs__msg__Joy__destroy(rover_msgs__msg__Joy * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_msgs__msg__Joy__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_msgs__msg__Joy__Sequence__init(rover_msgs__msg__Joy__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__Joy * data = NULL;

  if (size) {
    data = (rover_msgs__msg__Joy *)allocator.zero_allocate(size, sizeof(rover_msgs__msg__Joy), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_msgs__msg__Joy__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_msgs__msg__Joy__fini(&data[i - 1]);
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
rover_msgs__msg__Joy__Sequence__fini(rover_msgs__msg__Joy__Sequence * array)
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
      rover_msgs__msg__Joy__fini(&array->data[i]);
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

rover_msgs__msg__Joy__Sequence *
rover_msgs__msg__Joy__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__Joy__Sequence * array = (rover_msgs__msg__Joy__Sequence *)allocator.allocate(sizeof(rover_msgs__msg__Joy__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_msgs__msg__Joy__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_msgs__msg__Joy__Sequence__destroy(rover_msgs__msg__Joy__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_msgs__msg__Joy__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_msgs__msg__Joy__Sequence__are_equal(const rover_msgs__msg__Joy__Sequence * lhs, const rover_msgs__msg__Joy__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_msgs__msg__Joy__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_msgs__msg__Joy__Sequence__copy(
  const rover_msgs__msg__Joy__Sequence * input,
  rover_msgs__msg__Joy__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_msgs__msg__Joy);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_msgs__msg__Joy * data =
      (rover_msgs__msg__Joy *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_msgs__msg__Joy__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_msgs__msg__Joy__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_msgs__msg__Joy__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
