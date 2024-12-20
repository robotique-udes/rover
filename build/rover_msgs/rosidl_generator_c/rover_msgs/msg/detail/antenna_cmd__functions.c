// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rover_msgs:msg/AntennaCmd.idl
// generated code does not contain a copyright notice
#include "rover_msgs/msg/detail/antenna_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
rover_msgs__msg__AntennaCmd__init(rover_msgs__msg__AntennaCmd * msg)
{
  if (!msg) {
    return false;
  }
  // enable
  // speed
  return true;
}

void
rover_msgs__msg__AntennaCmd__fini(rover_msgs__msg__AntennaCmd * msg)
{
  if (!msg) {
    return;
  }
  // enable
  // speed
}

bool
rover_msgs__msg__AntennaCmd__are_equal(const rover_msgs__msg__AntennaCmd * lhs, const rover_msgs__msg__AntennaCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // enable
  if (lhs->enable != rhs->enable) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  return true;
}

bool
rover_msgs__msg__AntennaCmd__copy(
  const rover_msgs__msg__AntennaCmd * input,
  rover_msgs__msg__AntennaCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // enable
  output->enable = input->enable;
  // speed
  output->speed = input->speed;
  return true;
}

rover_msgs__msg__AntennaCmd *
rover_msgs__msg__AntennaCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__AntennaCmd * msg = (rover_msgs__msg__AntennaCmd *)allocator.allocate(sizeof(rover_msgs__msg__AntennaCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_msgs__msg__AntennaCmd));
  bool success = rover_msgs__msg__AntennaCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_msgs__msg__AntennaCmd__destroy(rover_msgs__msg__AntennaCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_msgs__msg__AntennaCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_msgs__msg__AntennaCmd__Sequence__init(rover_msgs__msg__AntennaCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__AntennaCmd * data = NULL;

  if (size) {
    data = (rover_msgs__msg__AntennaCmd *)allocator.zero_allocate(size, sizeof(rover_msgs__msg__AntennaCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_msgs__msg__AntennaCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_msgs__msg__AntennaCmd__fini(&data[i - 1]);
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
rover_msgs__msg__AntennaCmd__Sequence__fini(rover_msgs__msg__AntennaCmd__Sequence * array)
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
      rover_msgs__msg__AntennaCmd__fini(&array->data[i]);
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

rover_msgs__msg__AntennaCmd__Sequence *
rover_msgs__msg__AntennaCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__AntennaCmd__Sequence * array = (rover_msgs__msg__AntennaCmd__Sequence *)allocator.allocate(sizeof(rover_msgs__msg__AntennaCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_msgs__msg__AntennaCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_msgs__msg__AntennaCmd__Sequence__destroy(rover_msgs__msg__AntennaCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_msgs__msg__AntennaCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_msgs__msg__AntennaCmd__Sequence__are_equal(const rover_msgs__msg__AntennaCmd__Sequence * lhs, const rover_msgs__msg__AntennaCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_msgs__msg__AntennaCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_msgs__msg__AntennaCmd__Sequence__copy(
  const rover_msgs__msg__AntennaCmd__Sequence * input,
  rover_msgs__msg__AntennaCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_msgs__msg__AntennaCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_msgs__msg__AntennaCmd * data =
      (rover_msgs__msg__AntennaCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_msgs__msg__AntennaCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_msgs__msg__AntennaCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_msgs__msg__AntennaCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
