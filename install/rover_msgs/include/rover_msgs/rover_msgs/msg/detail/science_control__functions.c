// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rover_msgs:msg/ScienceControl.idl
// generated code does not contain a copyright notice
#include "rover_msgs/msg/detail/science_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
rover_msgs__msg__ScienceControl__init(rover_msgs__msg__ScienceControl * msg)
{
  if (!msg) {
    return false;
  }
  // cmd
  // current_sample
  // dig
  return true;
}

void
rover_msgs__msg__ScienceControl__fini(rover_msgs__msg__ScienceControl * msg)
{
  if (!msg) {
    return;
  }
  // cmd
  // current_sample
  // dig
}

bool
rover_msgs__msg__ScienceControl__are_equal(const rover_msgs__msg__ScienceControl * lhs, const rover_msgs__msg__ScienceControl * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cmd
  if (lhs->cmd != rhs->cmd) {
    return false;
  }
  // current_sample
  if (lhs->current_sample != rhs->current_sample) {
    return false;
  }
  // dig
  if (lhs->dig != rhs->dig) {
    return false;
  }
  return true;
}

bool
rover_msgs__msg__ScienceControl__copy(
  const rover_msgs__msg__ScienceControl * input,
  rover_msgs__msg__ScienceControl * output)
{
  if (!input || !output) {
    return false;
  }
  // cmd
  output->cmd = input->cmd;
  // current_sample
  output->current_sample = input->current_sample;
  // dig
  output->dig = input->dig;
  return true;
}

rover_msgs__msg__ScienceControl *
rover_msgs__msg__ScienceControl__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__ScienceControl * msg = (rover_msgs__msg__ScienceControl *)allocator.allocate(sizeof(rover_msgs__msg__ScienceControl), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_msgs__msg__ScienceControl));
  bool success = rover_msgs__msg__ScienceControl__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_msgs__msg__ScienceControl__destroy(rover_msgs__msg__ScienceControl * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_msgs__msg__ScienceControl__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_msgs__msg__ScienceControl__Sequence__init(rover_msgs__msg__ScienceControl__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__ScienceControl * data = NULL;

  if (size) {
    data = (rover_msgs__msg__ScienceControl *)allocator.zero_allocate(size, sizeof(rover_msgs__msg__ScienceControl), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_msgs__msg__ScienceControl__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_msgs__msg__ScienceControl__fini(&data[i - 1]);
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
rover_msgs__msg__ScienceControl__Sequence__fini(rover_msgs__msg__ScienceControl__Sequence * array)
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
      rover_msgs__msg__ScienceControl__fini(&array->data[i]);
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

rover_msgs__msg__ScienceControl__Sequence *
rover_msgs__msg__ScienceControl__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__ScienceControl__Sequence * array = (rover_msgs__msg__ScienceControl__Sequence *)allocator.allocate(sizeof(rover_msgs__msg__ScienceControl__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_msgs__msg__ScienceControl__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_msgs__msg__ScienceControl__Sequence__destroy(rover_msgs__msg__ScienceControl__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_msgs__msg__ScienceControl__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_msgs__msg__ScienceControl__Sequence__are_equal(const rover_msgs__msg__ScienceControl__Sequence * lhs, const rover_msgs__msg__ScienceControl__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_msgs__msg__ScienceControl__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_msgs__msg__ScienceControl__Sequence__copy(
  const rover_msgs__msg__ScienceControl__Sequence * input,
  rover_msgs__msg__ScienceControl__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_msgs__msg__ScienceControl);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_msgs__msg__ScienceControl * data =
      (rover_msgs__msg__ScienceControl *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_msgs__msg__ScienceControl__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_msgs__msg__ScienceControl__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_msgs__msg__ScienceControl__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
