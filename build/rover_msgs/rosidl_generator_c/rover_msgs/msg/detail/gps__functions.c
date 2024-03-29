// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rover_msgs:msg/Gps.idl
// generated code does not contain a copyright notice
#include "rover_msgs/msg/detail/gps__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
rover_msgs__msg__Gps__init(rover_msgs__msg__Gps * msg)
{
  if (!msg) {
    return false;
  }
  // latitude
  // longitude
  // height
  // heading_gps
  // heading_track
  // speed
  // satellite
  // heading
  return true;
}

void
rover_msgs__msg__Gps__fini(rover_msgs__msg__Gps * msg)
{
  if (!msg) {
    return;
  }
  // latitude
  // longitude
  // height
  // heading_gps
  // heading_track
  // speed
  // satellite
  // heading
}

bool
rover_msgs__msg__Gps__are_equal(const rover_msgs__msg__Gps * lhs, const rover_msgs__msg__Gps * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // latitude
  if (lhs->latitude != rhs->latitude) {
    return false;
  }
  // longitude
  if (lhs->longitude != rhs->longitude) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // heading_gps
  if (lhs->heading_gps != rhs->heading_gps) {
    return false;
  }
  // heading_track
  if (lhs->heading_track != rhs->heading_track) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // satellite
  if (lhs->satellite != rhs->satellite) {
    return false;
  }
  // heading
  if (lhs->heading != rhs->heading) {
    return false;
  }
  return true;
}

bool
rover_msgs__msg__Gps__copy(
  const rover_msgs__msg__Gps * input,
  rover_msgs__msg__Gps * output)
{
  if (!input || !output) {
    return false;
  }
  // latitude
  output->latitude = input->latitude;
  // longitude
  output->longitude = input->longitude;
  // height
  output->height = input->height;
  // heading_gps
  output->heading_gps = input->heading_gps;
  // heading_track
  output->heading_track = input->heading_track;
  // speed
  output->speed = input->speed;
  // satellite
  output->satellite = input->satellite;
  // heading
  output->heading = input->heading;
  return true;
}

rover_msgs__msg__Gps *
rover_msgs__msg__Gps__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__Gps * msg = (rover_msgs__msg__Gps *)allocator.allocate(sizeof(rover_msgs__msg__Gps), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_msgs__msg__Gps));
  bool success = rover_msgs__msg__Gps__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_msgs__msg__Gps__destroy(rover_msgs__msg__Gps * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_msgs__msg__Gps__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_msgs__msg__Gps__Sequence__init(rover_msgs__msg__Gps__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__Gps * data = NULL;

  if (size) {
    data = (rover_msgs__msg__Gps *)allocator.zero_allocate(size, sizeof(rover_msgs__msg__Gps), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_msgs__msg__Gps__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_msgs__msg__Gps__fini(&data[i - 1]);
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
rover_msgs__msg__Gps__Sequence__fini(rover_msgs__msg__Gps__Sequence * array)
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
      rover_msgs__msg__Gps__fini(&array->data[i]);
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

rover_msgs__msg__Gps__Sequence *
rover_msgs__msg__Gps__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__Gps__Sequence * array = (rover_msgs__msg__Gps__Sequence *)allocator.allocate(sizeof(rover_msgs__msg__Gps__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_msgs__msg__Gps__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_msgs__msg__Gps__Sequence__destroy(rover_msgs__msg__Gps__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_msgs__msg__Gps__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_msgs__msg__Gps__Sequence__are_equal(const rover_msgs__msg__Gps__Sequence * lhs, const rover_msgs__msg__Gps__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_msgs__msg__Gps__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_msgs__msg__Gps__Sequence__copy(
  const rover_msgs__msg__Gps__Sequence * input,
  rover_msgs__msg__Gps__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_msgs__msg__Gps);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_msgs__msg__Gps * data =
      (rover_msgs__msg__Gps *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_msgs__msg__Gps__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_msgs__msg__Gps__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_msgs__msg__Gps__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
