// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rover_msgs:srv/ScreenshotControl.idl
// generated code does not contain a copyright notice
#include "rover_msgs/srv/detail/screenshot_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `name`
// Member `ip_address`
// Member `metadata`
#include "rosidl_runtime_c/string_functions.h"

bool
rover_msgs__srv__ScreenshotControl_Request__init(rover_msgs__srv__ScreenshotControl_Request * msg)
{
  if (!msg) {
    return false;
  }
  // start
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    rover_msgs__srv__ScreenshotControl_Request__fini(msg);
    return false;
  }
  // ip_address
  if (!rosidl_runtime_c__String__init(&msg->ip_address)) {
    rover_msgs__srv__ScreenshotControl_Request__fini(msg);
    return false;
  }
  // metadata
  if (!rosidl_runtime_c__String__init(&msg->metadata)) {
    rover_msgs__srv__ScreenshotControl_Request__fini(msg);
    return false;
  }
  return true;
}

void
rover_msgs__srv__ScreenshotControl_Request__fini(rover_msgs__srv__ScreenshotControl_Request * msg)
{
  if (!msg) {
    return;
  }
  // start
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // ip_address
  rosidl_runtime_c__String__fini(&msg->ip_address);
  // metadata
  rosidl_runtime_c__String__fini(&msg->metadata);
}

bool
rover_msgs__srv__ScreenshotControl_Request__are_equal(const rover_msgs__srv__ScreenshotControl_Request * lhs, const rover_msgs__srv__ScreenshotControl_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // start
  if (lhs->start != rhs->start) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // ip_address
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->ip_address), &(rhs->ip_address)))
  {
    return false;
  }
  // metadata
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->metadata), &(rhs->metadata)))
  {
    return false;
  }
  return true;
}

bool
rover_msgs__srv__ScreenshotControl_Request__copy(
  const rover_msgs__srv__ScreenshotControl_Request * input,
  rover_msgs__srv__ScreenshotControl_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // start
  output->start = input->start;
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // ip_address
  if (!rosidl_runtime_c__String__copy(
      &(input->ip_address), &(output->ip_address)))
  {
    return false;
  }
  // metadata
  if (!rosidl_runtime_c__String__copy(
      &(input->metadata), &(output->metadata)))
  {
    return false;
  }
  return true;
}

rover_msgs__srv__ScreenshotControl_Request *
rover_msgs__srv__ScreenshotControl_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__srv__ScreenshotControl_Request * msg = (rover_msgs__srv__ScreenshotControl_Request *)allocator.allocate(sizeof(rover_msgs__srv__ScreenshotControl_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_msgs__srv__ScreenshotControl_Request));
  bool success = rover_msgs__srv__ScreenshotControl_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_msgs__srv__ScreenshotControl_Request__destroy(rover_msgs__srv__ScreenshotControl_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_msgs__srv__ScreenshotControl_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_msgs__srv__ScreenshotControl_Request__Sequence__init(rover_msgs__srv__ScreenshotControl_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__srv__ScreenshotControl_Request * data = NULL;

  if (size) {
    data = (rover_msgs__srv__ScreenshotControl_Request *)allocator.zero_allocate(size, sizeof(rover_msgs__srv__ScreenshotControl_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_msgs__srv__ScreenshotControl_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_msgs__srv__ScreenshotControl_Request__fini(&data[i - 1]);
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
rover_msgs__srv__ScreenshotControl_Request__Sequence__fini(rover_msgs__srv__ScreenshotControl_Request__Sequence * array)
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
      rover_msgs__srv__ScreenshotControl_Request__fini(&array->data[i]);
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

rover_msgs__srv__ScreenshotControl_Request__Sequence *
rover_msgs__srv__ScreenshotControl_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__srv__ScreenshotControl_Request__Sequence * array = (rover_msgs__srv__ScreenshotControl_Request__Sequence *)allocator.allocate(sizeof(rover_msgs__srv__ScreenshotControl_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_msgs__srv__ScreenshotControl_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_msgs__srv__ScreenshotControl_Request__Sequence__destroy(rover_msgs__srv__ScreenshotControl_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_msgs__srv__ScreenshotControl_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_msgs__srv__ScreenshotControl_Request__Sequence__are_equal(const rover_msgs__srv__ScreenshotControl_Request__Sequence * lhs, const rover_msgs__srv__ScreenshotControl_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_msgs__srv__ScreenshotControl_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_msgs__srv__ScreenshotControl_Request__Sequence__copy(
  const rover_msgs__srv__ScreenshotControl_Request__Sequence * input,
  rover_msgs__srv__ScreenshotControl_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_msgs__srv__ScreenshotControl_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_msgs__srv__ScreenshotControl_Request * data =
      (rover_msgs__srv__ScreenshotControl_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_msgs__srv__ScreenshotControl_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_msgs__srv__ScreenshotControl_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_msgs__srv__ScreenshotControl_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `status_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
rover_msgs__srv__ScreenshotControl_Response__init(rover_msgs__srv__ScreenshotControl_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // status_message
  if (!rosidl_runtime_c__String__init(&msg->status_message)) {
    rover_msgs__srv__ScreenshotControl_Response__fini(msg);
    return false;
  }
  return true;
}

void
rover_msgs__srv__ScreenshotControl_Response__fini(rover_msgs__srv__ScreenshotControl_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // status_message
  rosidl_runtime_c__String__fini(&msg->status_message);
}

bool
rover_msgs__srv__ScreenshotControl_Response__are_equal(const rover_msgs__srv__ScreenshotControl_Response * lhs, const rover_msgs__srv__ScreenshotControl_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // status_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status_message), &(rhs->status_message)))
  {
    return false;
  }
  return true;
}

bool
rover_msgs__srv__ScreenshotControl_Response__copy(
  const rover_msgs__srv__ScreenshotControl_Response * input,
  rover_msgs__srv__ScreenshotControl_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // status_message
  if (!rosidl_runtime_c__String__copy(
      &(input->status_message), &(output->status_message)))
  {
    return false;
  }
  return true;
}

rover_msgs__srv__ScreenshotControl_Response *
rover_msgs__srv__ScreenshotControl_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__srv__ScreenshotControl_Response * msg = (rover_msgs__srv__ScreenshotControl_Response *)allocator.allocate(sizeof(rover_msgs__srv__ScreenshotControl_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_msgs__srv__ScreenshotControl_Response));
  bool success = rover_msgs__srv__ScreenshotControl_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_msgs__srv__ScreenshotControl_Response__destroy(rover_msgs__srv__ScreenshotControl_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_msgs__srv__ScreenshotControl_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_msgs__srv__ScreenshotControl_Response__Sequence__init(rover_msgs__srv__ScreenshotControl_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__srv__ScreenshotControl_Response * data = NULL;

  if (size) {
    data = (rover_msgs__srv__ScreenshotControl_Response *)allocator.zero_allocate(size, sizeof(rover_msgs__srv__ScreenshotControl_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_msgs__srv__ScreenshotControl_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_msgs__srv__ScreenshotControl_Response__fini(&data[i - 1]);
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
rover_msgs__srv__ScreenshotControl_Response__Sequence__fini(rover_msgs__srv__ScreenshotControl_Response__Sequence * array)
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
      rover_msgs__srv__ScreenshotControl_Response__fini(&array->data[i]);
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

rover_msgs__srv__ScreenshotControl_Response__Sequence *
rover_msgs__srv__ScreenshotControl_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__srv__ScreenshotControl_Response__Sequence * array = (rover_msgs__srv__ScreenshotControl_Response__Sequence *)allocator.allocate(sizeof(rover_msgs__srv__ScreenshotControl_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_msgs__srv__ScreenshotControl_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_msgs__srv__ScreenshotControl_Response__Sequence__destroy(rover_msgs__srv__ScreenshotControl_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_msgs__srv__ScreenshotControl_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_msgs__srv__ScreenshotControl_Response__Sequence__are_equal(const rover_msgs__srv__ScreenshotControl_Response__Sequence * lhs, const rover_msgs__srv__ScreenshotControl_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_msgs__srv__ScreenshotControl_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_msgs__srv__ScreenshotControl_Response__Sequence__copy(
  const rover_msgs__srv__ScreenshotControl_Response__Sequence * input,
  rover_msgs__srv__ScreenshotControl_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_msgs__srv__ScreenshotControl_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_msgs__srv__ScreenshotControl_Response * data =
      (rover_msgs__srv__ScreenshotControl_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_msgs__srv__ScreenshotControl_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_msgs__srv__ScreenshotControl_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_msgs__srv__ScreenshotControl_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
