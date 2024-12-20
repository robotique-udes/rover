// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rover_msgs:msg/ArmMsg.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__ARM_MSG__FUNCTIONS_H_
#define ROVER_MSGS__MSG__DETAIL__ARM_MSG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rover_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "rover_msgs/msg/detail/arm_msg__struct.h"

/// Initialize msg/ArmMsg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rover_msgs__msg__ArmMsg
 * )) before or use
 * rover_msgs__msg__ArmMsg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
bool
rover_msgs__msg__ArmMsg__init(rover_msgs__msg__ArmMsg * msg);

/// Finalize msg/ArmMsg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
void
rover_msgs__msg__ArmMsg__fini(rover_msgs__msg__ArmMsg * msg);

/// Create msg/ArmMsg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rover_msgs__msg__ArmMsg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
rover_msgs__msg__ArmMsg *
rover_msgs__msg__ArmMsg__create();

/// Destroy msg/ArmMsg message.
/**
 * It calls
 * rover_msgs__msg__ArmMsg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
void
rover_msgs__msg__ArmMsg__destroy(rover_msgs__msg__ArmMsg * msg);

/// Check for msg/ArmMsg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
bool
rover_msgs__msg__ArmMsg__are_equal(const rover_msgs__msg__ArmMsg * lhs, const rover_msgs__msg__ArmMsg * rhs);

/// Copy a msg/ArmMsg message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
bool
rover_msgs__msg__ArmMsg__copy(
  const rover_msgs__msg__ArmMsg * input,
  rover_msgs__msg__ArmMsg * output);

/// Initialize array of msg/ArmMsg messages.
/**
 * It allocates the memory for the number of elements and calls
 * rover_msgs__msg__ArmMsg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
bool
rover_msgs__msg__ArmMsg__Sequence__init(rover_msgs__msg__ArmMsg__Sequence * array, size_t size);

/// Finalize array of msg/ArmMsg messages.
/**
 * It calls
 * rover_msgs__msg__ArmMsg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
void
rover_msgs__msg__ArmMsg__Sequence__fini(rover_msgs__msg__ArmMsg__Sequence * array);

/// Create array of msg/ArmMsg messages.
/**
 * It allocates the memory for the array and calls
 * rover_msgs__msg__ArmMsg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
rover_msgs__msg__ArmMsg__Sequence *
rover_msgs__msg__ArmMsg__Sequence__create(size_t size);

/// Destroy array of msg/ArmMsg messages.
/**
 * It calls
 * rover_msgs__msg__ArmMsg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
void
rover_msgs__msg__ArmMsg__Sequence__destroy(rover_msgs__msg__ArmMsg__Sequence * array);

/// Check for msg/ArmMsg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
bool
rover_msgs__msg__ArmMsg__Sequence__are_equal(const rover_msgs__msg__ArmMsg__Sequence * lhs, const rover_msgs__msg__ArmMsg__Sequence * rhs);

/// Copy an array of msg/ArmMsg messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
bool
rover_msgs__msg__ArmMsg__Sequence__copy(
  const rover_msgs__msg__ArmMsg__Sequence * input,
  rover_msgs__msg__ArmMsg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__ARM_MSG__FUNCTIONS_H_
