// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ibeo_8l_msgs:msg/Point2Df.idl
// generated code does not contain a copyright notice

#ifndef IBEO_8L_MSGS__MSG__POINT2_DF__FUNCTIONS_H_
#define IBEO_8L_MSGS__MSG__POINT2_DF__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_generator_c/visibility_control.h"
#include "ibeo_8l_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ibeo_8l_msgs/msg/point2_df__struct.h"

/// Initialize msg/Point2Df message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ibeo_8l_msgs__msg__Point2Df
 * )) before or use
 * ibeo_8l_msgs__msg__Point2Df__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ibeo_8l_msgs
bool
ibeo_8l_msgs__msg__Point2Df__init(ibeo_8l_msgs__msg__Point2Df * msg);

/// Finalize msg/Point2Df message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ibeo_8l_msgs
void
ibeo_8l_msgs__msg__Point2Df__fini(ibeo_8l_msgs__msg__Point2Df * msg);

/// Create msg/Point2Df message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ibeo_8l_msgs__msg__Point2Df__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ibeo_8l_msgs
ibeo_8l_msgs__msg__Point2Df *
ibeo_8l_msgs__msg__Point2Df__create();

/// Destroy msg/Point2Df message.
/**
 * It calls
 * ibeo_8l_msgs__msg__Point2Df__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ibeo_8l_msgs
void
ibeo_8l_msgs__msg__Point2Df__destroy(ibeo_8l_msgs__msg__Point2Df * msg);


/// Initialize array of msg/Point2Df messages.
/**
 * It allocates the memory for the number of elements and calls
 * ibeo_8l_msgs__msg__Point2Df__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ibeo_8l_msgs
bool
ibeo_8l_msgs__msg__Point2Df__Sequence__init(ibeo_8l_msgs__msg__Point2Df__Sequence * array, size_t size);

/// Finalize array of msg/Point2Df messages.
/**
 * It calls
 * ibeo_8l_msgs__msg__Point2Df__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ibeo_8l_msgs
void
ibeo_8l_msgs__msg__Point2Df__Sequence__fini(ibeo_8l_msgs__msg__Point2Df__Sequence * array);

/// Create array of msg/Point2Df messages.
/**
 * It allocates the memory for the array and calls
 * ibeo_8l_msgs__msg__Point2Df__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ibeo_8l_msgs
ibeo_8l_msgs__msg__Point2Df__Sequence *
ibeo_8l_msgs__msg__Point2Df__Sequence__create(size_t size);

/// Destroy array of msg/Point2Df messages.
/**
 * It calls
 * ibeo_8l_msgs__msg__Point2Df__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ibeo_8l_msgs
void
ibeo_8l_msgs__msg__Point2Df__Sequence__destroy(ibeo_8l_msgs__msg__Point2Df__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // IBEO_8L_MSGS__MSG__POINT2_DF__FUNCTIONS_H_
