// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ibeo_8l_msgs:msg/Point2Df.idl
// generated code does not contain a copyright notice

#ifndef IBEO_8L_MSGS__MSG__POINT2_DF__STRUCT_H_
#define IBEO_8L_MSGS__MSG__POINT2_DF__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Point2Df in the package ibeo_8l_msgs.
typedef struct ibeo_8l_msgs__msg__Point2Df
{
  float x;
  float y;
} ibeo_8l_msgs__msg__Point2Df;

// Struct for a sequence of ibeo_8l_msgs__msg__Point2Df.
typedef struct ibeo_8l_msgs__msg__Point2Df__Sequence
{
  ibeo_8l_msgs__msg__Point2Df * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ibeo_8l_msgs__msg__Point2Df__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // IBEO_8L_MSGS__MSG__POINT2_DF__STRUCT_H_
