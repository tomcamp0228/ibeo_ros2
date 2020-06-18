// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ibeo_8l_msgs:msg/ObjectListLuxRos.idl
// generated code does not contain a copyright notice

#ifndef IBEO_8L_MSGS__MSG__OBJECT_LIST_LUX_ROS__STRUCT_H_
#define IBEO_8L_MSGS__MSG__OBJECT_LIST_LUX_ROS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/header__struct.h"
// Member 'object_list'
#include "ibeo_8l_msgs/msg/object_lux_ros__struct.h"

// Struct defined in msg/ObjectListLuxRos in the package ibeo_8l_msgs.
typedef struct ibeo_8l_msgs__msg__ObjectListLuxRos
{
  std_msgs__msg__Header header;
  float scan_start_timestamp;
  uint16_t number_of_objects;
  ibeo_8l_msgs__msg__ObjectLuxRos__Sequence object_list;
} ibeo_8l_msgs__msg__ObjectListLuxRos;

// Struct for a sequence of ibeo_8l_msgs__msg__ObjectListLuxRos.
typedef struct ibeo_8l_msgs__msg__ObjectListLuxRos__Sequence
{
  ibeo_8l_msgs__msg__ObjectListLuxRos * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ibeo_8l_msgs__msg__ObjectListLuxRos__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // IBEO_8L_MSGS__MSG__OBJECT_LIST_LUX_ROS__STRUCT_H_
