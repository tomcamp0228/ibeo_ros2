// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ibeo_8l_msgs:msg/ObjectLuxRos.idl
// generated code does not contain a copyright notice

#ifndef IBEO_8L_MSGS__MSG__OBJECT_LUX_ROS__STRUCT_H_
#define IBEO_8L_MSGS__MSG__OBJECT_LUX_ROS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'UNCLASSIFIED'.
enum
{
  ibeo_8l_msgs__msg__ObjectLuxRos__UNCLASSIFIED = 0
};

/// Constant 'UNKNOWN_SMALL'.
enum
{
  ibeo_8l_msgs__msg__ObjectLuxRos__UNKNOWN_SMALL = 1
};

/// Constant 'UNKNOWN_BIG'.
enum
{
  ibeo_8l_msgs__msg__ObjectLuxRos__UNKNOWN_BIG = 2
};

/// Constant 'PEDESTRIAN'.
enum
{
  ibeo_8l_msgs__msg__ObjectLuxRos__PEDESTRIAN = 3
};

/// Constant 'BIKE'.
enum
{
  ibeo_8l_msgs__msg__ObjectLuxRos__BIKE = 4
};

/// Constant 'CAR'.
enum
{
  ibeo_8l_msgs__msg__ObjectLuxRos__CAR = 5
};

/// Constant 'TRUCK'.
enum
{
  ibeo_8l_msgs__msg__ObjectLuxRos__TRUCK = 6
};

/// Constant 'BICYCLE'.
enum
{
  ibeo_8l_msgs__msg__ObjectLuxRos__BICYCLE = 12
};

// Include directives for member types
// Member 'bounding_box_center'
// Member 'bounding_box_size'
// Member 'object_box_center'
// Member 'object_box_size'
// Member 'reference_point'
// Member 'reference_point_sigma'
// Member 'relative_velocity'
// Member 'absolute_velocity'
// Member 'absolute_velocity_sigma'
// Member 'closest_point'
// Member 'contour_point_list'
#include "ibeo_8l_msgs/msg/point2_df__struct.h"

// Struct defined in msg/ObjectLuxRos in the package ibeo_8l_msgs.
typedef struct ibeo_8l_msgs__msg__ObjectLuxRos
{
  uint16_t id;
  uint32_t age;
  float timestamp;
  uint8_t classification;
  uint8_t classification_certainty;
  uint32_t classification_age;
  uint16_t prediction_age;
  ibeo_8l_msgs__msg__Point2Df bounding_box_center;
  ibeo_8l_msgs__msg__Point2Df bounding_box_size;
  ibeo_8l_msgs__msg__Point2Df object_box_center;
  ibeo_8l_msgs__msg__Point2Df object_box_size;
  float object_box_orientation;
  ibeo_8l_msgs__msg__Point2Df reference_point;
  ibeo_8l_msgs__msg__Point2Df reference_point_sigma;
  ibeo_8l_msgs__msg__Point2Df relative_velocity;
  ibeo_8l_msgs__msg__Point2Df absolute_velocity;
  ibeo_8l_msgs__msg__Point2Df absolute_velocity_sigma;
  uint8_t number_of_contour_points;
  ibeo_8l_msgs__msg__Point2Df closest_point;
  ibeo_8l_msgs__msg__Point2Df__Sequence contour_point_list;
} ibeo_8l_msgs__msg__ObjectLuxRos;

// Struct for a sequence of ibeo_8l_msgs__msg__ObjectLuxRos.
typedef struct ibeo_8l_msgs__msg__ObjectLuxRos__Sequence
{
  ibeo_8l_msgs__msg__ObjectLuxRos * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ibeo_8l_msgs__msg__ObjectLuxRos__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // IBEO_8L_MSGS__MSG__OBJECT_LUX_ROS__STRUCT_H_
