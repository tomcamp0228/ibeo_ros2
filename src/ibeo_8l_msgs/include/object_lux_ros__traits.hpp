// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ibeo_8l_msgs:msg/ObjectLuxRos.idl
// generated code does not contain a copyright notice

#ifndef IBEO_8L_MSGS__MSG__OBJECT_LUX_ROS__TRAITS_HPP_
#define IBEO_8L_MSGS__MSG__OBJECT_LUX_ROS__TRAITS_HPP_

#include "ibeo_8l_msgs/msg/object_lux_ros__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

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
#include "ibeo_8l_msgs/msg/point2_df__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ibeo_8l_msgs::msg::ObjectLuxRos>()
{
  return "ibeo_8l_msgs::msg::ObjectLuxRos";
}

template<>
struct has_fixed_size<ibeo_8l_msgs::msg::ObjectLuxRos>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ibeo_8l_msgs::msg::ObjectLuxRos>
  : std::integral_constant<bool, false> {};

}  // namespace rosidl_generator_traits

#endif  // IBEO_8L_MSGS__MSG__OBJECT_LUX_ROS__TRAITS_HPP_
