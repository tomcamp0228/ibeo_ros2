// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ibeo_8l_msgs:msg/Point2Df.idl
// generated code does not contain a copyright notice

#ifndef IBEO_8L_MSGS__MSG__POINT2_DF__TRAITS_HPP_
#define IBEO_8L_MSGS__MSG__POINT2_DF__TRAITS_HPP_

#include "ibeo_8l_msgs/msg/point2_df__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ibeo_8l_msgs::msg::Point2Df>()
{
  return "ibeo_8l_msgs::msg::Point2Df";
}

template<>
struct has_fixed_size<ibeo_8l_msgs::msg::Point2Df>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ibeo_8l_msgs::msg::Point2Df>
  : std::integral_constant<bool, true> {};

}  // namespace rosidl_generator_traits

#endif  // IBEO_8L_MSGS__MSG__POINT2_DF__TRAITS_HPP_
