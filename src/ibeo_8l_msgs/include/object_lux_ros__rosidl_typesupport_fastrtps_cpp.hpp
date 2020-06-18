// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from ibeo_8l_msgs:msg/ObjectLuxRos.idl
// generated code does not contain a copyright notice

#ifndef IBEO_8L_MSGS__MSG__OBJECT_LUX_ROS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define IBEO_8L_MSGS__MSG__OBJECT_LUX_ROS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "ibeo_8l_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "ibeo_8l_msgs/msg/object_lux_ros__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace ibeo_8l_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ibeo_8l_msgs
cdr_serialize(
  const ibeo_8l_msgs::msg::ObjectLuxRos & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ibeo_8l_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ibeo_8l_msgs::msg::ObjectLuxRos & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ibeo_8l_msgs
get_serialized_size(
  const ibeo_8l_msgs::msg::ObjectLuxRos & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ibeo_8l_msgs
max_serialized_size_ObjectLuxRos(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ibeo_8l_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ibeo_8l_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ibeo_8l_msgs, msg, ObjectLuxRos)();

#ifdef __cplusplus
}
#endif

#endif  // IBEO_8L_MSGS__MSG__OBJECT_LUX_ROS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
