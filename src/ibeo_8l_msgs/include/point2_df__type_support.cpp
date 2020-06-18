// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ibeo_8l_msgs:msg/Point2Df.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ibeo_8l_msgs/msg/point2_df__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ibeo_8l_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

static const ::rosidl_typesupport_introspection_cpp::MessageMember Point2Df_message_member_array[2] = {
  {
    "x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::Point2Df, x),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::Point2Df, y),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Point2Df_message_members = {
  "ibeo_8l_msgs::msg",  // message namespace
  "Point2Df",  // message name
  2,  // number of fields
  sizeof(ibeo_8l_msgs::msg::Point2Df),
  Point2Df_message_member_array  // message members
};

static const rosidl_message_type_support_t Point2Df_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Point2Df_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ibeo_8l_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ibeo_8l_msgs::msg::Point2Df>()
{
  return &::ibeo_8l_msgs::msg::rosidl_typesupport_introspection_cpp::Point2Df_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ibeo_8l_msgs, msg, Point2Df)() {
  return &::ibeo_8l_msgs::msg::rosidl_typesupport_introspection_cpp::Point2Df_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
