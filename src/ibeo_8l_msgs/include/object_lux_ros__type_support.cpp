// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ibeo_8l_msgs:msg/ObjectLuxRos.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ibeo_8l_msgs/msg/object_lux_ros__struct.hpp"
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

size_t size_function__ObjectLuxRos__contour_point_list(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ibeo_8l_msgs::msg::Point2Df> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ObjectLuxRos__contour_point_list(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ibeo_8l_msgs::msg::Point2Df> *>(untyped_member);
  return &member[index];
}

void * get_function__ObjectLuxRos__contour_point_list(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ibeo_8l_msgs::msg::Point2Df> *>(untyped_member);
  return &member[index];
}

void resize_function__ObjectLuxRos__contour_point_list(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ibeo_8l_msgs::msg::Point2Df> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ObjectLuxRos_message_member_array[20] = {
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "age",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, age),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "classification",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, classification),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "classification_certainty",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, classification_certainty),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "classification_age",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, classification_age),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "prediction_age",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, prediction_age),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounding_box_center",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ibeo_8l_msgs::msg::Point2Df>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, bounding_box_center),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounding_box_size",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ibeo_8l_msgs::msg::Point2Df>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, bounding_box_size),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_box_center",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ibeo_8l_msgs::msg::Point2Df>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, object_box_center),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_box_size",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ibeo_8l_msgs::msg::Point2Df>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, object_box_size),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_box_orientation",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, object_box_orientation),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reference_point",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ibeo_8l_msgs::msg::Point2Df>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, reference_point),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reference_point_sigma",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ibeo_8l_msgs::msg::Point2Df>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, reference_point_sigma),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "relative_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ibeo_8l_msgs::msg::Point2Df>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, relative_velocity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "absolute_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ibeo_8l_msgs::msg::Point2Df>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, absolute_velocity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "absolute_velocity_sigma",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ibeo_8l_msgs::msg::Point2Df>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, absolute_velocity_sigma),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "number_of_contour_points",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, number_of_contour_points),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "closest_point",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ibeo_8l_msgs::msg::Point2Df>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, closest_point),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "contour_point_list",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ibeo_8l_msgs::msg::Point2Df>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectLuxRos, contour_point_list),  // bytes offset in struct
    nullptr,  // default value
    size_function__ObjectLuxRos__contour_point_list,  // size() function pointer
    get_const_function__ObjectLuxRos__contour_point_list,  // get_const(index) function pointer
    get_function__ObjectLuxRos__contour_point_list,  // get(index) function pointer
    resize_function__ObjectLuxRos__contour_point_list  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ObjectLuxRos_message_members = {
  "ibeo_8l_msgs::msg",  // message namespace
  "ObjectLuxRos",  // message name
  20,  // number of fields
  sizeof(ibeo_8l_msgs::msg::ObjectLuxRos),
  ObjectLuxRos_message_member_array  // message members
};

static const rosidl_message_type_support_t ObjectLuxRos_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ObjectLuxRos_message_members,
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
get_message_type_support_handle<ibeo_8l_msgs::msg::ObjectLuxRos>()
{
  return &::ibeo_8l_msgs::msg::rosidl_typesupport_introspection_cpp::ObjectLuxRos_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ibeo_8l_msgs, msg, ObjectLuxRos)() {
  return &::ibeo_8l_msgs::msg::rosidl_typesupport_introspection_cpp::ObjectLuxRos_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
