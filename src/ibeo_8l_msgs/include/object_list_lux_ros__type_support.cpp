// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ibeo_8l_msgs:msg/ObjectListLuxRos.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ibeo_8l_msgs/msg/object_list_lux_ros__struct.hpp"
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

size_t size_function__ObjectListLuxRos__object_list(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ibeo_8l_msgs::msg::ObjectLuxRos> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ObjectListLuxRos__object_list(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ibeo_8l_msgs::msg::ObjectLuxRos> *>(untyped_member);
  return &member[index];
}

void * get_function__ObjectListLuxRos__object_list(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ibeo_8l_msgs::msg::ObjectLuxRos> *>(untyped_member);
  return &member[index];
}

void resize_function__ObjectListLuxRos__object_list(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ibeo_8l_msgs::msg::ObjectLuxRos> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ObjectListLuxRos_message_member_array[4] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectListLuxRos, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "scan_start_timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectListLuxRos, scan_start_timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "number_of_objects",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectListLuxRos, number_of_objects),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_list",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ibeo_8l_msgs::msg::ObjectLuxRos>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs::msg::ObjectListLuxRos, object_list),  // bytes offset in struct
    nullptr,  // default value
    size_function__ObjectListLuxRos__object_list,  // size() function pointer
    get_const_function__ObjectListLuxRos__object_list,  // get_const(index) function pointer
    get_function__ObjectListLuxRos__object_list,  // get(index) function pointer
    resize_function__ObjectListLuxRos__object_list  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ObjectListLuxRos_message_members = {
  "ibeo_8l_msgs::msg",  // message namespace
  "ObjectListLuxRos",  // message name
  4,  // number of fields
  sizeof(ibeo_8l_msgs::msg::ObjectListLuxRos),
  ObjectListLuxRos_message_member_array  // message members
};

static const rosidl_message_type_support_t ObjectListLuxRos_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ObjectListLuxRos_message_members,
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
get_message_type_support_handle<ibeo_8l_msgs::msg::ObjectListLuxRos>()
{
  return &::ibeo_8l_msgs::msg::rosidl_typesupport_introspection_cpp::ObjectListLuxRos_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ibeo_8l_msgs, msg, ObjectListLuxRos)() {
  return &::ibeo_8l_msgs::msg::rosidl_typesupport_introspection_cpp::ObjectListLuxRos_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
