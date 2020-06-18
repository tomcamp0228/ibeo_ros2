// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ibeo_8l_msgs:msg/ObjectListLuxRos.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ibeo_8l_msgs/msg/object_list_lux_ros__rosidl_typesupport_introspection_c.h"
#include "ibeo_8l_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ibeo_8l_msgs/msg/object_list_lux_ros__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/header__rosidl_typesupport_introspection_c.h"
// Member `object_list`
#include "ibeo_8l_msgs/msg/object_lux_ros.h"
// Member `object_list`
#include "ibeo_8l_msgs/msg/object_lux_ros__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

size_t ObjectListLuxRos__rosidl_typesupport_introspection_c__size_function__ObjectLuxRos__object_list(
  const void * untyped_member)
{
  const ibeo_8l_msgs__msg__ObjectLuxRos__Sequence * member =
    (const ibeo_8l_msgs__msg__ObjectLuxRos__Sequence *)(untyped_member);
  return member->size;
}

const void * ObjectListLuxRos__rosidl_typesupport_introspection_c__get_const_function__ObjectLuxRos__object_list(
  const void * untyped_member, size_t index)
{
  const ibeo_8l_msgs__msg__ObjectLuxRos__Sequence * member =
    (const ibeo_8l_msgs__msg__ObjectLuxRos__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ObjectListLuxRos__rosidl_typesupport_introspection_c__get_function__ObjectLuxRos__object_list(
  void * untyped_member, size_t index)
{
  ibeo_8l_msgs__msg__ObjectLuxRos__Sequence * member =
    (ibeo_8l_msgs__msg__ObjectLuxRos__Sequence *)(untyped_member);
  return &member->data[index];
}

bool ObjectListLuxRos__rosidl_typesupport_introspection_c__resize_function__ObjectLuxRos__object_list(
  void * untyped_member, size_t size)
{
  ibeo_8l_msgs__msg__ObjectLuxRos__Sequence * member =
    (ibeo_8l_msgs__msg__ObjectLuxRos__Sequence *)(untyped_member);
  ibeo_8l_msgs__msg__ObjectLuxRos__Sequence__fini(member);
  return ibeo_8l_msgs__msg__ObjectLuxRos__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ObjectListLuxRos__rosidl_typesupport_introspection_c__ObjectListLuxRos_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectListLuxRos, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "scan_start_timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectListLuxRos, scan_start_timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "number_of_objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectListLuxRos, number_of_objects),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_list",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectListLuxRos, object_list),  // bytes offset in struct
    NULL,  // default value
    ObjectListLuxRos__rosidl_typesupport_introspection_c__size_function__ObjectLuxRos__object_list,  // size() function pointer
    ObjectListLuxRos__rosidl_typesupport_introspection_c__get_const_function__ObjectLuxRos__object_list,  // get_const(index) function pointer
    ObjectListLuxRos__rosidl_typesupport_introspection_c__get_function__ObjectLuxRos__object_list,  // get(index) function pointer
    ObjectListLuxRos__rosidl_typesupport_introspection_c__resize_function__ObjectLuxRos__object_list  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ObjectListLuxRos__rosidl_typesupport_introspection_c__ObjectListLuxRos_message_members = {
  "ibeo_8l_msgs__msg",  // message namespace
  "ObjectListLuxRos",  // message name
  4,  // number of fields
  sizeof(ibeo_8l_msgs__msg__ObjectListLuxRos),
  ObjectListLuxRos__rosidl_typesupport_introspection_c__ObjectListLuxRos_message_member_array  // message members
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ObjectListLuxRos__rosidl_typesupport_introspection_c__ObjectListLuxRos_message_type_support_handle = {
  0,
  &ObjectListLuxRos__rosidl_typesupport_introspection_c__ObjectListLuxRos_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ibeo_8l_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ibeo_8l_msgs, msg, ObjectListLuxRos)() {
  ObjectListLuxRos__rosidl_typesupport_introspection_c__ObjectListLuxRos_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ObjectListLuxRos__rosidl_typesupport_introspection_c__ObjectListLuxRos_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ibeo_8l_msgs, msg, ObjectLuxRos)();
  if (!ObjectListLuxRos__rosidl_typesupport_introspection_c__ObjectListLuxRos_message_type_support_handle.typesupport_identifier) {
    ObjectListLuxRos__rosidl_typesupport_introspection_c__ObjectListLuxRos_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ObjectListLuxRos__rosidl_typesupport_introspection_c__ObjectListLuxRos_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
