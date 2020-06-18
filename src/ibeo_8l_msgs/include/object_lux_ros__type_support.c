// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ibeo_8l_msgs:msg/ObjectLuxRos.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ibeo_8l_msgs/msg/object_lux_ros__rosidl_typesupport_introspection_c.h"
#include "ibeo_8l_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ibeo_8l_msgs/msg/object_lux_ros__struct.h"


// Include directives for member types
// Member `bounding_box_center`
// Member `bounding_box_size`
// Member `object_box_center`
// Member `object_box_size`
// Member `reference_point`
// Member `reference_point_sigma`
// Member `relative_velocity`
// Member `absolute_velocity`
// Member `absolute_velocity_sigma`
// Member `closest_point`
// Member `contour_point_list`
#include "ibeo_8l_msgs/msg/point2_df.h"
// Member `bounding_box_center`
// Member `bounding_box_size`
// Member `object_box_center`
// Member `object_box_size`
// Member `reference_point`
// Member `reference_point_sigma`
// Member `relative_velocity`
// Member `absolute_velocity`
// Member `absolute_velocity_sigma`
// Member `closest_point`
// Member `contour_point_list`
#include "ibeo_8l_msgs/msg/point2_df__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

size_t ObjectLuxRos__rosidl_typesupport_introspection_c__size_function__Point2Df__contour_point_list(
  const void * untyped_member)
{
  const ibeo_8l_msgs__msg__Point2Df__Sequence * member =
    (const ibeo_8l_msgs__msg__Point2Df__Sequence *)(untyped_member);
  return member->size;
}

const void * ObjectLuxRos__rosidl_typesupport_introspection_c__get_const_function__Point2Df__contour_point_list(
  const void * untyped_member, size_t index)
{
  const ibeo_8l_msgs__msg__Point2Df__Sequence * member =
    (const ibeo_8l_msgs__msg__Point2Df__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ObjectLuxRos__rosidl_typesupport_introspection_c__get_function__Point2Df__contour_point_list(
  void * untyped_member, size_t index)
{
  ibeo_8l_msgs__msg__Point2Df__Sequence * member =
    (ibeo_8l_msgs__msg__Point2Df__Sequence *)(untyped_member);
  return &member->data[index];
}

bool ObjectLuxRos__rosidl_typesupport_introspection_c__resize_function__Point2Df__contour_point_list(
  void * untyped_member, size_t size)
{
  ibeo_8l_msgs__msg__Point2Df__Sequence * member =
    (ibeo_8l_msgs__msg__Point2Df__Sequence *)(untyped_member);
  ibeo_8l_msgs__msg__Point2Df__Sequence__fini(member);
  return ibeo_8l_msgs__msg__Point2Df__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_member_array[20] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "age",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, age),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "classification",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, classification),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "classification_certainty",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, classification_certainty),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "classification_age",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, classification_age),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "prediction_age",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, prediction_age),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounding_box_center",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, bounding_box_center),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounding_box_size",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, bounding_box_size),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_box_center",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, object_box_center),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_box_size",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, object_box_size),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_box_orientation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, object_box_orientation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reference_point",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, reference_point),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reference_point_sigma",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, reference_point_sigma),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "relative_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, relative_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "absolute_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, absolute_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "absolute_velocity_sigma",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, absolute_velocity_sigma),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "number_of_contour_points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, number_of_contour_points),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "closest_point",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, closest_point),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "contour_point_list",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ibeo_8l_msgs__msg__ObjectLuxRos, contour_point_list),  // bytes offset in struct
    NULL,  // default value
    ObjectLuxRos__rosidl_typesupport_introspection_c__size_function__Point2Df__contour_point_list,  // size() function pointer
    ObjectLuxRos__rosidl_typesupport_introspection_c__get_const_function__Point2Df__contour_point_list,  // get_const(index) function pointer
    ObjectLuxRos__rosidl_typesupport_introspection_c__get_function__Point2Df__contour_point_list,  // get(index) function pointer
    ObjectLuxRos__rosidl_typesupport_introspection_c__resize_function__Point2Df__contour_point_list  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_members = {
  "ibeo_8l_msgs__msg",  // message namespace
  "ObjectLuxRos",  // message name
  20,  // number of fields
  sizeof(ibeo_8l_msgs__msg__ObjectLuxRos),
  ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_member_array  // message members
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_type_support_handle = {
  0,
  &ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ibeo_8l_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ibeo_8l_msgs, msg, ObjectLuxRos)() {
  ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ibeo_8l_msgs, msg, Point2Df)();
  ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ibeo_8l_msgs, msg, Point2Df)();
  ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_member_array[9].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ibeo_8l_msgs, msg, Point2Df)();
  ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_member_array[10].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ibeo_8l_msgs, msg, Point2Df)();
  ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_member_array[12].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ibeo_8l_msgs, msg, Point2Df)();
  ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_member_array[13].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ibeo_8l_msgs, msg, Point2Df)();
  ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_member_array[14].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ibeo_8l_msgs, msg, Point2Df)();
  ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_member_array[15].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ibeo_8l_msgs, msg, Point2Df)();
  ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_member_array[16].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ibeo_8l_msgs, msg, Point2Df)();
  ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_member_array[18].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ibeo_8l_msgs, msg, Point2Df)();
  ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_member_array[19].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ibeo_8l_msgs, msg, Point2Df)();
  if (!ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_type_support_handle.typesupport_identifier) {
    ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ObjectLuxRos__rosidl_typesupport_introspection_c__ObjectLuxRos_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
