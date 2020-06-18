// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ibeo_8l_msgs:msg/ObjectLuxRos.idl
// generated code does not contain a copyright notice
#include "ibeo_8l_msgs/msg/object_lux_ros__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


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
#include "ibeo_8l_msgs/msg/point2_df__functions.h"

bool
ibeo_8l_msgs__msg__ObjectLuxRos__init(ibeo_8l_msgs__msg__ObjectLuxRos * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // age
  // timestamp
  // classification
  // classification_certainty
  // classification_age
  // prediction_age
  // bounding_box_center
  if (!ibeo_8l_msgs__msg__Point2Df__init(&msg->bounding_box_center)) {
    ibeo_8l_msgs__msg__ObjectLuxRos__fini(msg);
    return false;
  }
  // bounding_box_size
  if (!ibeo_8l_msgs__msg__Point2Df__init(&msg->bounding_box_size)) {
    ibeo_8l_msgs__msg__ObjectLuxRos__fini(msg);
    return false;
  }
  // object_box_center
  if (!ibeo_8l_msgs__msg__Point2Df__init(&msg->object_box_center)) {
    ibeo_8l_msgs__msg__ObjectLuxRos__fini(msg);
    return false;
  }
  // object_box_size
  if (!ibeo_8l_msgs__msg__Point2Df__init(&msg->object_box_size)) {
    ibeo_8l_msgs__msg__ObjectLuxRos__fini(msg);
    return false;
  }
  // object_box_orientation
  // reference_point
  if (!ibeo_8l_msgs__msg__Point2Df__init(&msg->reference_point)) {
    ibeo_8l_msgs__msg__ObjectLuxRos__fini(msg);
    return false;
  }
  // reference_point_sigma
  if (!ibeo_8l_msgs__msg__Point2Df__init(&msg->reference_point_sigma)) {
    ibeo_8l_msgs__msg__ObjectLuxRos__fini(msg);
    return false;
  }
  // relative_velocity
  if (!ibeo_8l_msgs__msg__Point2Df__init(&msg->relative_velocity)) {
    ibeo_8l_msgs__msg__ObjectLuxRos__fini(msg);
    return false;
  }
  // absolute_velocity
  if (!ibeo_8l_msgs__msg__Point2Df__init(&msg->absolute_velocity)) {
    ibeo_8l_msgs__msg__ObjectLuxRos__fini(msg);
    return false;
  }
  // absolute_velocity_sigma
  if (!ibeo_8l_msgs__msg__Point2Df__init(&msg->absolute_velocity_sigma)) {
    ibeo_8l_msgs__msg__ObjectLuxRos__fini(msg);
    return false;
  }
  // number_of_contour_points
  // closest_point
  if (!ibeo_8l_msgs__msg__Point2Df__init(&msg->closest_point)) {
    ibeo_8l_msgs__msg__ObjectLuxRos__fini(msg);
    return false;
  }
  // contour_point_list
  if (!ibeo_8l_msgs__msg__Point2Df__Sequence__init(&msg->contour_point_list, 0)) {
    ibeo_8l_msgs__msg__ObjectLuxRos__fini(msg);
    return false;
  }
  return true;
}

void
ibeo_8l_msgs__msg__ObjectLuxRos__fini(ibeo_8l_msgs__msg__ObjectLuxRos * msg)
{
  if (!msg) {
    return;
  }
  // id
  // age
  // timestamp
  // classification
  // classification_certainty
  // classification_age
  // prediction_age
  // bounding_box_center
  ibeo_8l_msgs__msg__Point2Df__fini(&msg->bounding_box_center);
  // bounding_box_size
  ibeo_8l_msgs__msg__Point2Df__fini(&msg->bounding_box_size);
  // object_box_center
  ibeo_8l_msgs__msg__Point2Df__fini(&msg->object_box_center);
  // object_box_size
  ibeo_8l_msgs__msg__Point2Df__fini(&msg->object_box_size);
  // object_box_orientation
  // reference_point
  ibeo_8l_msgs__msg__Point2Df__fini(&msg->reference_point);
  // reference_point_sigma
  ibeo_8l_msgs__msg__Point2Df__fini(&msg->reference_point_sigma);
  // relative_velocity
  ibeo_8l_msgs__msg__Point2Df__fini(&msg->relative_velocity);
  // absolute_velocity
  ibeo_8l_msgs__msg__Point2Df__fini(&msg->absolute_velocity);
  // absolute_velocity_sigma
  ibeo_8l_msgs__msg__Point2Df__fini(&msg->absolute_velocity_sigma);
  // number_of_contour_points
  // closest_point
  ibeo_8l_msgs__msg__Point2Df__fini(&msg->closest_point);
  // contour_point_list
  ibeo_8l_msgs__msg__Point2Df__Sequence__fini(&msg->contour_point_list);
}

ibeo_8l_msgs__msg__ObjectLuxRos *
ibeo_8l_msgs__msg__ObjectLuxRos__create()
{
  ibeo_8l_msgs__msg__ObjectLuxRos * msg = (ibeo_8l_msgs__msg__ObjectLuxRos *)malloc(sizeof(ibeo_8l_msgs__msg__ObjectLuxRos));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ibeo_8l_msgs__msg__ObjectLuxRos));
  bool success = ibeo_8l_msgs__msg__ObjectLuxRos__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
ibeo_8l_msgs__msg__ObjectLuxRos__destroy(ibeo_8l_msgs__msg__ObjectLuxRos * msg)
{
  if (msg) {
    ibeo_8l_msgs__msg__ObjectLuxRos__fini(msg);
  }
  free(msg);
}


bool
ibeo_8l_msgs__msg__ObjectLuxRos__Sequence__init(ibeo_8l_msgs__msg__ObjectLuxRos__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  ibeo_8l_msgs__msg__ObjectLuxRos * data = NULL;
  if (size) {
    data = (ibeo_8l_msgs__msg__ObjectLuxRos *)calloc(size, sizeof(ibeo_8l_msgs__msg__ObjectLuxRos));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ibeo_8l_msgs__msg__ObjectLuxRos__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ibeo_8l_msgs__msg__ObjectLuxRos__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ibeo_8l_msgs__msg__ObjectLuxRos__Sequence__fini(ibeo_8l_msgs__msg__ObjectLuxRos__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ibeo_8l_msgs__msg__ObjectLuxRos__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ibeo_8l_msgs__msg__ObjectLuxRos__Sequence *
ibeo_8l_msgs__msg__ObjectLuxRos__Sequence__create(size_t size)
{
  ibeo_8l_msgs__msg__ObjectLuxRos__Sequence * array = (ibeo_8l_msgs__msg__ObjectLuxRos__Sequence *)malloc(sizeof(ibeo_8l_msgs__msg__ObjectLuxRos__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = ibeo_8l_msgs__msg__ObjectLuxRos__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
ibeo_8l_msgs__msg__ObjectLuxRos__Sequence__destroy(ibeo_8l_msgs__msg__ObjectLuxRos__Sequence * array)
{
  if (array) {
    ibeo_8l_msgs__msg__ObjectLuxRos__Sequence__fini(array);
  }
  free(array);
}
