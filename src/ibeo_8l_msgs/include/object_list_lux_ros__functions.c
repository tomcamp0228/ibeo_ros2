// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ibeo_8l_msgs:msg/ObjectListLuxRos.idl
// generated code does not contain a copyright notice
#include "ibeo_8l_msgs/msg/object_list_lux_ros__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header__functions.h"
// Member `object_list`
#include "ibeo_8l_msgs/msg/object_lux_ros__functions.h"

bool
ibeo_8l_msgs__msg__ObjectListLuxRos__init(ibeo_8l_msgs__msg__ObjectListLuxRos * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ibeo_8l_msgs__msg__ObjectListLuxRos__fini(msg);
    return false;
  }
  // scan_start_timestamp
  // number_of_objects
  // object_list
  if (!ibeo_8l_msgs__msg__ObjectLuxRos__Sequence__init(&msg->object_list, 0)) {
    ibeo_8l_msgs__msg__ObjectListLuxRos__fini(msg);
    return false;
  }
  return true;
}

void
ibeo_8l_msgs__msg__ObjectListLuxRos__fini(ibeo_8l_msgs__msg__ObjectListLuxRos * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // scan_start_timestamp
  // number_of_objects
  // object_list
  ibeo_8l_msgs__msg__ObjectLuxRos__Sequence__fini(&msg->object_list);
}

ibeo_8l_msgs__msg__ObjectListLuxRos *
ibeo_8l_msgs__msg__ObjectListLuxRos__create()
{
  ibeo_8l_msgs__msg__ObjectListLuxRos * msg = (ibeo_8l_msgs__msg__ObjectListLuxRos *)malloc(sizeof(ibeo_8l_msgs__msg__ObjectListLuxRos));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ibeo_8l_msgs__msg__ObjectListLuxRos));
  bool success = ibeo_8l_msgs__msg__ObjectListLuxRos__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
ibeo_8l_msgs__msg__ObjectListLuxRos__destroy(ibeo_8l_msgs__msg__ObjectListLuxRos * msg)
{
  if (msg) {
    ibeo_8l_msgs__msg__ObjectListLuxRos__fini(msg);
  }
  free(msg);
}


bool
ibeo_8l_msgs__msg__ObjectListLuxRos__Sequence__init(ibeo_8l_msgs__msg__ObjectListLuxRos__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  ibeo_8l_msgs__msg__ObjectListLuxRos * data = NULL;
  if (size) {
    data = (ibeo_8l_msgs__msg__ObjectListLuxRos *)calloc(size, sizeof(ibeo_8l_msgs__msg__ObjectListLuxRos));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ibeo_8l_msgs__msg__ObjectListLuxRos__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ibeo_8l_msgs__msg__ObjectListLuxRos__fini(&data[i - 1]);
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
ibeo_8l_msgs__msg__ObjectListLuxRos__Sequence__fini(ibeo_8l_msgs__msg__ObjectListLuxRos__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ibeo_8l_msgs__msg__ObjectListLuxRos__fini(&array->data[i]);
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

ibeo_8l_msgs__msg__ObjectListLuxRos__Sequence *
ibeo_8l_msgs__msg__ObjectListLuxRos__Sequence__create(size_t size)
{
  ibeo_8l_msgs__msg__ObjectListLuxRos__Sequence * array = (ibeo_8l_msgs__msg__ObjectListLuxRos__Sequence *)malloc(sizeof(ibeo_8l_msgs__msg__ObjectListLuxRos__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = ibeo_8l_msgs__msg__ObjectListLuxRos__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
ibeo_8l_msgs__msg__ObjectListLuxRos__Sequence__destroy(ibeo_8l_msgs__msg__ObjectListLuxRos__Sequence * array)
{
  if (array) {
    ibeo_8l_msgs__msg__ObjectListLuxRos__Sequence__fini(array);
  }
  free(array);
}
