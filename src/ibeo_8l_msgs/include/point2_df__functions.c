// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ibeo_8l_msgs:msg/Point2Df.idl
// generated code does not contain a copyright notice
#include "ibeo_8l_msgs/msg/point2_df__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
ibeo_8l_msgs__msg__Point2Df__init(ibeo_8l_msgs__msg__Point2Df * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  return true;
}

void
ibeo_8l_msgs__msg__Point2Df__fini(ibeo_8l_msgs__msg__Point2Df * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
}

ibeo_8l_msgs__msg__Point2Df *
ibeo_8l_msgs__msg__Point2Df__create()
{
  ibeo_8l_msgs__msg__Point2Df * msg = (ibeo_8l_msgs__msg__Point2Df *)malloc(sizeof(ibeo_8l_msgs__msg__Point2Df));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ibeo_8l_msgs__msg__Point2Df));
  bool success = ibeo_8l_msgs__msg__Point2Df__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
ibeo_8l_msgs__msg__Point2Df__destroy(ibeo_8l_msgs__msg__Point2Df * msg)
{
  if (msg) {
    ibeo_8l_msgs__msg__Point2Df__fini(msg);
  }
  free(msg);
}


bool
ibeo_8l_msgs__msg__Point2Df__Sequence__init(ibeo_8l_msgs__msg__Point2Df__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  ibeo_8l_msgs__msg__Point2Df * data = NULL;
  if (size) {
    data = (ibeo_8l_msgs__msg__Point2Df *)calloc(size, sizeof(ibeo_8l_msgs__msg__Point2Df));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ibeo_8l_msgs__msg__Point2Df__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ibeo_8l_msgs__msg__Point2Df__fini(&data[i - 1]);
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
ibeo_8l_msgs__msg__Point2Df__Sequence__fini(ibeo_8l_msgs__msg__Point2Df__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ibeo_8l_msgs__msg__Point2Df__fini(&array->data[i]);
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

ibeo_8l_msgs__msg__Point2Df__Sequence *
ibeo_8l_msgs__msg__Point2Df__Sequence__create(size_t size)
{
  ibeo_8l_msgs__msg__Point2Df__Sequence * array = (ibeo_8l_msgs__msg__Point2Df__Sequence *)malloc(sizeof(ibeo_8l_msgs__msg__Point2Df__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = ibeo_8l_msgs__msg__Point2Df__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
ibeo_8l_msgs__msg__Point2Df__Sequence__destroy(ibeo_8l_msgs__msg__Point2Df__Sequence * array)
{
  if (array) {
    ibeo_8l_msgs__msg__Point2Df__Sequence__fini(array);
  }
  free(array);
}
