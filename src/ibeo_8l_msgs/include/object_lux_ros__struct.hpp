// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ibeo_8l_msgs:msg/ObjectLuxRos.idl
// generated code does not contain a copyright notice

#ifndef IBEO_8L_MSGS__MSG__OBJECT_LUX_ROS__STRUCT_HPP_
#define IBEO_8L_MSGS__MSG__OBJECT_LUX_ROS__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

// Protect against ERROR being predefined on Windows, in case somebody defines a
// constant by that name.
#if defined(_WIN32)
  #if defined(ERROR)
    #undef ERROR
  #endif
  #if defined(NO_ERROR)
    #undef NO_ERROR
  #endif
#endif

// Include directives for member types
// Member 'bounding_box_center'
// Member 'bounding_box_size'
// Member 'object_box_center'
// Member 'object_box_size'
// Member 'reference_point'
// Member 'reference_point_sigma'
// Member 'relative_velocity'
// Member 'absolute_velocity'
// Member 'absolute_velocity_sigma'
// Member 'closest_point'
// Member 'contour_point_list'
#include "ibeo_8l_msgs/msg/point2_df__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ibeo_8l_msgs__msg__ObjectLuxRos __attribute__((deprecated))
#else
# define DEPRECATED__ibeo_8l_msgs__msg__ObjectLuxRos __declspec(deprecated)
#endif

namespace ibeo_8l_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectLuxRos_
{
  using Type = ObjectLuxRos_<ContainerAllocator>;

  explicit ObjectLuxRos_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : bounding_box_center(_init),
    bounding_box_size(_init),
    object_box_center(_init),
    object_box_size(_init),
    reference_point(_init),
    reference_point_sigma(_init),
    relative_velocity(_init),
    absolute_velocity(_init),
    absolute_velocity_sigma(_init),
    closest_point(_init)
  {
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->age = 0ul;
      this->timestamp = 0.0f;
      this->classification = 0;
      this->classification_certainty = 0;
      this->classification_age = 0ul;
      this->prediction_age = 0;
      this->object_box_orientation = 0.0f;
      this->number_of_contour_points = 0;
    }
  }

  explicit ObjectLuxRos_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : bounding_box_center(_alloc, _init),
    bounding_box_size(_alloc, _init),
    object_box_center(_alloc, _init),
    object_box_size(_alloc, _init),
    reference_point(_alloc, _init),
    reference_point_sigma(_alloc, _init),
    relative_velocity(_alloc, _init),
    absolute_velocity(_alloc, _init),
    absolute_velocity_sigma(_alloc, _init),
    closest_point(_alloc, _init)
  {
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->age = 0ul;
      this->timestamp = 0.0f;
      this->classification = 0;
      this->classification_certainty = 0;
      this->classification_age = 0ul;
      this->prediction_age = 0;
      this->object_box_orientation = 0.0f;
      this->number_of_contour_points = 0;
    }
  }

  // field types and members
  using _id_type =
    uint16_t;
  _id_type id;
  using _age_type =
    uint32_t;
  _age_type age;
  using _timestamp_type =
    float;
  _timestamp_type timestamp;
  using _classification_type =
    uint8_t;
  _classification_type classification;
  using _classification_certainty_type =
    uint8_t;
  _classification_certainty_type classification_certainty;
  using _classification_age_type =
    uint32_t;
  _classification_age_type classification_age;
  using _prediction_age_type =
    uint16_t;
  _prediction_age_type prediction_age;
  using _bounding_box_center_type =
    ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>;
  _bounding_box_center_type bounding_box_center;
  using _bounding_box_size_type =
    ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>;
  _bounding_box_size_type bounding_box_size;
  using _object_box_center_type =
    ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>;
  _object_box_center_type object_box_center;
  using _object_box_size_type =
    ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>;
  _object_box_size_type object_box_size;
  using _object_box_orientation_type =
    float;
  _object_box_orientation_type object_box_orientation;
  using _reference_point_type =
    ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>;
  _reference_point_type reference_point;
  using _reference_point_sigma_type =
    ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>;
  _reference_point_sigma_type reference_point_sigma;
  using _relative_velocity_type =
    ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>;
  _relative_velocity_type relative_velocity;
  using _absolute_velocity_type =
    ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>;
  _absolute_velocity_type absolute_velocity;
  using _absolute_velocity_sigma_type =
    ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>;
  _absolute_velocity_sigma_type absolute_velocity_sigma;
  using _number_of_contour_points_type =
    uint8_t;
  _number_of_contour_points_type number_of_contour_points;
  using _closest_point_type =
    ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>;
  _closest_point_type closest_point;
  using _contour_point_list_type =
    std::vector<ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>, typename ContainerAllocator::template rebind<ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>>::other>;
  _contour_point_list_type contour_point_list;

  // setters for named parameter idiom
  Type & set__id(
    const uint16_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__age(
    const uint32_t & _arg)
  {
    this->age = _arg;
    return *this;
  }
  Type & set__timestamp(
    const float & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__classification(
    const uint8_t & _arg)
  {
    this->classification = _arg;
    return *this;
  }
  Type & set__classification_certainty(
    const uint8_t & _arg)
  {
    this->classification_certainty = _arg;
    return *this;
  }
  Type & set__classification_age(
    const uint32_t & _arg)
  {
    this->classification_age = _arg;
    return *this;
  }
  Type & set__prediction_age(
    const uint16_t & _arg)
  {
    this->prediction_age = _arg;
    return *this;
  }
  Type & set__bounding_box_center(
    const ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> & _arg)
  {
    this->bounding_box_center = _arg;
    return *this;
  }
  Type & set__bounding_box_size(
    const ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> & _arg)
  {
    this->bounding_box_size = _arg;
    return *this;
  }
  Type & set__object_box_center(
    const ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> & _arg)
  {
    this->object_box_center = _arg;
    return *this;
  }
  Type & set__object_box_size(
    const ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> & _arg)
  {
    this->object_box_size = _arg;
    return *this;
  }
  Type & set__object_box_orientation(
    const float & _arg)
  {
    this->object_box_orientation = _arg;
    return *this;
  }
  Type & set__reference_point(
    const ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> & _arg)
  {
    this->reference_point = _arg;
    return *this;
  }
  Type & set__reference_point_sigma(
    const ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> & _arg)
  {
    this->reference_point_sigma = _arg;
    return *this;
  }
  Type & set__relative_velocity(
    const ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> & _arg)
  {
    this->relative_velocity = _arg;
    return *this;
  }
  Type & set__absolute_velocity(
    const ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> & _arg)
  {
    this->absolute_velocity = _arg;
    return *this;
  }
  Type & set__absolute_velocity_sigma(
    const ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> & _arg)
  {
    this->absolute_velocity_sigma = _arg;
    return *this;
  }
  Type & set__number_of_contour_points(
    const uint8_t & _arg)
  {
    this->number_of_contour_points = _arg;
    return *this;
  }
  Type & set__closest_point(
    const ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> & _arg)
  {
    this->closest_point = _arg;
    return *this;
  }
  Type & set__contour_point_list(
    const std::vector<ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>, typename ContainerAllocator::template rebind<ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>>::other> & _arg)
  {
    this->contour_point_list = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t UNCLASSIFIED =
    0u;
  static constexpr uint8_t UNKNOWN_SMALL =
    1u;
  static constexpr uint8_t UNKNOWN_BIG =
    2u;
  static constexpr uint8_t PEDESTRIAN =
    3u;
  static constexpr uint8_t BIKE =
    4u;
  static constexpr uint8_t CAR =
    5u;
  static constexpr uint8_t TRUCK =
    6u;
  static constexpr uint8_t BICYCLE =
    12u;

  // pointer types
  using RawPtr =
    ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator> *;
  using ConstRawPtr =
    const ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ibeo_8l_msgs__msg__ObjectLuxRos
    std::shared_ptr<ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ibeo_8l_msgs__msg__ObjectLuxRos
    std::shared_ptr<ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectLuxRos_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->age != other.age) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->classification != other.classification) {
      return false;
    }
    if (this->classification_certainty != other.classification_certainty) {
      return false;
    }
    if (this->classification_age != other.classification_age) {
      return false;
    }
    if (this->prediction_age != other.prediction_age) {
      return false;
    }
    if (this->bounding_box_center != other.bounding_box_center) {
      return false;
    }
    if (this->bounding_box_size != other.bounding_box_size) {
      return false;
    }
    if (this->object_box_center != other.object_box_center) {
      return false;
    }
    if (this->object_box_size != other.object_box_size) {
      return false;
    }
    if (this->object_box_orientation != other.object_box_orientation) {
      return false;
    }
    if (this->reference_point != other.reference_point) {
      return false;
    }
    if (this->reference_point_sigma != other.reference_point_sigma) {
      return false;
    }
    if (this->relative_velocity != other.relative_velocity) {
      return false;
    }
    if (this->absolute_velocity != other.absolute_velocity) {
      return false;
    }
    if (this->absolute_velocity_sigma != other.absolute_velocity_sigma) {
      return false;
    }
    if (this->number_of_contour_points != other.number_of_contour_points) {
      return false;
    }
    if (this->closest_point != other.closest_point) {
      return false;
    }
    if (this->contour_point_list != other.contour_point_list) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectLuxRos_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectLuxRos_

// alias to use template instance with default allocator
using ObjectLuxRos =
  ibeo_8l_msgs::msg::ObjectLuxRos_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t ObjectLuxRos_<ContainerAllocator>::UNCLASSIFIED;
template<typename ContainerAllocator>
constexpr uint8_t ObjectLuxRos_<ContainerAllocator>::UNKNOWN_SMALL;
template<typename ContainerAllocator>
constexpr uint8_t ObjectLuxRos_<ContainerAllocator>::UNKNOWN_BIG;
template<typename ContainerAllocator>
constexpr uint8_t ObjectLuxRos_<ContainerAllocator>::PEDESTRIAN;
template<typename ContainerAllocator>
constexpr uint8_t ObjectLuxRos_<ContainerAllocator>::BIKE;
template<typename ContainerAllocator>
constexpr uint8_t ObjectLuxRos_<ContainerAllocator>::CAR;
template<typename ContainerAllocator>
constexpr uint8_t ObjectLuxRos_<ContainerAllocator>::TRUCK;
template<typename ContainerAllocator>
constexpr uint8_t ObjectLuxRos_<ContainerAllocator>::BICYCLE;

}  // namespace msg

}  // namespace ibeo_8l_msgs

#endif  // IBEO_8L_MSGS__MSG__OBJECT_LUX_ROS__STRUCT_HPP_
