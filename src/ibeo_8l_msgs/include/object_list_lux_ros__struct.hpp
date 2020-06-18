// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ibeo_8l_msgs:msg/ObjectListLuxRos.idl
// generated code does not contain a copyright notice

#ifndef IBEO_8L_MSGS__MSG__OBJECT_LIST_LUX_ROS__STRUCT_HPP_
#define IBEO_8L_MSGS__MSG__OBJECT_LIST_LUX_ROS__STRUCT_HPP_

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
// Member 'header'
#include "std_msgs/msg/header__struct.hpp"
// Member 'object_list'
#include "ibeo_8l_msgs/msg/object_lux_ros__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ibeo_8l_msgs__msg__ObjectListLuxRos __attribute__((deprecated))
#else
# define DEPRECATED__ibeo_8l_msgs__msg__ObjectListLuxRos __declspec(deprecated)
#endif

namespace ibeo_8l_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectListLuxRos_
{
  using Type = ObjectListLuxRos_<ContainerAllocator>;

  explicit ObjectListLuxRos_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->scan_start_timestamp = 0.0f;
      this->number_of_objects = 0;
    }
  }

  explicit ObjectListLuxRos_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->scan_start_timestamp = 0.0f;
      this->number_of_objects = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _scan_start_timestamp_type =
    float;
  _scan_start_timestamp_type scan_start_timestamp;
  using _number_of_objects_type =
    uint16_t;
  _number_of_objects_type number_of_objects;
  using _object_list_type =
    std::vector<ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator>, typename ContainerAllocator::template rebind<ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator>>::other>;
  _object_list_type object_list;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__scan_start_timestamp(
    const float & _arg)
  {
    this->scan_start_timestamp = _arg;
    return *this;
  }
  Type & set__number_of_objects(
    const uint16_t & _arg)
  {
    this->number_of_objects = _arg;
    return *this;
  }
  Type & set__object_list(
    const std::vector<ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator>, typename ContainerAllocator::template rebind<ibeo_8l_msgs::msg::ObjectLuxRos_<ContainerAllocator>>::other> & _arg)
  {
    this->object_list = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ibeo_8l_msgs::msg::ObjectListLuxRos_<ContainerAllocator> *;
  using ConstRawPtr =
    const ibeo_8l_msgs::msg::ObjectListLuxRos_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ibeo_8l_msgs::msg::ObjectListLuxRos_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ibeo_8l_msgs::msg::ObjectListLuxRos_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ibeo_8l_msgs::msg::ObjectListLuxRos_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ibeo_8l_msgs::msg::ObjectListLuxRos_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ibeo_8l_msgs::msg::ObjectListLuxRos_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ibeo_8l_msgs::msg::ObjectListLuxRos_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ibeo_8l_msgs::msg::ObjectListLuxRos_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ibeo_8l_msgs::msg::ObjectListLuxRos_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ibeo_8l_msgs__msg__ObjectListLuxRos
    std::shared_ptr<ibeo_8l_msgs::msg::ObjectListLuxRos_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ibeo_8l_msgs__msg__ObjectListLuxRos
    std::shared_ptr<ibeo_8l_msgs::msg::ObjectListLuxRos_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectListLuxRos_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->scan_start_timestamp != other.scan_start_timestamp) {
      return false;
    }
    if (this->number_of_objects != other.number_of_objects) {
      return false;
    }
    if (this->object_list != other.object_list) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectListLuxRos_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectListLuxRos_

// alias to use template instance with default allocator
using ObjectListLuxRos =
  ibeo_8l_msgs::msg::ObjectListLuxRos_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ibeo_8l_msgs

#endif  // IBEO_8L_MSGS__MSG__OBJECT_LIST_LUX_ROS__STRUCT_HPP_
