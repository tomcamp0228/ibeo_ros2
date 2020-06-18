// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ibeo_8l_msgs:msg/Point2Df.idl
// generated code does not contain a copyright notice

#ifndef IBEO_8L_MSGS__MSG__POINT2_DF__STRUCT_HPP_
#define IBEO_8L_MSGS__MSG__POINT2_DF__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__ibeo_8l_msgs__msg__Point2Df __attribute__((deprecated))
#else
# define DEPRECATED__ibeo_8l_msgs__msg__Point2Df __declspec(deprecated)
#endif

namespace ibeo_8l_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Point2Df_
{
  using Type = Point2Df_<ContainerAllocator>;

  explicit Point2Df_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
    }
  }

  explicit Point2Df_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;

  // setters for named parameter idiom
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> *;
  using ConstRawPtr =
    const ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ibeo_8l_msgs__msg__Point2Df
    std::shared_ptr<ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ibeo_8l_msgs__msg__Point2Df
    std::shared_ptr<ibeo_8l_msgs::msg::Point2Df_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Point2Df_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    return true;
  }
  bool operator!=(const Point2Df_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Point2Df_

// alias to use template instance with default allocator
using Point2Df =
  ibeo_8l_msgs::msg::Point2Df_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ibeo_8l_msgs

#endif  // IBEO_8L_MSGS__MSG__POINT2_DF__STRUCT_HPP_
