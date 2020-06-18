cmake_minimum_required(VERSION 3.5)
project(ibeo_8l_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)





## Generate messages in the 'msg' folder
set(msg_files 
  "msg/Point2Df.msg"
  "msg/ObjectLuxRos.msg"
  "msg/ObjectListLuxRos.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
  
)

ament_package()