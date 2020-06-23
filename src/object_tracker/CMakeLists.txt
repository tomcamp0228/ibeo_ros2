cmake_minimum_required(VERSION 3.5)
project(object_tracker)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   ibeo_8l_msgs
#   roscpp
#   pcl_ros
#   pcl_conversions
# )

# find_package(PCL REQUIRED COMPONENTS
# features
# )

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ibeo_8l_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED) #  COMPONENTS common io
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${RCLCPP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${ibeo_8l_msgs_INCLUDE_DIRS}
)


add_executable(cloud_extraction
  src/cloud_extraction.cpp
)
# add_dependencies(cloud_extraction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cloud_extraction
  # ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(
  cloud_extraction
  "std_msgs"
  "rclcpp"
  "nav_msgs"
  "ibeo_8l_msgs"
  "pcl_conversions"
  "pcl_msgs"
  "PCL_COMMON"
  "PCL_IO"
  "PCL_FEATURES"
  "sensor_msgs"
  "visualization_msgs"
)

add_executable(object_tracker
  src/object_tracker.cpp
)
# add_dependencies(object_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_tracker
  # ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(
  object_tracker
  "std_msgs"
  "rclcpp"
  "nav_msgs"
  "ibeo_8l_msgs"
  "pcl_conversions"
  "pcl_msgs"
  "PCL_COMMON"
  "PCL_IO"
  "PCL_FEATURES"
  "sensor_msgs"
  "visualization_msgs"
)
# add_executable(kalman_test
#   src/kalman_test.cpp
# )
# add_dependencies(kalman_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(kalman_test
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

install(TARGETS object_tracker
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS cloud_extraction
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package(
)