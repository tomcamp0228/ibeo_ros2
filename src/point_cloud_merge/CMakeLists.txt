cmake_minimum_required(VERSION 3.5)
project(point_cloud_merge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include
  ${RCLCPP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${ibeo_8l_msgs_INCLUDE_DIRS}
)

add_executable(point_cloud_merge
  src/point_cloud_merge.cpp
)
target_link_libraries(point_cloud_merge
  ${PCL_LIBRARIES}
)

ament_target_dependencies(
  point_cloud_merge
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

install(TARGETS point_cloud_merge
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
