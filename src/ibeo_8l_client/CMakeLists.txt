cmake_minimum_required(VERSION 3.5)
project(ibeo_8l_client)

## Set Boost paths
set(Boost_INCLUDE_DIRS /usr/include/boost)
set(Boost_LIBRARIES_DIRS /usr/lib/x86_64-linux-gnu)

## Set IBEO paths
set (IBEO_DIR1 ${CMAKE_CURRENT_SOURCE_DIR}/../../install/ibeo_8l_sdk/lib/ibeo_8l_sdk)
set (IBEO_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../../build/ibeo_8l_sdk/libibeo_8l_sdk.a)

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


## Find macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ibeo_8l_msgs REQUIRED)
find_package(ibeo_8l_sdk REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED) #  COMPONENTS common io
find_package(Boost REQUIRED COMPONENTS system chrono regex thread program_options date_time)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

###########
## Build ##
###########


link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})



add_executable(ibeo_file_sim src/ibeo_file_sim.cpp)
target_include_directories(ibeo_file_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ibeo_8l_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${RCLCPP_INCLUDE_DIRS}
  ${ibeo_8l_sdk_INCLUDE_DIRS}
)

target_link_libraries(ibeo_file_sim 
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${IBEO_LIB}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

ament_target_dependencies(
  ibeo_file_sim
  "std_msgs"
  "rclcpp"
  "nav_msgs"
  "ibeo_8l_msgs"
  "ibeo_8l_sdk"
  "pcl_conversions"
  "pcl_msgs"
  "PCL_COMMON"
  "PCL_IO"
  "sensor_msgs"
)
#################
#################
add_executable(ibeo_live src/ibeo_live.cpp)
target_include_directories(ibeo_live PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ibeo_8l_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${ibeo_8l_sdk_INCLUDE_DIRS}
)

target_link_libraries(ibeo_live 
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${IBEO_LIB}
  ${std_interfaces}
)

ament_target_dependencies(
  ibeo_live
  "std_msgs"
  "rclcpp"
  "pcl_conversions"
  "nav_msgs"
  "ibeo_8l_msgs"
  "ibeo_8l_sdk"
  "pcl_msgs"
  "PCL_COMMON"
  "PCL_IO"
  "sensor_msgs"
)

add_executable(obj_visualization src/obj_visualization.cpp)
target_include_directories(obj_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ibeo_8l_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${ibeo_8l_sdk_INCLUDE_DIRS}
)

target_link_libraries(obj_visualization 
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${IBEO_LIB}
  ${std_interfaces}
)

ament_target_dependencies(
  obj_visualization
  "std_msgs"
  "rclcpp"
  "pcl_conversions"
  "nav_msgs"
  "ibeo_8l_msgs"
  "ibeo_8l_sdk"
  "pcl_msgs"
  "PCL_COMMON"
  "PCL_IO"
  "sensor_msgs"
  "visualization_msgs"
)


install(TARGETS ibeo_live
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ibeo_file_sim
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS obj_visualization
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY record
  DESTINATION share/${PROJECT_NAME}
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