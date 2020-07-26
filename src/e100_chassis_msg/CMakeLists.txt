cmake_minimum_required(VERSION 3.5)
project(e100_chassis_msg)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(e100_chassis_msg)

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

set(msg_files
  "msg/E100EbsGeneralStatusFeedback.msg"
  "msg/E100ElectricParkBrakeStatusFeedback.msg"
  "msg/E100EpsSteeringControl.msg"
  "msg/E100EpsSteeringFeedback.msg"
  "msg/E100IduGeneralStatus1Control.msg"
  "msg/E100NmfIduControl.msg"
  "msg/E100VcuSpeedControl.msg"
  "msg/E100VcuSpeedFeedback.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
ADD_LINTER_TESTS)

# ament_target_dependencies(${PROJECT_NAME}_node std_msgs e100_chassis_msg)# ipopt z ssl uv uWS
# install(DIRECTORY ${PROJECT_NAME} DESTINATION ${PROJECT_NAME})

ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
