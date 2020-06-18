cmake_minimum_required(VERSION 3.5)
project(ibeo_8l_sdk)

## Set Boost paths
# set(BOOST_ROOT /opt/boost)
# set(Boost_NO_SYSTEM_PATHS ON)
set(Boost_INCLUDE_DIRS /usr/include/boost)
set(Boost_LIBRARIES_DIRS /usr/lib/x86_64-linux-gnu)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# find_package(Boost REQUIRED COMPONENTS
# system chrono regex thread
# program_options date_time
# )
find_package(Boost REQUIRED)
find_package(rosidl_default_generators REQUIRED)
## Find all sources of the SDK
file(GLOB_RECURSE SDK_SRCS "src/ibeosdk/*.cpp")
file(GLOB_RECURSE SDK_HDRS "src/ibeosdk/*.hpp "
                                                                  "src/ibeosdk/*.h")


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
##catkin_package(
##  INCLUDE_DIRS src
 ## LIBRARIES ibeo_8l_sdk
##  CATKIN_DEPENDS rclcpp
##  DEPENDS Boost
##)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 src
# ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}

)
include_directories(
 src/ibeosdk
)

## Declare a C++ library
add_library(${PROJECT_NAME} STATIC
   ${SDK_SRCS} ${SDK_HDRS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${ibeo_8l_sdk_DIR}
   -lrt
)

install(
  DIRECTORY src/ibeosdk
#   EXPORT export_${PROJECT_NAME}
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()