cmake_minimum_required(VERSION 3.0.2)
project(imu_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_runtime
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS log_setup log program_options thread REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imu_driver
  CATKIN_DEPENDS message_runtime roscpp rospy sensor_msgs std_msgs tf
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${EIGEN3_INCLUDE_DIR} include/minimu9-ahrs src/minimu9-ahrs ${catkin_INCLUDE_DIRS})

add_executable(imu_driver 
  src/minimu9-ahrs/i2c_bus.cpp
  src/minimu9-ahrs/l3g.cpp
  src/minimu9-ahrs/lis3mdl.cpp
  src/minimu9-ahrs/lsm303.cpp
  src/minimu9-ahrs/lsm6.cpp
  src/main.cpp
  src/minimu9-ahrs/prog_options.cpp
  src/minimu9-ahrs/minimu9.cpp)

target_link_libraries(imu_driver ${catkin_LIBRARIES} ${Boost_LIBRARIES})