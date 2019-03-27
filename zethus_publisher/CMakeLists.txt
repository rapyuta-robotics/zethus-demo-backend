cmake_minimum_required(VERSION 2.8.3)
project(zethus_publisher)

# C++ 11
add_compile_options(-std=c++11)

# Warnings
add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_grasps
  moveit_planners_ompl
  moveit_ros_planning
  moveit_visual_tools
  roscpp
  roslint
  rosparam_shortcuts
  rostest
  sensor_msgs
  std_msgs
  tf
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

###################################
## Catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_grasps
    moveit_planners_ompl
    moveit_ros_planning
    moveit_visual_tools
    roscpp
    rosparam_shortcuts
    sensor_msgs
    std_msgs
    tf
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    Boost
    EIGEN3
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  SYSTEM
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library with project namespace to avoid naming collision
# add_library(
#   ${PROJECT_NAME}
#   src/bin_pick_sequencer.cpp
#   src/async_bin_pick_sequencer.cpp
#   src/pickit_interface.cpp
# )

# Specify libraries to link a library or executable target against
# target_link_libraries(
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# catkin_install_python(
#   PROGRAMS
#     scripts/pickit_pose_publisher.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Declare a C++ executable
add_executable(
  ${PROJECT_NAME}_moveit_msgs_publisher
  src/moveit_msgs_publisher.cpp
)

# Rename C++ executable without namespace
set_target_properties(
  ${PROJECT_NAME}_moveit_msgs_publisher
  PROPERTIES
    OUTPUT_NAME moveit_msgs_publisher PREFIX ""
)

# Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}_moveit_msgs_publisher
  #${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
# install(
#   TARGETS
#     ${PROJECT_NAME}
#     ${PROJECT_NAME}_bin_pick_sequencer_main
#   ARCHIVE DESTINATION
#     ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION
#     ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION
#     ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark cpp header files for installation
# install(
#   DIRECTORY
#     include/${PROJECT_NAME}/
#   DESTINATION
#     ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# Mark roslaunch files for installation
# install(
#   DIRECTORY
#     launch
#   DESTINATION
#     ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# Mark config files for installation
# install(
#   DIRECTORY
#     config
#   DESTINATION
#     ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)

#   add_rostest_gtest(ros_test test/ros_test.test test/ros_test.cpp)
#   target_link_libraries(ros_test
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     ${Boost_LIBRARIES}
#   )

# endif()

## Test for correct C++ source code
roslint_cpp()
