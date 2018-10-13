cmake_minimum_required(VERSION 2.8.3)
project(au_pid)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rostest
  actionlib
  dynamic_reconfigure
  message_generation
  std_msgs
  std_srvs
  sensor_msgs
  actionlib_msgs
  au_core
  au_control
  eigen_conversions
)


catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
#add_action_files (
#  DIRECTORY action
#  FILES
  
#)

## Dynamic parameter


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
  sensor_msgs
  actionlib_msgs
  au_core
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES state_command visual_servo
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs actionlib_msgs au_core au_control
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# build lqr_controller
#add_library(lqr
#    src/lqr_controller.cpp
#)
#add_dependencies(lqr
#    ${${PROJECT_NAME}_EXPORTED_TARGETS}
#)
#target_link_libraries(lqr
#    ${catkin_LIBRARIES}
#)
#add_executable(lqr_controller
#    src/lqr_controller_node.cpp
#)
#add_dependencies(lqr_controller
#    ${${PROJECT_NAME}_EXPORTED_TARGETS}
#)
#target_link_libraries(lqr_controller
#    lqr
#    ${catkin_LIBRARIES}
#)



# build PID aggregator
add_executable(pid_aggregator
    src/pid_aggregator_node.cpp
    src/pid_aggregator.cpp
    src/pid_wrapper.cpp
)
add_dependencies(pid_aggregator
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pid_aggregator
    ${catkin_LIBRARIES}
)



###########
## Tests ##
###########

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

#    add_rostest_gtest(pid_aggregator_tests
#            test/test_pid_aggregator.test
#            test/pid_aggregator_tests.cpp
#    )
#    target_link_libraries(pid_aggregator_tests ${catkin_LIBRARIES})



endif()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
    src/pid_control_lib/robotStateCommand.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h|*.hpp"
  PATTERN ".svn" EXCLUDE
)
