# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(joint_qualification_controllers)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS pr2_controller_interface pr2_mechanism_model pr2_hardware_interface control_toolbox roscpp robot_mechanism_controllers pluginlib std_msgs sensor_msgs realtime_tools urdf message_generation)


include_directories(include ${PROJECT_NAME})
# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()
add_message_files(
  FILES
  # TODO: List your msg files here
  ActuatorData.msg
  CBPositionData.msg
  CBRunData.msg
  CounterbalanceTestData.msg
  HysteresisData2.msg
  HysteresisData.msg
  HysteresisRun.msg
  JointData.msg
  JointPositionData.msg
  RobotData.msg
  WristDiffData.msg
  WristRollTurn.msg
)

generate_messages()

add_library(joint_qualification_controllers 
src/head_position_controller.cpp
src/hysteresis_controller.cpp
src/hysteresis_controller2.cpp
src/checkout_controller.cpp
src/counterbalance_test_controller.cpp
src/wrist_difference_controller.cpp
src/motor_joint_calibration_controller.cpp
src/joint_limit_calibration_controller.cpp)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS pr2_controller_interface pr2_mechanism_model pr2_hardware_interface control_toolbox roscpp robot_mechanism_controllers pluginlib std_msgs sensor_msgs realtime_tools urdf
    CATKIN_DEPENDS message_runtime# TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

