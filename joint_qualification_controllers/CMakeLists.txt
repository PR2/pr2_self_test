cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_genmsg()
rosbuild_add_library(joint_qualification_controllers 
src/head_position_controller.cpp
src/hysteresis_controller.cpp
src/checkout_controller.cpp
src/counterbalance_test_controller.cpp
src/wrist_difference_controller.cpp
src/motor_joint_calibration_controller.cpp
src/joint_limit_calibration_controller.cpp)
