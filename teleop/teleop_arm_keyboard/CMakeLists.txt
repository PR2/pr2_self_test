cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
rosbuild_init()

rosbuild_add_executable(teleop_arm_keyboard teleop_arm_keyboard.cpp)


