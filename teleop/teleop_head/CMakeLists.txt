cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(teleop_head)
rospack_add_executable(teleop_head_keyboard teleop_head_keyboard.cpp)

