cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_executable(tests/wge100_camera_test/led_flash_test tests/wge100_camera_test/led_flash_test.cpp)

rosbuild_add_pyunit(unit_test/check_test_files.py)

