cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
genmsg()
gensrv()

rospack_add_executable(tests/wge100_camera_test/led_flash_test tests/wge100_camera_test/led_flash_test.cpp)

rosbuild_add_rostest_graphical(gazebo/launch/pr2_checkout.launch)
rosbuild_add_rostest_graphical(gazebo/launch/hysteresis_head_pan.launch)
rosbuild_add_rostest_graphical(gazebo/launch/sinesweep_head_pan.launch)
