cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
genmsg()
gensrv()

# Test test_monitor, with trans, ethercat, camera listeners
rosbuild_add_rostest(unit_test/launch/fake_monitor.launch)
