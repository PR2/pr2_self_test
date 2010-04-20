cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_genmsg()
#rosbuild_gensrv()

# Test test_monitor, with trans, ethercat, camera listeners
rosbuild_add_rostest(unit_test/launch/fake_monitor.launch)

# Parse launch files except in arm_life_test
rosbuild_add_pyunit(unit_test/test/check_launch_files.py)
rosbuild_add_pyunit(unit_test/test/check_config_files.py)
