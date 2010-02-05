cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
#rosbuild_genmsg()
#rosbuild_gensrv()

# Parse launch files except in arm_life_test
rosbuild_add_pyunit(unit_test/check_launch_files.py)
