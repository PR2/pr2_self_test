cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
genmsg()
gensrv()

# Test test_monitor, with trans, ethercat, camera listeners
rosbuild_add_rostest(unit_test/launch/fake_monitor.launch)

#execute_process(COMMAND rosrun roslaunch_parse_tester package_parse_test.py qualification --black_dir  RETURN_VARIABLE ret_var)
#if(NOT "${ret_var}" STREQUAL 0)
#  MESSAGE(FATAL_ERROR "Unable to parse all launch files. Error -> ${ret_var}")
#endif(NOT "${ret_var}" STREQUAL 0)