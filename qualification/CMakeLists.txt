cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_executable(tests/wge100_camera_test/led_flash_test tests/wge100_camera_test/led_flash_test.cpp)

rosbuild_add_rostest(gazebo/launch/pr2_checkout.launch)
rosbuild_add_rostest(gazebo/launch/hysteresis_head_pan.launch)
rosbuild_add_rostest(gazebo/launch/sinesweep_head_panlaunch)

#execute_process(COMMAND rosrun roslaunch_parse_tester package_parse_test.py qualification --env=ROS_TEST_HOST,localhost --black_dir config/wge100_camera RETURN_VARIABLE ret_var)
#if(NOT "${ret_var}" STREQUAL 0)
#  MESSAGE(FATAL_ERROR "Unable to parse all launch files. Error -> ${ret_var}")
#endif(NOT "${ret_var}" STREQUAL 0)
#endif(ret_val != 0)