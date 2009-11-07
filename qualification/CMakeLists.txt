cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
genmsg()
gensrv()

rospack_add_executable(tests/wge100_camera_test/led_flash_test tests/wge100_camera_test/led_flash_test.cpp)

# Check whether there's an X display.  Intended to use in gating tests that
# require a display.
# TODO: after ROS 0.10 is released, remove this re-declaration
macro(rosbuild_check_for_display var)
  execute_process(COMMAND "xdpyinfo"
                  OUTPUT_VARIABLE _dummy
                  ERROR_VARIABLE _dummy
                  RESULT_VARIABLE _xdpyinfo_failed)
  if(_xdpyinfo_failed)
    set(${var} 0)
  else(_xdpyinfo_failed)
    set(${var} 1)
  endif(_xdpyinfo_failed)
endmacro(rosbuild_check_for_display)

rosbuild_check_for_display(disp)
if(disp)
  rosbuild_add_rostest(gazebo/launch/pr2_checkout.launch)
  rosbuild_add_rostest(gazebo/launch/hysteresis_head_pan.launch)
  rosbuild_add_rostest(gazebo/launch/sinesweep_head_pan.launch)
endif(disp)
