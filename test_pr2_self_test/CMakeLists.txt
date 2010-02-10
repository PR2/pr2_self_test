cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(selftest_demo src/selftest_demo.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_rostest(test/test_self_test_caller.launch)

# Test result.py for processing qualification results
rosbuild_check_for_display(disp)
if(disp)
  rosbuild_add_pyunit(test/test_results/all_tests_pass.py)
  rosbuild_add_pyunit(test/test_results/subtest_failure.py)
  rosbuild_add_pyunit(test/test_results/subtest_retry.py)
  rosbuild_add_pyunit(test/test_results/shutdown_failure.py)
  rosbuild_add_pyunit(test/test_results/prestart_failure.py)
  rosbuild_add_pyunit(test/test_results/prestart_error.py)
  rosbuild_add_pyunit(test/test_results/test_canceled.py)
  rosbuild_add_pyunit(test/test_results/tests_operator_pass.py)
endif(disp)