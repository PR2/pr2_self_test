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

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_executable(pr2_laser_snapshotter src/pr2_laser_snapshotter.cpp)
rosbuild_add_executable(point_cloud_srv src/point_cloud_srv.cpp)

# Needed to run rostests
rosbuild_add_executable(dummy_scan_producer test/dummy_scan_producer.cpp)

rosbuild_add_executable(test_assembler 
                       test/test_assembler.cpp)
rosbuild_link_boost(test_assembler thread)
rosbuild_add_gtest_build_flags(test_assembler)

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_laser_assembler.launch)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
