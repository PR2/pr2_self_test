# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_counterbalance_check)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS joint_qualification_controllers rospy std_msgs pr2_self_test_msgs pr2_controller_configuration actionlib pr2_controllers_msgs pr2_controller_manager)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

catkin_python_setup()
set(ROSPACK_MAKEDIST true)

# Append to CPACK_SOURCE_IGNORE_FILES a semicolon-separated list of
# directories (or patterns, but directories should suffice) that should
# be excluded from the distro.  This is not the place to put things that
# should be ignored everywhere, like "build" directories; that happens in
# rosbuild/rosbuild.cmake.  Here should be listed packages that aren't
# ready for inclusion in a distro.
#
# This list is combined with the list in rosbuild/rosbuild.cmake.  Note
# that CMake 2.6 may be required to ensure that the two lists are combined
# properly.  CMake 2.4 seems to have unpredictable scoping rules for such
# variables.
#list(APPEND CPACK_SOURCE_IGNORE_FILES /core/experimental)
include_directories(${PYTHON_MATPLOT_DIRS})

# CATKIN_MIGRATION
# use bloom tool
## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)
if(CATKIN_ENABLE_TESTING)
	#catkin_add_nosetests(test/cb_analysis_test.py DEPENDENCIES ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
  find_package(rostest REQUIRED)

  #DFS: temporarily removing due to non-functioning test
  #add_rostest(test/cb_test.test DEPENDENCIES ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
endif()


# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS joint_qualification_controllers rospy std_msgs pr2_self_test_msgs pr2_controller_configuration actionlib pr2_controllers_msgs pr2_controller_manager
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

