cmake_minimum_required(VERSION 2.8.3)
project(joint_qualification_controllers)
find_package(catkin REQUIRED COMPONENTS pr2_controller_interface pr2_mechanism_model pr2_hardware_interface control_toolbox roscpp robot_mechanism_controllers pluginlib std_msgs sensor_msgs realtime_tools urdf message_generation)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lpthread")

include_directories(include ${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  ActuatorData.msg
  CBRunData.msg
  CBPositionData.msg
  CounterbalanceTestData.msg
  HysteresisData2.msg
  HysteresisData.msg
  HysteresisRun.msg
  JointData.msg
  JointPositionData.msg
  RobotData.msg
  WristDiffData.msg
  WristRollTurn.msg
)

add_library(joint_qualification_controllers
src/head_position_controller.cpp
src/hysteresis_controller.cpp
src/hysteresis_controller2.cpp
src/checkout_controller.cpp
src/counterbalance_test_controller.cpp
src/wrist_difference_controller.cpp
src/motor_joint_calibration_controller.cpp
src/joint_limit_calibration_controller.cpp)

generate_messages(
    DEPENDENCIES geometry_msgs std_msgs
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS} joint_qualification_controllers_gencpp)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS pr2_controller_interface pr2_mechanism_model pr2_hardware_interface control_toolbox roscpp robot_mechanism_controllers pluginlib std_msgs sensor_msgs realtime_tools urdf
    CATKIN_DEPENDS message_runtime # TODO
    INCLUDE_DIRS include # TODO include
    LIBRARIES ${PROJECT_NAME}# TODO
)


install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES controller_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
