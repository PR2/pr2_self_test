cmake_minimum_required(VERSION 2.8.3)
project(joint_qualification_controllers)
find_package(catkin REQUIRED COMPONENTS pr2_controller_interface pr2_mechanism_model pr2_hardware_interface control_toolbox roscpp robot_mechanism_controllers pluginlib std_msgs sensor_msgs realtime_tools urdf)

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

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS pr2_controller_interface pr2_mechanism_model pr2_hardware_interface control_toolbox roscpp robot_mechanism_controllers pluginlib std_msgs sensor_msgs realtime_tools urdf
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
