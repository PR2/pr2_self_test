cmake_minimum_required(VERSION 2.8.3)
project(pr2_bringup_tests)
find_package(catkin REQUIRED COMPONENTS pr2_bringup pr2_controller_manager ethercat_trigger_controllers pr2_mannequin_mode image_view camera_calibration)

include_directories(${Boost_INCLUDE_DIRS})
catkin_package(
    DEPENDS pr2_bringup pr2_controller_manager ethercat_trigger_controllers pr2_mannequin_mode image_view camera_calibration
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(DIRECTORY launch/ calibration_bringup/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES life_test.rosinstall DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/gripper_accel.sh scripts/imu_data_plot.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


