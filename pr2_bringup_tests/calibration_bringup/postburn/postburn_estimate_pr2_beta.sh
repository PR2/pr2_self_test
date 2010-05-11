#! /bin/bash

mkdir -p ~/postburn
cp /tmp/pr2_calibration/cal_measurements.bag ~/postburn
roslaunch pr2_bringup_tests postburn_estimate_pr2_beta.launch

rosrun pr2_calibration_launch backup_measurements.sh
