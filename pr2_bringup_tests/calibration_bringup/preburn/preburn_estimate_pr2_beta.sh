#! /bin/bash
mkdir -p ~/preburn
cp /tmp/pr2_calibration/cal_measurements.bag ~/preburn
roslaunch pr2_bringup_tests preburn_estimate_pr2_beta.launch
rosrun pr2_calibration_launch backup_measurements.sh
