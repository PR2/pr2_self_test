#! /bin/bash
rosrun pr2_calibration_estimation post_process.py /u/pr2admin/preburn/cal_measurements.bag /u/pr2admin/preburn `rospack find pr2_calibration_launch`/view_results/head_arm_config.yaml

