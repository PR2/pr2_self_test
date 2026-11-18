## Overview

This package contains tools to test and qualify a production PR2.

[PR2 Calibration Bringup SOP](#calibration_bringup) -
Information qualifying the mechanical calibration of newly built robots

## calibration_bringup

### Verifying the Calibration

Once the robot is calibrated, we need to do a sanity check on the sensor
streams to ensure that they're calibrated. All of these checks will be
done through rviz.

Restart the robot: 

```
sudo robot stop
sudo robot start
```



On the robot, start moving the tilt scanner 

```
rosrun pr2_calibration_launch robot_cal_tilt_profile.py laser_tilt_controller
```



To easily move the arms, you can start mannequin mode


```
roslaunch pr2_mannequin_mode pr2_mannequin_mode.launch
```



On your local machine, bring up rviz: 

```
source /opt/ros/fuerte/setup.sh
export ROS_MASTER_URI=http://prXXXX:11311
rosrun rviz rviz
```



### Check various sensor pairs

Ensure that both the fixed frame and target frame are in the
`base_link`

1\. Narrow Stereo against both arms

- Bring up `narrow_stereo/points` point cloud view
- Verify that point cloud matches up with robot model 1. Wide Stereo
  against both arms
- Bring up `wide_stereo/points` point cloud view
- Verify that point cloud matches up with robot model 1. Forearm cameras
  against both arms
- Bring up `r_forearm_cam/image_rect` & `l_forearm_cam/image_rect`
  camera views
- Make sure robot model matches up with both arms in images 1. Tilt
  laser against arms
- Bring up `tilt_scan` laser scan view
- Make sure robot model arms both match up with points 1. Tilt laser
  against wide stereo
- Bring up `tilt_scan` laser scan view
- Bring up `wide_stereo/left/image_rect` camera view
- Place large checkerboard in view of camera
- Verify that laser points on checkerboard match up with image 1. Tilt
  laser against narrow stereo
- Bring up `tilt_scan` laser scan view
- Bring up `narrow_stereo/left/image_rect` camera view
- Place large checkerboard in view of camera
- Verify that laser points on checkerboard match up with image
