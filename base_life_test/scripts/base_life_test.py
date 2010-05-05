#!/usr/bin/env python
#BSD goes here

import roslib; roslib.load_manifest('base_life_test')
import tf
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker
import copy
from numpy import pi, cos, sin, mean
import time

base_frame = 'base_link'
map_frame = 'map'

# Initial pose
x_center = 2.207
y_center = 5.253
th_center = 3.071
x_range = 0.00
y_range = 0.6
th_range = 0.0
x_freq = 1.1
y_freq = 0.52
th_freq = 1.1

drive_halted = False

pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped)


def halt_drive(srv):
  global drive_halted
  drive_halted = True
  
  return EmptyResponse()

def reset_drive(srv):
  global drive_halted
  send_initial_pose()
  drive_halted = False

  return EmptyResponse()

def motors_cb(msg):
  global drive_halted
  
  if msg.data:
    drive_halted = True
  

def send_initial_pose():
  global pose_pub

  msg = PoseWithCovarianceStamped()
  msg.header.stamp = rospy.get_rostime()
  msg.header.frame_id = '/map'
  msg.pose.pose.position.x = x_center
  msg.pose.pose.position.y = y_center
  msg.pose.pose.orientation.z = 1

  msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  pose_pub.publish(msg)

def main():
  rospy.init_node('base_life_test')
  listener = tf.TransformListener()  
  rate = rospy.Rate(500)
  
  halt_sub = rospy.Service('pr2_base/halt_drive', Empty, halt_drive)
  reset_sub = rospy.Service('pr2_base/reset_drive', Empty, reset_drive)

  motors_sub = rospy.Subscriber('pr2_etherCAT/motors_halted', Bool, motors_cb)


  drive_pub = rospy.Publisher('base_driving', Bool, latch = True)

  marker_pub = rospy.Publisher('visualization_markers', Marker)
  cmd_pub = rospy.Publisher('cmd_vel', Twist)

  listener.waitForTransform(base_frame, map_frame, rospy.Time(0), rospy.Duration(5))

  t_start = rospy.Time.now().to_sec()
  started = time.asctime()

  last_drive_update = rospy.get_time()

  global drive_halted
  while not rospy.is_shutdown():
    #Update target location (sinusoids over the side length - buffer)
    t = rospy.Time.now().to_sec() - t_start
    x = x_center + sin(t / 2 * pi * x_freq) * x_range
    y = y_center + sin(t / 2 * pi * y_freq) * y_range
    th = th_center + sin(t / 2 * pi * th_freq) * th_range
    if not drive_halted:
      command_base_towards(x, y, th, listener, marker_pub, cmd_pub)
    rate.sleep()

    if rospy.get_time() - last_drive_update > 0.5:
      last_drive_update = rospy.get_time()
      drive_pub.publish(drive_halted)

  print "ran from %s to %s"%(started, time.asctime())


def command_base_towards(x, y, th, listener, marker_pub, cmd_pub):
    target = PoseStamped()
    target.header.frame_id = map_frame
    target.pose.position.x = x
    target.pose.position.y = y
    q = tf.transformations.quaternion_about_axis(th, (0, 0, 1))
    target.pose.orientation  = Quaternion(q[0], q[1], q[2], q[3])
  
    local_target = listener.transformPose(base_frame, target)

    marker = Marker()
    marker.header.frame_id = local_target.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.id = 10
    marker.pose = copy.deepcopy(local_target.pose)
    marker.color.r = 1
    marker.color.a = 1
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3
    marker.type = 0
    marker_pub.publish(marker)
    #Compute error between target location and actual location

    base_cmd = Twist()
    base_cmd.linear.y = 3 * local_target.pose.position.y
    #Special-case code for u-only movement to prevent wheel scrub.
    if abs(base_cmd.linear.y) > 0.1:
      base_cmd.linear.x = 3 * local_target.pose.position.x
    if abs(base_cmd.linear.x) + abs(base_cmd.linear.x) > 0.1:
      q = local_target.pose.orientation
      base_cmd.angular.z = 3 * tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    error = []
    for x in [local_target.pose.position.x, local_target.pose.position.y, local_target.pose.orientation.z / 5.0]:
      error.append(abs(x))
      if abs(x) > 1000:
        rospy.signal_shutdown("Error, too far from target.  Shutting down to be safe")
        print "Shutting down - failed to maintain target trajectory"
    #print int(20 * max(error) / 0.6) * "*"
    cmd_pub.publish(base_cmd) 
    #Command base velocity
   

if __name__=='__main__':
  main()
