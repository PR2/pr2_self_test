#!/usr/bin/env python

PKG = 'base_life_test'

import roslib; roslib.load_manifest(PKG)

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class AssistedTeleop:
  def __init__(self):
    rospy.init_node('safe_shuffle_filter')
    self.vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb)
    self.laser_sub = rospy.Subscriber("base_scan", LaserScan, self.laser_cb)
    self.pub = rospy.Publisher("base_controller/command", Twist)
    self.last_scan = None

    #the larger this timestep the further away from things we'll stop and the
    #easier it will be to back up, the problem is it may make going through doorways
    #more difficult
    self.dt = 0.01

    #how far away from objects we'll start slowing down
    self.threshold = 0.05
    #self.threshold = 0.10

    #we'll only forward simulate one step... more may be too cpu intensive
    self.steps = 1

    #just used to filter out min_range values
    self.robot_radius = 0.15

    #the footprint of PR2
    self.footprint = [(0.35, 0.35), (0.35, -0.35), (-0.35, -0.35), (-0.35, 0.35)]

    #how far the laser is set back from the center of rotation of the robot
    self.laser_offset_x = 0.15

  def laser_cb(self, scan):
    self.last_scan = scan

  def cmd_vel_cb(self, cmd):
    if not self.last_scan:
      rospy.logwarn("No laser scan received yet")
      return

    dist = self.sim_cmd(cmd)
    #print "Dist %.2f" % math.sqrt(dist)
    scale = 1.0

    #if we're below our distance threshold... we'll scale our speed back
    if dist < math.pow(self.threshold, 2):
      scale = math.sqrt(dist) / self.threshold
    if dist < math.pow(self.threshold / 2, 2):
      scale = 0.0

    new_cmd = cmd
    new_cmd.linear.x = scale * cmd.linear.x
    new_cmd.linear.y = scale * cmd.linear.y
    new_cmd.angular.z = scale * cmd.angular.z
    self.pub.publish(new_cmd)

  def new_x_position(self, xi, vx, vy, theta, dt):
    return xi + (vx * math.cos(theta) + vy * math.cos(math.pi / 2 + theta)) * dt

  def new_y_position(self, yi, vx, vy, theta, dt):
    return yi + (vx * math.sin(theta) + vy * math.sin(math.pi / 2 + theta)) * dt

  def new_th_position(self, thetai, vth, dt):
    return thetai + vth * dt

  def sq_distance(self, p1, p2):
    return (p1[0] - p2[0]) * (p1[0] - p2[0]) +(p1[1] - p2[1]) * (p1[1] - p2[1])

  #simple projection of a laser scan
  def range_to_point(self, r, angle):
     return (r * math.cos(angle) + self.laser_offset_x, r * math.sin(angle))

  #compute the squared distance between a point and a line segment
  def point_seg_sq_dist(self, l0, l1, p):
    r_numerator = (p[0] - l0[0]) * (l1[0] - l0[0]) + (p[1] - l0[1]) * (l1[1] - l0[1])
    r_denomenator = (l1[0] - l0[0]) * (l1[0] - l0[0]) + (l1[1] - l0[1]) * (l1[1] - l0[1])
    r = r_numerator / r_denomenator

    s = ((l0[1] - p[1]) * (l1[0] - l0[0]) - (l0[0] - p[0]) * (l1[1] - l0[1])) / r_denomenator

    line_dist = abs(s) * r_denomenator

    if r >= 0 and r <= 1:
      return line_dist

    dist1 = (p[0] - l0[0]) * (p[0] - l0[0]) + (p[1] - l0[1]) * (p[1] - l0[1])
    dist2 = (p[0] - l1[0]) * (p[0] - l1[0]) + (p[1] - l1[1]) * (p[1] - l1[1])

    if dist1 < dist2:
      return dist1

    return dist2

  #get the oriented footprint of the robot at a (x, y, th) pose
  def getOrientedFootprint(self, x, y, th):
    oriented = []
    cos_th = math.cos(th)
    sin_th = math.sin(th)
    for pt in self.footprint:
      new_pt_x = x + (pt[0] * cos_th - pt[1] * sin_th)
      new_pt_y = y + (pt[0] * sin_th + pt[1] * cos_th)
      oriented.append((new_pt_x, new_pt_y))
    
    return oriented


  #forward simulate a velocity command and return the minimum squared distance to an obstacle
  def sim_cmd(self, cmd):
    x = 0.0
    y = 0.0
    th = 0.0

    start = rospy.Time.now()

    #3 meters is plenty far to start the min dist at
    min_sq_dist = 9.0

    #we need to simulate where the robot will be for the next few steps of time
    for i in range(0, self.steps):
      x = self.new_x_position(x, cmd.linear.x, cmd.linear.y, th, self.dt)
      y = self.new_y_position(y, cmd.linear.x, cmd.linear.y, th, self.dt)
      th = self.new_th_position(th, cmd.angular.z, self.dt)

      o_footprint = self.getOrientedFootprint(x, y, th)

      angle = self.last_scan.angle_min 

      for r in self.last_scan.ranges:
        laser_pt = self.range_to_point(r, angle)
        sq_dist = self.sq_distance((x, y), laser_pt)

	#check the distance of points in the laser scan to each segment in the footprint
        for j in range(0, len(o_footprint) - 1):
	  line_sq_dist = self.point_seg_sq_dist(o_footprint[j], o_footprint[j + 1], laser_pt)
          if sq_dist >= math.pow(self.robot_radius, 2) and line_sq_dist <= min_sq_dist:
	    min_sq_dist = line_sq_dist

	#make sure we do the last line in the footprint
	line_sq_dist = self.point_seg_sq_dist(o_footprint[0], o_footprint[len(o_footprint) - 1], laser_pt)
        if sq_dist >= math.pow(self.robot_radius, 2) and line_sq_dist <= min_sq_dist:
	  min_sq_dist = line_sq_dist

	angle += self.last_scan.angle_increment

    #return the sq_distance to the nearest obstacle
    return min_sq_dist

if __name__ == '__main__':
  ast = AssistedTeleop()
  rospy.spin()
