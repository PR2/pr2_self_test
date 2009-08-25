# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from numpy.numarray.numerictypes import Float64

# Author: Matthew Piccoli

import roslib
roslib.load_manifest('qualification')

import rospy
from qualification.srv import *
from std_msgs.msg import Float64
from pr2_msgs.msg import PressureState
from time import sleep
import traceback

class FingertipQualification:
  def pressure_callback(self,data):
    self.data0 = data.data0
    self.data1 = data.data1
    
  def __init__(self):
    self.set_cmd = 0.0
    #self.pub = rospy.Publisher('r_gripper_joint/set_commmand', Float64)
    self.pub = rospy.Publisher('joint_effort_controller/commmand', Float64)
    rospy.Subscriber("pressure/r_gripper_motor", PressureState, self.pressure_callback)
    rospy.init_node('fingertip_qualification')
    self.initial = rospy.get_param('grasp_force_initial', -25.0)
    self.increment = rospy.get_param('grasp_force_increment', -25.0) 
    self.num_increments = rospy.get_param('grasp_increments', 4) 
    self.x0 = rospy.get_param('x^0', -39181) 
    self.x1 = rospy.get_param('x^1', -4060.5)
    self.x2 = rospy.get_param('x^2', -45.618)
    self.x3 = rospy.get_param('x^3', -17.442)
    self.tol_percent = rospy.get_param('tolerance_percent', 10.0)
    self.fingertip_refresh = rospy.get_param('fingertip_refresh_hz', 25)
    self.num_sensors = rospy.get_param('num_sensors', 22)
    self.num_sensors_outside = rospy.get_param('num_sensors_outside', 7)
    self.force = self.initial

  def open_gripper(self):
    self.set_cmd = 100
    self.pub.publish(self.set_cmd)
    sleep(3)
    
  def close_gripper(self):
    self.set_cmd = -100
    self.pub.publish(self.set_cmd)
    sleep(3)
    
  def check_connected(self):
    sleep(1/self.fingertip_refresh*2)
    #TODO::check if data exists!!!!!
    for i in range(self.num_sensors_outside, self.num_sensors - 1):
      if self.data0[i] == 0:
        print "fingertip 0, sensor %i is not reporting" %i
        #TODO::Report failiure
      if self.data1[i] == 0:
        print "fingertip 1, sensor %i is not reporting" %i
        #TODO::Report failiure
        
  def increment_value(self):
    self.force = self.force + self.increment
    
  def record_zero_pos(self):
    self.starting_sum0 = 0
    self.starting_sum1 = 0
    for i in range(self.num_sensors_outside, self.num_sensors - 1):
      self.starting_sum0 = self.starting_sum0 + self.data0[j]
      self.starting_sum1 = self.starting_sum1 + self.data1[j]
      
  def check_increase(self):
    current_sum0 = 0
    current_sum1 = 0
    for i in range(self.num_sensors_outside, self.num_sensors - 1):
      current_sum0 = current_sum0 + self.data0[j]
      current_sum0 = current_sum0 + self.data1[j]
    expected_value = self.force*self.force*self.force*self.x3 + self.force*self.force*self.x2 + self.force*self.x1 + self.x0
    if current_sum0-self.starting_sum0 > expected_value*(1+self.tol_percent/100): #value is too high for this pad
      print "fingertip 0 is reporting values of %s that are too high above %s at force %s" %(current_sum0-self.starting_sum0, expected_value,self.force)
      #TODO::Report failiure
    if current_sum0-self.starting_sum0 < expected_value*(1-self.tol_percent/100): #value is too low for this pad
      print "fingertip 0 is reporting values of %s that are too low below %s at force %s" %(current_sum0-self.starting_sum0, expected_value,self.force)
      #TODO::Report failiure
    if current_sum1-self.starting_sum1 > expected_value*(1+self.tol_percent/100): #value is too high for this pad
      print "fingertip 1 is reporting values of %s that are too high above %s at force %s" %(current_sum0-self.starting_sum0, expected_value,self.force)
      #TODO::Report failiure
    if current_sum1-self.starting_sum1 < expected_value*(1-self.tol_percent/100): #value is too low for this pad
      print "fingertip 1 is reporting values of %s that are too low below %s at force %s" %(current_sum0-self.starting_sum0, expected_value,self.force)
      #TODO::Report failiure

if __name__ == '__main__':
  try:
    qual = FingertipQualification()
    qual.open_gripper()
    qual.check_connected()
    for j in range(0,qual.num_increments):
      qual.record_zero_pos()
      qual.close_gripper()
      qual.check_increase()
      qual.open_gripper()
      qual.increment_value()
    rospy.spin()
  except Exception, e:
    print 'Caught exception in fingertip_qualification.\n%s' % traceback.format_exc()
    rospy.logerr('Fingertip qualification exception.\n%s' % traceback.format_exc())

    print 'Quitting fingertip qualification'