#!/usr/bin/env python
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

##\author Melonee Wise
##\brief Commands PR2 head for burn in test

PKG = "life_test"

import roslib; roslib.load_manifest(PKG)

import random
import rospy

from sensor_msgs.msg import JointState

head_pub = None

def point_head(pan, tilt):
    js = JointState()
    js.name = ['head_pan_joint', 'head_tilt_joint']
    js.position = [ pan, tilt ]
    js.header.stamp = rospy.get_rostime()
    head_pub.publish(js)

if __name__ == "__main__":
   rospy.init_node('head_commander')
   head_pub = rospy.Publisher('head_controller/command', JointState)

   rate = rospy.get_param('cycle_rate', 1.0)
   my_rate = rospy.Rate(float(rate))

   while not rospy.is_shutdown():
       pan = random.uniform(-2.7, 2.7)
       tilt = random.uniform(-0.32, 1.20)
       point_head(pan, tilt)
       my_rate.sleep()
