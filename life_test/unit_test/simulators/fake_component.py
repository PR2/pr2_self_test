#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

##\author Kevin Watts
##\brief Tests Test Monitors

PKG = 'life_test'
import roslib; roslib.load_manifest(PKG)


from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import *
from pr2_mechanism_msgs.msg import MechanismStatistics, JointStatistics, ActuatorStatistics

import os
import time

import rospy
import threading

from time import sleep

import math

class FakeComponent:
    def __init__(self):
        rospy.init_node('fake_component')
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)
        self.mech_pub = rospy.Publisher('mechanism_statistics', MechanismStatistics)
        self._reset_srv = rospy.Service('reset_motors', Empty, self.on_reset)
        self._halt_srv = rospy.Service('halt_motors', Empty, self.on_halt)

        self._mutex = threading.Lock()
        self._ok = True

        self._start_time = rospy.get_time()

        self._last_diag_pub = rospy.get_time()

    def on_halt(self, srv):
        self._mutex.acquire()
        self._ok = True
        self._mutex.release()

    def on_reset(self, srv):
        self._mutex.acquire()
        self._ok = False
        self._mutex.release()

    def publish_mech_stats(self):
        ok = False
        self._mutex.acquire()
        ok = self._ok
        self._mutex.release()

        # Joint state is a sine, period 1s, Amplitude 2,
        trig_arg = rospy.get_time() - self._start_time

        sine = math.sin(trig_arg)
        cosine = math.cos(trig_arg)

        jnt_st = JointStatistics()
        jnt_st.name = 'fake_joint'
        jnt_st.position = float(2 * sine)
        jnt_st.velocity = float(2 * cosine)
        jnt_st.measured_effort = float(-2 * sine)
        jnt_st.commanded_effort = float(-2 * sine)
        jnt_st.is_calibrated = 1

        cont_st = JointStatistics()
        cont_st.name = 'cont_joint'
        cont_st.position = 5 * float(0.5 * sine)
        cont_st.velocity = 2.5 * float(0.5 * cosine)
        cont_st.is_calibrated = 1

        cont_act_st = ActuatorStatistics()
        cont_act_st.name = 'cont_motor'
        cont_act_st.calibration_reading = False 
        wrapped_position = (cont_st.position % 6.28)
        if wrapped_position > 3.14:
            cont_act_st.calibration_reading = True
        
        act_st = ActuatorStatistics()
        act_st.name = 'fake_motor'
        act_st.calibration_reading = True
        if sine > 0.0:
            act_st.calibration_reading = False

        mech_st = MechanismStatistics()
        mech_st.actuator_statistics = [ act_st, cont_act_st ]
        mech_st.joint_statistics = [ jnt_st, cont_st ]
        mech_st.header.stamp = rospy.get_rostime()

        self.mech_pub.publish(mech_st)

    def publish_diag(self):
        ok = False
        self._mutex.acquire()
        ok = self._ok
        self._mutex.release()

        msg = DiagnosticArray()
        stat = DiagnosticStatus()
        msg.status.append(stat)

        stat.level = 0
        if not ok:
            stat.level = 2
        stat.name = 'EtherCAT Master'
        stat.message = 'OK'

        stat_cam = DiagnosticStatus()
        stat_cam.level = 0
        stat_cam.name = 'wge100: Frequency Status'
        stat_cam.message = 'OK'

        msg.status.append(stat_cam)
        msg.header.stamp = rospy.get_rostime()
     
        self.diag_pub.publish(msg)

    def publish(self):
        self.publish_mech_stats()
        if rospy.get_time() - self._last_diag_pub > 1.0:
            self.publish_diag()

if __name__ == '__main__':
    fc = FakeComponent()
    my_rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        my_rate.sleep()
        fc.publish()
