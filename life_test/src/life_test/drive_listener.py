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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#

##\author Kevin Watts
##\brief Listens to pr2_etherCAT/motors_halted, allows base to drive

from __future__ import with_statement
PKG = 'life_test'

import roslib; roslib.load_manifest(PKG)

from std_msgs.msg import Bool
from std_srvs.srv import *

import rospy

import threading

class DriveListener:
    def __init__(self):

        self._mutex = threading.Lock()

        self._cal = False
        self._ok = True
        self._update_time = -1
        self._reset_driving = rospy.ServiceProxy('pr2_base/reset_drive', Empty)
        self._halt_driving = rospy.ServiceProxy('pr2_base/halt_drive', Empty)

        self._halt_motors =  rospy.ServiceProxy('pr2_etherCAT/halt_motors', Empty)

        self._diag_sub = rospy.Subscriber('pr2_etherCAT/motors_halted', Bool, self._motors_cb)

        self._cal_sub = rospy.Subscriber('calibrated', Bool, self._cal_cb)

        self._is_driving = True

        self._reset_drive_state = rospy.Service('start_driving', Empty, self.restart_driving)

    # Doesn't do anything
    def create(self, params):
        return True

    def halt(self):
        self._is_driving = False
        self._halt_driving()
        self._halt_motors()

    def reset(self):
        pass

    def restart_driving(self, srv):
        self._is_driving = True

        # Send pose to robot

        self._reset_driving()

        return EmptyResponse()

    def _cal_cb(self, msg):
        with self._mutex:
            self._cal = msg.data

    def _motors_cb(self, msg):
        with self._mutex:
            self._ok = not msg.data
            self._update_time = rospy.get_time()
            
            if not self._ok:
                self.halt()
    
    def check_ok(self):
        with self._mutex:
            msg = ''
            stat = 0
            if not self._cal:
                stat = 1
                msg = 'Uncalibrated'

            if not self._is_driving:
                stat = 1
                msg = 'Not driving'

            if not self._ok:
                stat = 2
                msg = ''

            

            if rospy.get_time() - self._update_time > 3:
                stat = 3
                msg = 'Motors Stale'
                if self._update_time == -1:
                    msg = 'No Motors Data'
        
        return stat, msg, None
    
