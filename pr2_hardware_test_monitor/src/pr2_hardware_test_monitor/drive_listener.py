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
PKG = 'pr2_hardware_test_monitor'

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

        self._is_driving = True
        self._drive_sub = rospy.Subscriber('base_driving', Bool, self._drive_cb)

        # This restarts the driving. Driving is NOT reset by the "reset_test" service
        self._reset_drive_state = rospy.Service('start_driving', Empty, self.restart_driving)

    def create(self, params):
        return True

    def halt(self):
        self._halt_driving()

    def reset(self):
        pass

    def restart_driving(self, srv):
        self._reset_driving()

        return EmptyResponse()

    def _drive_cb(self, msg):
        with self._mutex:
            self._is_driving = not msg.data
            self._update_time = rospy.get_time()
            
    def check_ok(self):
        with self._mutex:
            msg = ''
            stat = 0
            if not self._is_driving:
                stat = 1
                msg = 'Not driving'

            if rospy.get_time() - self._update_time > 3:
                stat = 3
                msg = 'Drive Status Stale'
                if self._update_time == -1:
                    msg = 'No Drive Status'
        
        return stat, msg, None
    
