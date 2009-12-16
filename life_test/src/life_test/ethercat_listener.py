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
##\brief Listens to pr2_etherCAT/motors_halted, makes sure etherCAT state is OK
PKG = 'life_test'

import roslib
roslib.load_manifest(PKG)

from std_msgs.msg import Bool
from std_srvs.srv import *

import rospy

import threading

class EthercatListener:
    def __init__(self):
        self._diag_sub = rospy.Subscriber('pr2_etherCAT/motors_halted', Bool, self._motors_cb)
        self._mutex = threading.Lock()

        self._ok = True
        self._update_time = -1
        self._reset_motors = rospy.ServiceProxy('pr2_etherCAT/reset_motors', Empty)
        self._halt_motors = rospy.ServiceProxy('pr2_etherCAT/halt_motors', Empty)

    # Doesn't do anything
    def create(self, params):
        return True

    def halt(self):
        self._halt_motors()

    def reset(self):
        self._reset_motors()

    def _motors_cb(self, msg):
        self._mutex.acquire()
        self._ok = msg.data
        self._update_time = rospy.get_time()
        self._mutex.release()
    
    def check_ok(self):
        self._mutex.acquire()
        msg = ''
        stat = 0
        if not self._ok:
            stat = 2
            msg = 'Motors Halted'

        if rospy.get_time() - self._update_time > 3:
            stat = 3
            msg = 'EtherCAT Stale'
            if self._update_time == -1:
                msg = 'No EtherCAT Data'
        
        self._mutex.release()
        return stat, msg, None
    
