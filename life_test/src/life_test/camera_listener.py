#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
##\brief Listens to diagnostics from wge100 camera and reports OK/FAIL
PKG = 'life_test'

import roslib
roslib.load_manifest(PKG)

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import *

import rospy

import threading

class CameraListener:
    def __init__(self):
        self._diag_sub = rospy.Subscriber('/diagnostics', DiagnosticArray, self._diag_callback)
        self._mutex = threading.Lock()

        self._ok = True
        self._update_time = 0

    # Doesn't do anything
    def create(self, params):
        return True

    def halt(self):
        pass

    def reset(self):
        pass

    def _diag_callback(self, msg):
        self._mutex.acquire()
        for stat in msg.status:
            if stat.name.find('wge100') >= 0:
                self._ok = (stat.level == 0)
                self._update_time = rospy.get_time()
                if not self._ok:
                    break

        self._mutex.release()
    
    def check_ok(self):
        self._mutex.acquire()
        msg = ''
        stat = 0
        if not self._ok:
            stat = 2
            msg = 'Camera Error'

        if rospy.get_time() - self._update_time > 3:
            stat = 3
            msg = 'Camera Stale'
            if self._update_time == 0:
                msg = 'No Camera Data'
        
        self._mutex.release()
        return stat, msg, None
    
