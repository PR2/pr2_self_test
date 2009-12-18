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
##\brief Listens to ecstats, makes sure no dropping packets

PKG = 'life_test'

import roslib
roslib.load_manifest(PKG)

from ectools.msg import ecstats
from std_srvs.srv import *
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

import rospy

import threading

class ECStatsListener:
    def __init__(self):
        self._ec_stats_sub = rospy.Subscriber('ecstats', ecstats, self._ecstats_cb)
        self._mutex = threading.Lock()

        self._ok = True
        self._update_time = -1
        
        self._halt_motors = rospy.ServiceProxy('pr2_etherCAT/halt_motors', Empty)

        self._drop_count_at_reset = 0
        self._time_at_last_reset = rospy.get_time()
        self._reset_count = 0

        self._has_link = False
        self._max_device_count = 0
        self._total_sent = 0
        self._interval_sent = 0
        self._total_dropped = 0
        self._interval_dropped = 0
        self._total_bandwidth = 0
        self._interval_bandwidth = 0

        self._lost_link_count_since_reset = 0
        self._lost_link_count = 0

    def create(self, params):
        return True

    def halt(self):
        pass

    def reset(self):
        self._time_at_last_reset = rospy.get_time()
        self._drop_count_at_reset = self._total_dropped
        self._ok = True
        self._reset_count += 1
        self._lost_link_count_since_reset = 0

    def _ecstats_cb(self, msg):
        self._mutex.acquire()
        self._has_link              = msg.has_link
        self._max_device_count      = msg.max_device_count
        self._total_sent            = msg.total_sent_packets
        self._interval_sent_packets = msg.interval_sent_packets
        self._total_dropped         = msg.total_dropped_packets
        self._interval_dropped      = msg.interval_dropped_packets
        self._total_bandwidth       = msg.total_bandwidth_mbps
        self._interval_bandwidth    = msg.interval_bandwidth_mbps

        if not self._has_link:
            self._lost_link_count += 1
            self._lost_link_count_since_reset += 1

        was_ok = self._ok
        self._ok = self._ok and self._has_link
        #self._total_dropped == self._drop_count_at_reset \
        if was_ok and not self._ok:
            try:
                #self._halt_motors()
                rospy.logerr('Should\'ve halted motors, went down')
            except:
                rospy.logerr('Attempted to halt motors after dropped packets, failed')

        self._update_time = rospy.get_time()
        self._mutex.release()
    
    def check_ok(self):
        self._mutex.acquire()

        stat = 0 if self._ok else 2
        msg = '' if self._ok else 'Dropped Packets'

        if rospy.get_time() - self._update_time > 3:
            stat = 3
            msg = 'Packet Data Stale'
            if self._update_time == -1:
                msg = 'No Packet Data'
        
        diag = DiagnosticStatus()
        diag.name = 'EC Stats Packet Data'
        diag.level = stat
        diag.message = msg
        if diag.level == 0:
            diag.message = 'OK'
            
        diag.values = [
            KeyValue(key='Has Link?',              value=str(self._has_link)),
            KeyValue(key='Dropped Since Reset',    value=str(self._total_dropped - self._drop_count_at_reset)),
            KeyValue(key='Total Drops',            value=str(self._total_dropped)),
            KeyValue(key='Lost Link Count',        value=str(self._lost_link_count)),
            KeyValue(key='Lost Links Since Reset', value=str(self._lost_link_count_since_reset)),
            KeyValue(key='Number of Resets',       value=str(self._reset_count)),
            KeyValue(key='Time Since Last Reset',  value=str(rospy.get_time() - self._time_at_last_reset)),
            KeyValue(key='Drops at Last Reset',    value=str(self._drop_count_at_reset)),
            KeyValue(key='Max Device Count',       value=str(self._max_device_count)),
            KeyValue(key='Total Sent Packets',     value=str(self._total_sent)),
            KeyValue(key='Interval Sent Packets',  value=str(self._interval_sent)),
            KeyValue(key='Total Bandwidth',        value=str(self._total_bandwidth)),
            KeyValue(key='Interval Bandwidth',     value=str(self._interval_bandwidth))
            
            ]
        
        self._mutex.release()
        

        return stat, msg, [ diag ]
    
