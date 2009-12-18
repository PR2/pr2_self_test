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
##\brief Listens to ecstats returns OK if no packets dropped

PKG = 'qualification'

import roslib
roslib.load_manifest(PKG)

from ectools.msg import ecstats
from qualification.srv import TestResult, TestResultRequest

import rospy

import threading
import traceback
from time import sleep

DURATION = 15

class ECStatsTest:
    def __init__(self):
        rospy.init_node('ecstats_test')
        self._ec_stats_sub = rospy.Subscriber('ecstats', ecstats, self._ecstats_cb)
        self._mutex = threading.Lock()
        self._start_time = rospy.get_time()

        self._ok = False
        self._update_time = -1
 
        self._has_link = False
        self._max_device_count = 0
        self._total_sent = 0
        self._interval_sent = 0
        self._total_dropped = 0
        self._interval_dropped = 0
        self._total_bandwidth = 0
        self._interval_bandwidth = 0

        self._result_service = rospy.ServiceProxy('test_result', TestResult)
        self._data_sent = False

    def send_results(self, r):
        if self._data_sent:
            return 
        self._result_service.call(r)
        self._data_sent = True

    def test_failed_service_call(self, exception = ''):
        r = TestResultRequest()
        r.result = TestResultRequest.RESULT_FAIL
        r.text_summary = 'Caught exception in ecstats test'
        r.html_result = '<p>%s</p>\n' % exception
        self.send_results(r)

    def is_done(self):
        return rospy.get_time() - self._start_time > DURATION

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

        self._ok = self._total_dropped == 0 and self._has_link

        self._mutex.release()

    def process_results(self):
        # TODO!!!!!!!!!!
        self._mutex.acquire()
        r = TestResultRequest()
        if self._ok:
            r.text_summary = 'Slip ring packet drop test: OK'
        else:
            r.text_summary = 'Slip ring packet drop test: FAIL'
        
        html = ['<p>Slip ring packet drop test data.</p>']
        html.append('<table border="1" cellpadding="2" cellspacing="0">')
        html.append('<tr><td><b>Name</b></td><td><b>Value</b></td></tr>')
        html.append('<tr><td>Has Link?</td><td>%s</td></tr>' % (str(self._has_link)))
        html.append('<tr><td>Dropped Packets</td><td>%d</td></tr>' % (self._total_dropped))
        html.append('<tr><td>Total Sent</td><td>%d</td></tr>' % (self._total_sent))
        html.append('<tr><td>Max Device Count (should be 0)</td><td>%d</td></tr>' % (self._max_device_count))
        html.append('<tr><td>Duration</td><td>%d</td></tr>' % (DURATION))
        html.append('</table>')

        r.html_result = '\n'.join(html)
        
        if self._ok:
            r.result = TestResultRequest.RESULT_PASS
        else:
            r.result = TestResultRequest.RESULT_FAIL

        self._mutex.release()

        self.send_results(r)
    
if __name__ == '__main__':
    monitor = ECStatsTest()
    try:
        while not rospy.is_shutdown() and not monitor.is_done():
            sleep(0.5)
        monitor.process_results()
        rospy.spin()
    except KeyboardInterrupt:
        pass
    except:
        traceback.print_exc()
        monitor.test_failed_service_call(traceback.format_exc())
            
