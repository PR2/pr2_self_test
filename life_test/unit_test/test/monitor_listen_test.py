#!/usr/bin/env python
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
#

##\author Kevin Watts

##\brief Tests receipt of test monitor messages from life tests

DURATION = 5

PKG = 'life_test'
import roslib; roslib.load_manifest(PKG)

import unittest
import rospy, rostest
from time import sleep
import sys
from optparse import OptionParser

from life_test.msg import TestStatus
import threading

class TestMonitorUnit(unittest.TestCase):
    def __init__(self, *args):
        super(TestMonitorUnit, self).__init__(*args)

        parser = OptionParser(usage="usage ./%prog [options]", 
                              prog="monitor_listen_test.py")
        # Option comes with rostest, will fail w/o this line
        parser.add_option('--gtest_output', action="store",
                          dest="gtest")

        self._mutex = threading.Lock()
        self._ignore_time = 5
        self._ok = True
        self._message = None

        rospy.init_node('test_monitor_listener')
        self._start = rospy.get_time()

        options, args = parser.parse_args(rospy.myargv())

        rospy.Subscriber('test_status', TestStatus, self.cb)
    
    def cb(self, msg):
        self._mutex.acquire()
        self._ok = self._ok and (msg.test_ok == 0)
        self._message = msg.message
        self._mutex.release()

    def test_qual_unit(self):
        while not rospy.is_shutdown():
            sleep(1.0)
            if rospy.get_time() - self._start > DURATION:
                break

        self._mutex.acquire()
        self.assert_(not rospy.is_shutdown(), "Rospy shutdown")
        self.assert_(self._message is not None, "No data from test monitor")
        self.assert_(self._ok, "Test Monitor reports error. %s" % self._message)
        self._mutex.release()

if __name__ == '__main__':
    print 'SYS ARGS:', sys.argv
    rostest.run(PKG, sys.argv[0], TestMonitorUnit, sys.argv)
