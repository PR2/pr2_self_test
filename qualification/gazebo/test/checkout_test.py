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

##\brief Tests loading of checkout controller on PR2 with gazebo

DURATION = 120

PKG = 'qualification'
import roslib; roslib.load_manifest(PKG)

import unittest
import rospy, rostest
from time import sleep
import sys

from qualification.srv import TestResult, TestResultResponse

class TestCheckoutPR2(unittest.TestCase):
    def __init__(self, *args):
        super(TestCheckoutPR2, self).__init__(*args)

        self.success = False
        self.srv = None
    
    def result_cb(self, srv):
        if srv.result == 0:
            self.success = True

        self.srv = srv

        print 'Summary:', srv.text_summary
        return TestResultResponse()

    def test_checkout_pr2(self):
        rospy.init_node('test_checkout')
        rospy.Service('test_result', TestResult, self.result_cb)
        while self.srv is None:
            sleep(5.0)
        
        self.assert_(self.srv is not None, "No result from checkout controller")
        self.assert_(self.success, "Checkout result was unsuccessful. Data: %s" % self.srv.text_summary)


if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], TestCheckoutPR2, sys.argv)
