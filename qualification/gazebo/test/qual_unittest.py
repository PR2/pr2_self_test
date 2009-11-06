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

##\brief Tests receipt of test status service call for qual tests

PKG = 'qualification'
import roslib; roslib.load_manifest(PKG)

import unittest
import rospy, rostest
from time import sleep
import sys
from optparse import OptionParser

from qualification.srv import TestResult, TestResultResponse

DURATION = 150

class TestQualUnit(unittest.TestCase):
    def __init__(self, *args):
        super(TestQualUnit, self).__init__(*args)

        parser = OptionParser(usage="usage ./%prog [options]", prog="qual_unittest.py")
        parser.add_option('--human_ok', action="store_true",
                          dest="human", default=False,
                          metavar="HUMAN", help="Result-HUMAN_REQUIRED is a pass")
        # Option comes with rostest, will fail w/o this line
        parser.add_option('--gtest_output', action="store",
                          dest="gtest")



        self.success = False
        self.srv = None

        rospy.init_node('test_qual')

        options, args = parser.parse_args(rospy.myargv())

        rospy.Service('test_result', TestResult, self.result_cb)

        self._human = options.human
    
    def result_cb(self, srv):
        if srv.result == 0:
            self.success = True
        elif srv.result == 2 and self._human:
            self.success = True

        self.srv = srv

        return TestResultResponse()

    def test_qual_unit(self):
        start = rospy.get_time()
        while self.srv is None and not rospy.is_shutdown():
            sleep(5.0)
            if rospy.get_time() - start > DURATION:
                break
        
        self.assert_(not rospy.is_shutdown(), "Rospy shutdown")
        self.assert_(self.srv is not None, "No result from qualification test")
        self.assert_(self.success, "Qual test result was unsuccessful. Human OK: %s. Data: %s" % (self._human, self.srv.text_summary))


if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], TestQualUnit, sys.argv)
