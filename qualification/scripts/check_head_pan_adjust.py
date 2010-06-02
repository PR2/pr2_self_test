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

import roslib
roslib.load_manifest('qualification')
import wx
import sys

from pr2_self_test_msgs.srv import TestResult, TestResultRequest

import rospy

result = rospy.ServiceProxy('test_result', TestResult)

app = wx.PySimpleApp()
ret = wx.MessageBox("The plot shows the encoder count of the head pan. Press OK if you can adjust the head pan so the backlash is less than 4 ticks. Press Cancel if you can't", "Adjust Head Pan", wx.YES_NO)
if (ret == wx.NO):
    r = TestResultRequest()
    r.result = r.RESULT_FAIL
    r.text_summary = 'Operator was unable to adjust head pan'
    r.html_result = '<p>Unable to adjust head pan. May be operator error.</p>\n'
else:
    r = TestResultRequest()
    r.result = r.RESULT_PASS
    r.text_summary = 'Head pan adjusted'
    r.html_result = ''
    
try:
    rospy.wait_for_service('test_result', 2)
    result.call(r)
    rospy.spin()
except:
    # Timeout exceeded while waiting for service
    sys.exit(0)
