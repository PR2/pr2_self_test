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

from qualification.srv import *

import rospy

finish = rospy.ServiceProxy('prestartup_done', ScriptDone)
result = rospy.ServiceProxy('test_result', TestResult)


app = wx.PySimpleApp()
ret = wx.MessageBox("Did you successfully manage to focus the camera?", "Camera Focus", wx.YES_NO)
if (ret == wx.NO):
    done = ScriptDoneRequest()
    done.result = ScriptDoneRequest.RESULT_FAIL
    done.failure_msg = 'User was unable to focus the camera.'

    r = TestResultRequest()
    r.result = r.RESULT_FAIL
    r.text_summary = 'User unable to focus camera'
    r.html_result = '<p>Unable to focus camera. It may not have loaded properly.</p>\n'
else:
    done = ScriptDoneRequest()
    done.result = ScriptDoneRequest.RESULT_OK
    done.failure_msg = ''

    r = TestResultRequest()
    r.result = r.RESULT_PASS
    r.text_summary = 'Camera Focused'
    r.html_result = ''
    
try:
    if len(sys.argv) == 1:
        rospy.wait_for_service('prestartup_done', 2)
        finish.call(done)
    else:
        rospy.wait_for_service('test_result', 2)
        result.call(r)
except:
    # Timeout exceeded while waiting for service
    sys.exit(0)
