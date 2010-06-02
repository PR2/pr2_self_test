#!/usr/bin/env python
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

##\author Jeremy Leibs

import roslib
roslib.load_manifest('qualification')

from pr2_self_test_msgs.srv import TestResult, TestResultRequest

import rospy 

NAME = 'rebrand_ctr'

import os
import sys
from StringIO import StringIO
import subprocess

if __name__ == "__main__":
    rospy.init_node(NAME)
    
    essid = rospy.myargv()[1]

    r = TestResultRequest()
    r.plots = []

    ctr350_rebrand_cmd = ['ctr350','-i', '10.68.0.250','-p','willow','-n','PRLAN']
    ctr350_rebrand = subprocess.Popen(ctr350_rebrand_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (o,e) = ctr350_rebrand.communicate()

    if ctr350_rebrand.returncode != 0:
        r.html_result = r.html_result + "<p>Invocation of ctr350 on 10.68.0.250 failed with: %s</p>\n"%e
        r.result = TestResultRequest.RESULT_FAIL
        r.text_summary = "Utility failed"
    elif 'WAP has resumed successfully on: 10.68.0.250' not in o:
        r.html_result = "<pre>%s</pre>"%o
        r.result = TestResultRequest.RESULT_FAIL
        r.text_summary = "Resume failed"
    else:
        r.html_result = "<pre>%s</pre>"%o
        r.result = TestResultRequest.RESULT_PASS
        r.text_summary = "Configured"
    
    # block until the test_result service is available
    rospy.wait_for_service('test_result')
    result_service = rospy.ServiceProxy('test_result', TestResult)
    result_service.call(r)

    rospy.spin()
