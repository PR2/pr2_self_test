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

from qualification.msg import *
from qualification.srv import *

import rospy 

NAME = 'config_ctr'

import os
import sys
from StringIO import StringIO
import subprocess

import wx

from threading import Thread

import time

if __name__ == "__main__":
    rospy.init_node(NAME)
    
    r = TestResultRequest()
    r.plots = []
    r.result = TestResultRequest.RESULT_PASS

    wrt610n_config_cmd = ['wrt610n','-w','config' '--essid=willow','--newip=10.68.0.5','--newpasswd=willow']
    wrt610n_config = subprocess.Popen(wrt610n_config_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (o,e) = wrt610n_config.communicate()
    if wrt610n_config.returncode != 0:
        r.html_result = r.html_result + "<p>Invocation of %s failed with: %s</p>\n"%(wrt610n_version_cmd,e)
        r.text_summary = "Utility failed"
        r.result = TestResultRequest.RESULT_FAIL
    elif "Router has resumed successfully on: 10.68.0.5 with firmare: 'DD-WRT v24-sp2 (09/30/09) big - build 13000M NEWD-2 Eko'" not in o:
        r.html_result =  r.html_result + "<pre>%s</pre>"%o
        r.text_summary = "Resume failed"
        r.result = TestResultRequest.RESULT_FAIL
    else:
        r.html_result = r.html_result +  "<pre>%s</pre>"%o
        r.result = TestResultRequest.RESULT_PASS
        r.text_summary = "Router Configured"


    # block until the test_result service is available
    rospy.wait_for_service('test_result')
    result_service = rospy.ServiceProxy('test_result', TestResult)
    result_service.call(r)
