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

import sys
import rospy
from std_srvs.srv import *
from qualification.srv import *
import qualification.msg
import std_msgs
import rospy
import subprocess
import os
import os.path
import wx
import traceback
from invent_client.invent_client import Invent
from qualification.srv import TestResult, TestResultRequest

rospy.init_node("wge100_camera_set_name")

def getparam(name):
    val = rospy.get_param(name, None)
    if val == None:
        failed("Parameter %s not set"%name)
    return val

def failed(message):
    send_response("Error setting camera name", message, -1)

def passed(message):
    send_response("Camera successfully set to %s at %s "%(cameraname, cameraip),
            message, 0)

def send_response(summary, message, retval):
    print message.replace('<b>','').replace('</b>','')
    r=TestResultRequest()
    r.text_summary = summary
    r.html_result = "<p>%s</p>"%message.replace('\n','<br>')
    r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
    
    result_service = rospy.ServiceProxy('test_result', TestResult)
    rospy.sleep(5);
    rospy.wait_for_service('test_result')
    result_service.call(r)
    exit(retval)

try:
    # Get inventory password from qualification
    username = getparam('/invent/username')
    password = getparam('/invent/password')
    barcode = getparam('/qualification/serial')
    cameraname = getparam('~camera_name')
    cameraip = getparam('~camera_ip')
    progip = getparam('~programming_ip')
    
    # Fail if invalid username/password
    i = Invent(username, password)
    if not i.login():
        failed("Could not connect to invent.")
    
    # Get camera url
    try:
        camera_url = i.getItemReferences(barcode)["camera_url"]
        if camera_url == '':
            raise KeyError
    except KeyError:
        failed("Could not get camera url from invent. Try setting the  serial and MAC")
                                            
    # Set the camera's name
    p = subprocess.Popen(["rosrun", "wge100_camera", "set_name",
        camera_url+"@"+progip, cameraname, cameraip ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    setnameout = p.communicate()
    
    passed("Output from setname tool...\n<b>Standard output:</b>\n"+setnameout[0]+"\n\n<b>Standard error:</b>\n"+setnameout[1])

    if retval == 0:
        i.setKV(barcode, "Configured name", cameraname)
        i.setKV(barcode, "Configured ip", cameraip)
        passed("passed") # FIXME
    else:
        i.setKV(barcode, "Configured name", "<configure failed>")
        i.setKV(barcode, "Configured ip", "<configure failed>")
        failed("failed") # FIXME
except:
    failed('<b>Exception:</b>\n%s'%traceback.format_exc())
