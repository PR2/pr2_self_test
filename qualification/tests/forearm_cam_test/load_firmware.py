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

def check_programmed(iface):
    args = ['rosrun', 'forearm_cam', 'discover']
    if iface != None:
        args.append(iface)
    p = subprocess.Popen(args, stdout=subprocess.PIPE)
    impactout = p.communicate()[0]
    return 'Found camera' in impactout

rospy.init_node("load_firmware", anonymous=True)

r = TestResultRequest()
r.plots = []

try:
    iface=rospy.get_param("~cam_interface")
except:
    iface=None

try:
    impactdir=rospy.get_param("~impactdir")
except:
    import traceback
    traceback.print_exc()
#if (len(sys.argv) != 2):
    #print >> sys.stderr, 'must specify impact directory (%i) args given'%len(sys.argv);
    print >> sys.stderr, 'impactdir option must indicate impact project directory';
    r.html_result = "<p>Bad arguments to load_firmware.py.</p>"
    r.text_summary = "Error in test."
    r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
    print "error"
else:
    os.chdir(impactdir);
    reprogram = True
    if check_programmed(iface):
        app = wx.PySimpleApp()
	ret = wx.MessageBox("The device is already programmed. Skip reprogramming?", "Device Programmed", wx.YES|wx.NO)
	if ret == wx.YES:
	    reprogram = False
    if reprogram:
        p = subprocess.Popen(['./startimpact', '-batch', 'load_firmware.cmd'], stderr=subprocess.PIPE)
        impactout = p.communicate()[1]

        impactout = impactout.replace('\n','<br>')

        if '''INFO:iMPACT - '1': Flash was programmed successfully.''' in impactout:
            r.text_summary = "Firmware download succeeded."
            r.html_result = "<p>Test passed.</p><p>"+impactout+"</p>" 
            r.result = TestResultRequest.RESULT_PASS
            print "pass"
        else:
            r.text_summary = "Firmware download failed."
            r.result = TestResultRequest.RESULT_FAIL
            r.html_result = "<p>Test Failed.</p><p>"+impactout+"</p>"
            print "fail"
            print impactout
    else: 
        r.html_result = "<p>Firmware was already programmed.</p>"
        r.text_summary = "Firmware already programmed (Pass)." 
        r.result = TestResultRequest.RESULT_PASS
        print "pass"

    
result_service = rospy.ServiceProxy('test_result', TestResult)

rospy.sleep(5);

# block until the test_result service is available
rospy.wait_for_service('test_result')
result_service.call(r)

