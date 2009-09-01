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
import os

from qualification.srv import *

import rospy

def getparam(name):
    val = rospy.get_param(name, None)
    if val == None:
        print >> sys.stderr, "Parameter %s not set"%name
        exit(-1)
    return val

rospy.init_node("wge100_set_mac_dialog")
barcode = getparam('qual_item/serial')
camera_path = getparam('~camera_path')+"/board_config"

app = wx.PySimpleApp()
ret = wx.MessageBox("Does the window labeled 'Camera to be Programmed' show images from camera %s, and are you sure you want to permanently set its MAC and serial number?"%barcode, "Set MAC and Serial Number", wx.YES_NO)
done = ScriptDoneRequest()
if (ret == wx.NO):
    done.result = ScriptDoneRequest.RESULT_FAIL
    done.failure_msg = 'User pressed NO.'
else:
    if os.system("rosrun qualification wge100_board_config.py board_config:=%s %s"%(camera_path, barcode)) == 0:
        done.result = ScriptDoneRequest.RESULT_OK
        done.failure_msg = ''
    else:
        done.result = ScriptDoneRequest.RESULT_FAIL
        done.failure_msg = 'Board configuration failed.'
    
try:
    finish = rospy.ServiceProxy('prestartup_done', ScriptDone)
    rospy.wait_for_service('prestartup_done', 2)
    finish.call(done)
    sys.exit(0)
except:
    # Timeout exceeded while waiting for service
    sys.exit(0)
