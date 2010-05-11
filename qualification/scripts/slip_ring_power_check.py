#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

##\author Kevin Watts
##\brief Checks that power wires of slip rings are OK

PKG = 'qualification'
import roslib; roslib.load_manifest(PKG)

import wx
import sys

from qualification.srv import ScriptDone, ScriptDoneRequest
import rospy

SRV_NAME = 'prestartup_done'

def prompt_check_wires():
    msg = "Check slip ring power cable for resistance.\n\nCheck both cables, make sure resistance is less than 1ohm.\n\nClick \"OK\" if resistance is less than 1ohm or \"Cancel\" to abort."

    dlg = wx.MessageDialog(None, msg, "Check slip ring power wires", wx.OK|wx.CANCEL)
    rv = dlg.ShowModal() == wx.ID_OK
    dlg.Destroy()
    return rv

if __name__ == '__main__':
    rospy.init_node('slip_ring_power_test')
    done_srv = rospy.ServiceProxy(SRV_NAME, ScriptDone)
    rospy.wait_for_service(SRV_NAME)
    
    app = wx.PySimpleApp()

    val = prompt_check_wires()
    if val:
        done_srv.call(result = ScriptDoneRequest.RESULT_OK)
    else:
        done_srv.call(result = ScriptDoneRequest.RESULT_FAIL, failure_msg='Slip ring power connection is bad')

    rospy.spin()
    
    sys.exit(0)
        
