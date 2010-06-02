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
##\brief User verifies that the projector is on

import roslib
roslib.load_manifest('qualification')
import wx
import sys

from pr2_self_test_msgs.srv import ScriptDone, ScriptDoneRequest

import rospy

SRV_NAME = 'visual_check'

rospy.init_node('projector_verifier')
done_srv = rospy.ServiceProxy(SRV_NAME, ScriptDone)
rospy.wait_for_service(SRV_NAME)

app = wx.PySimpleApp()
ret = wx.MessageDialog(None, "Is the projector on? If so, click YES", "Projector Check", wx.YES_NO)

ok = ret.ShowModal() == wx.ID_YES

try:
    if ok:
        done_srv.call(result = ScriptDoneRequest.RESULT_OK)
    else:
        done_srv.call(result = ScriptDoneRequest.RESULT_FAIL, failure_msg = "User reported projector off")
    
    ret.Destroy()
    rospy.spin()
except KeyboardInterrupt:
    pass

sys.exit(0)
