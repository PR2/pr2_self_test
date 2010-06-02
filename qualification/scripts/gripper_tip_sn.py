#!/usr/bin/env python
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
#

##\author Kevin Watts
##\brief Gets the 4 digit gripper tip code from users and puts it in invent

PKG = 'qualification'
import roslib
roslib.load_manifest(PKG)
import wx
import sys

from pr2_self_test_msgs.srv import ScriptDone, ScriptDoneRequest

import rospy

def _report_invalid_id():
    dlg = wx.MessageDialog(None, "Invalid ID. Tip ID should be four digits, numeric. Press Cancel to abort or OK to retry.", "Invalid ID", wx.OK|wx.CANCEL)
    return dlg.ShowModal() == wx.ID_OK

def _report_id_mismatch():
    dlg = wx.MessageBox("Tip ID entries don't match.", "ID Mismatch", wx.OK|wx.ICON_ERROR, None)
    return 

def get_code_from_user():
    code1 = None
    while not rospy.is_shutdown():
        code1 = wx.GetTextFromUser("Enter the four digit ID on the back of the gripper tip.", "Gripper Tip ID")
        if not unicode(code1).isnumeric() or not len(code1) == 4:
            if _report_invalid_id():
                continue
            else:
                return None

        code2 = wx.GetTextFromUser("Re-enter the four digit ID on the back of the gripper tip.", "Gripper Tip ID")
        if code1 == code2:
            break
        _report_id_mismatch()
        
    if not code1 or code1 != code2:
        return None

    code = str(code1)
    return code
    

def add_reference(reference):
    from invent_client.invent_client import Invent
    username = rospy.get_param('/invent/username', '')
    password = rospy.get_param('/invent/password', '')
    serial = rospy.get_param('/qual_item/serial', None)
    
    iv = Invent(username, password)
    if not iv.login() or serial == None:
        return False
    
    iv.addItemReference(serial, 'PPS', reference)

    return True
    
    
if __name__ == '__main__':
    rospy.init_node('tip_sensor_id_recovery')
    done_srv = rospy.ServiceProxy('prestartup_done', ScriptDone)
    rospy.wait_for_service('prestartup_done')
    
    app = wx.PySimpleApp()
 
    code = get_code_from_user()
    if code is None:
        done_srv.call(result = ScriptDoneRequest.RESULT_FAIL, failure_msg = "User was unable to get correct PPS ID code from tip sensor")
        rospy.spin()
        sys.exit()

    if not add_reference(code):
        done_srv.call(result = ScriptDoneRequest.RESULT_FAIL, failure_msg = "Unable to load PPS code %s into inventory system." % code)
        rospy.spin()
        sys.exit()

    done_srv.call(result = ScriptDoneRequest.RESULT_OK, failure_msg = "Loaded PPS code %s into invent" % code)

    rospy.spin()
    
    sys.exit(0)
