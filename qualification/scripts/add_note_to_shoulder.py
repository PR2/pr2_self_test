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
##\brief Adds note to shoulder explaining which UA was under test


SHOULDER_PN = '6804204'

PKG = 'qualification'
import roslib; roslib.load_manifest(PKG)


import wx
import sys

from qualification.srv import ScriptDone, ScriptDoneRequest
import rospy

from invent_client.invent_client import Invent

def _report_invalid_id():
    dlg = wx.MessageDialog(None, "Invalid Shoulder Serial number. Press Cancel to abort, OK to retry", "Invalid ID", wx.OK|wx.CANCEL)
    return dlg.ShowModal() == wx.ID_OK

def get_sn_from_user(iv):
    while not rospy.is_shutdown():
        sn = str(wx.GetTextFromUser("Enter the shoulder serial number by scanning the shoulder barcode", "Enter Shoulder SN"))
        if not unicode(sn).isnumeric() or not len(sn) == 12 or \
                not sn.startswith(SHOULDER_PN) or not iv.check_serial_valid(sn):
            if _report_invalid_id():
                continue
            return None
        
        return sn
    return None

def check_shoulder_ok(iv, shoulder_sn):
    if not iv.login():
        wx.MessageBox("Unable to login to invent", "Unable to login", wx.OK|wx.ICON_ERROR, None)
        return False

    if not iv.get_test_status(shoulder_sn):
        wx.MessageBox("Shoulder has not passed qualification. Qualify shoulder and retry", "Shoulder Not Qualified", wx.OK|wx.ICON_ERROR, None)
        return False
    return True

def set_note_to_shoulder(iv, shoulder_sn):
    ua_sn = rospy.get_param('/qual_item/serial', None)
    ua_test_name = rospy.get_param('/qual_item/name', None)

    if ua_sn is None or ua_test_name is None:
        wx.MessageBox("Unable to recover upperarm SN or test name from qual system", "Qual System Error", wx.OK|wx.ICON_ERROR, None)
        return False

    note = "Shoulder was qualified with upperarm %s usin qual test \"%s\"" % (ua_sn, ua_test_name)
    iv.setNote(shoulder_sn, note)

    return True
    

if __name__ == '__main__':
    rospy.init_node('shoulder_sn_getter')
    done_srv = rospy.ServiceProxy('prestartup_done', ScriptDone)
    rospy.wait_for_service('prestartup_done')
    
    app = wx.PySimpleApp()

    username = rospy.get_param('/invent/username', '')
    password = rospy.get_param('/invent/password', '')

    iv = Invent(username, password)
    if not iv.login():
        wx.MessageBox("Unable to login to invent", "Qual System Error", wx.OK|wx.ICON_ERROR, None)
        done_srv.call(result = ScriptDoneRequest.RESULT_FAIL, failure_msg = "Unable to login to invent")
        rospy.spin()
        sys.exit()
 
    sn = get_sn_from_user(iv)
    if sn is None:
        done_srv.call(result = ScriptDoneRequest.RESULT_FAIL, failure_msg = "User was unable to get shoulder SN")
        rospy.spin()
        sys.exit()

    if not check_shoulder_ok(iv, sn):
        done_srv.call(result = ScriptDoneRequest.RESULT_FAIL, failure_msg = "Shoulder %s had not passed qualification. Unable to complete." % sn)
        rospy.spin()
        sys.exit()

    if not set_note_to_shoulder(iv, sn):
        done_srv.call(result = ScriptDoneRequest.RESULT_FAIL, failure_msg = "Unable to load note into inventory system for shoulder %s" % sn)
        rospy.spin()
        sys.exit()

    done_srv.call(result = ScriptDoneRequest.RESULT_OK, failure_msg = "Qualifying with shoulder %s" % sn)

    rospy.spin()
    
    sys.exit(0)
        
