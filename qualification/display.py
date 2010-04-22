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
##\brief Displays results from qual tests

PKG = 'qualification'

import roslib; roslib.load_manifest(PKG)
from qualification.qual_frame import PlotsPanel
from qualification.test import SubTest
from qualification.result import SubTestResult

import wx
from wx import xrc
from wx import html

import os
import rospy

from qualification.srv import TestResult, TestResultResponse

##\brief Makes waiting page for subtests
def make_waiting_page():
    return '<html><H2 align=center>Waiting for Test Data...</H2></html>'

class QualDisplayFrame(wx.Frame):
    def __init__(self, parent):
        wx.Frame.__init__(self, parent, wx.ID_ANY, "Qualification Display")
        
        # Load the XRC resource
        xrc_path = os.path.join(roslib.packages.get_pkg_dir(PKG), 'xrc/gui.xrc')
        xrc_res = xrc.XmlResource(xrc_path)

        self._shutdown_timer = wx.Timer(self, wx.ID_ANY)
        self.Bind(wx.EVT_TIMER, self._on_shutdown_timer, self._shutdown_timer)
        self._shutdown_timer.Start(10)

        self._plots_panel = PlotsPanel(self, xrc_res, self)
        self._plots_panel.show_waiting(make_waiting_page())

        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.SetSizer(sizer)
        sizer.Add(self._plots_panel, 1, wx.EXPAND)

        self._result_service = rospy.Service('test_result', TestResult, self.subtest_callback)

    ##\brief Check if we're shut down
    def _on_shutdown_timer(self, event):
        if rospy.is_shutdown():
            self.Close(True)

    def subtest_callback(self, msg):
        wx.CallAfter(self._st_data, msg)
        return TestResultResponse()

    def _st_data(self, msg):
        self._result_service.shutdown()
        self._result_service = None
        
        test = SubTest(os.path.join(roslib.packages.get_pkg_dir(PKG), 'N/A'), 1, "Qual Test")
        result = SubTestResult(test, msg)
        self._plots_panel.show_plots(result.make_result_page(),
                                     True)

    # Callback from the PlotsPanel
    def retry_subtest(self, notes):
        self.subtest_result(False, notes)
        
    def subtest_result(self, ok, notes):
        are_you_sure = wx.MessageDialog(self, 
                                        "This will clear the results window for the next result. Are you sure you want to clear the window? Press OK to clear.", 
                                        "Clear Window?",
                                        wx.OK|wx.CANCEL)
        if (are_you_sure.ShowModal() != wx.ID_OK):
            return
        
        
        self._plots_panel.show_waiting(make_waiting_page())
        self._result_service = rospy.Service('test_result', TestResult, self.subtest_callback)
        

    def cancel(self):
        are_you_sure = wx.MessageDialog(self, "Are you sure you want to quit?", "Confirm shut down",
                                        wx.OK|wx.CANCEL)
        if are_you_sure.ShowModal() != wx.ID_OK:
            return

        self.Close(True)

class QualDisplayApp(wx.App):
    def OnInit(self):
        from qualification.result_dir import check_qual_result_dir
        if not check_qual_result_dir():
            wx.MessageBox("Unable to write to the temporary results directory. This will cause weird problems. Open a terminal and type, \"sudo rm /tmp/* -rf\" to remove the offending directory.", 
                          "Unable to Write Results", wx.OK|wx.ICON_ERROR, None)
            return False

        self._frame = QualDisplayFrame(None)
        self._frame.SetSize(wx.Size(700, 900))
        self._frame.Layout()

        self._frame.Show(True)

        return True

if __name__ == '__main__':
    rospy.init_node('qual_display', anonymous=True)
    try:
        app = QualDisplayApp()
        app.MainLoop()
    except KeyboardInterrupt:
        pass
    except Exception, e:
        import traceback
        traceback.print_exc()
