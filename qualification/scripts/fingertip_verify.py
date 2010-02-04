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

##\brief Displays fingertip pressure GUI. Display turns green after sensor is pressed

from __future__ import with_statement

TOUCH = 4000
NUMSENSORS = 22
MINVALUE = -100000
RATE = 5

import roslib; roslib.load_manifest('qualification')

import wx
import threading
from wx import xrc

from qualification.srv import TestResult, TestResultRequest
from pr2_msgs.msg import PressureState
import rospy

import sys

from fingertip_pressure.fingertip_panel import FingertipPressurePanel


class FingertipVerifyPanel(FingertipPressurePanel):
    def __init__(self, parent):
        FingertipPressurePanel.__init__(self, parent)

        self.set_stale()

        
    def set_stale(self):
        self._max_values = [MINVALUE for i in range(0, NUMSENSORS)]
        for i in range(0, NUMSENSORS):
            self.pad[i].SetBackgroundColour("Light Blue")
            self.pad[i].SetValue("#%i\nStale" % i)

    def check_ok(self):
        ok = True
        for i in range(0, NUMSENSORS):
            ok = ok and self._max_values[i] > TOUCH
        
        return ok
                
    def new_message(self, data):
        with self._mutex:
            for i in range(0, NUMSENSORS):
                self._max_values[i] = max(data[i], self._max_values[i])
                
                if self._max_values[i] > TOUCH:
                    self.pad[i].SetBackgroundColour("Light Green")
                else:
                    self.pad[i].SetBackgroundColour("Red")

                self.pad[i].SetValue("#%i\n%i\nMax: %i" % (i, data[i], self._max_values[i]))
                    
        
class FingerTipStatusPanel(wx.Panel):
    def __init__(self, parent, topic, result_cb, left = True):
        wx.Panel.__init__(self, parent, wx.ID_ANY)

        self._mutex = threading.Lock()

        sizer = wx.BoxSizer(wx.VERTICAL)
        self.verify_panel = FingertipVerifyPanel(self)
        sizer.Add(self.verify_panel.panel, 1, wx.EXPAND)

        self.pass_button = wx.Button(self, wx.ID_OK, "Pass")
        self.fail_button = wx.Button(self, wx.ID_CANCEL, "Fail")

        self.pass_button.Bind(wx.EVT_BUTTON, self.on_pass)
        self.fail_button.Bind(wx.EVT_BUTTON, self.on_fail)

        self.pass_button.Enabled = False

        button_sizer = wx.BoxSizer(wx.HORIZONTAL)
        button_sizer.Add(self.pass_button, 0, wx.ALIGN_CENTER | wx.ALL, 5)
        button_sizer.Add(self.fail_button, 0, wx.ALIGN_CENTER | wx.ALL, 5)

        sizer.Add(button_sizer)

        self.SetSizer(sizer)
        self.Layout()

        self.result_service = rospy.ServiceProxy('/test_result', TestResult)
        
        self._last_update_time = 0
        self._pressure_sub = rospy.Subscriber(topic, PressureState, self.pressure_cb)

        self._left_side = left

        self.timer = wx.Timer(self, 1)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        
        self.stale_timer = wx.Timer(self, 2)
        self.Bind(wx.EVT_TIMER, self.on_stale_timer, self.stale_timer)
        self.stale_timer.Start(1000)

        self.result_cb = result_cb

    def on_close(self):
        if self._pressure_sub:
            self._pressure_sub.unregister()
            self._pressure_sub = None


    def on_pass(self, event):
        self.result_cb(True)

    def on_fail(self, event):
        self.result_cb(False)


    def on_timer(self, event):
        pass

    def on_stale_timer(self, event):
        if rospy.get_time() - self._last_update_time > 3:
            self.verify_panel.set_stale()
            self.pass_button.Enabled = False

    def pressure_cb(self, msg):
        with self._mutex:
            if self._left_side:
                self._data = msg.l_finger_tip
            else:
                self._data = msg.r_finger_tip
            
            self._last_update_time = rospy.get_time()

            if not self.timer.IsRunning():
                self.timer.Start(1000/RATE, True)
                wx.CallAfter(self.display)

    def display(self):
        with self._mutex:
            self.verify_panel.new_message(self._data)

            if self.verify_panel.check_ok():
                self.pass_button.Enabled = True

            self.Refresh()

class FingerTipVerifyFrame(wx.Frame):
    def __init__(self, parent, id, title):
        wx.Frame.__init__(self, parent, wx.ID_ANY, title)

        sizer = wx.BoxSizer()
        self.SetSizer(sizer)
        
        self._panel = FingerTipStatusPanel(self, 'pressure/r_gripper_motor', self.result_cb)

        sizer.Add(self._panel, 1, wx.EXPAND)

        self.data_sent = False

        self.shutdown_timer = wx.Timer(self, 10)
        self.Bind(wx.EVT_TIMER, self.on_shutdown_timer, self.shutdown_timer)

        self.Bind(wx.EVT_CLOSE, self.on_close)


    def send_results(self, result, msg = ''):
        if self.data_sent:
            return

        r = TestResultRequest()
        if result:
            r.html_result = '<p>Fingertip verification OK</p>\n'
            r.text_summary = 'Fingertip verification OK'
            r.result = TestResultRequest.RESULT_PASS
        else:
            r.html_result = '<p>Fingertip verification failed</p>\n<p>%s</p>' % msg
            r.text_summary = 'Fingertip verification failed.'
            r.result = TestResultRequest.RESULT_PASS

        print 'Sending test result. Summary: %s' % r.text_summary
        try:
            rospy.wait_for_service('test_result', 2)
        except:
            return
        self.result_service.call(r)
        self.data_sent = True

    def result_cb(self, result, msg = ''):
        self.send_results(result, msg = '')
        self.Close(True)

    def on_close(self, event):
        try:
            if event.CanVeto():
                dialog = wx.MessageDialog(self, 'Are you sure you want to close this window? Fingertip verification will automatically fail.', 'Confirm Close', wx.OK|wx.CANCEL)
                if dialog.ShowModal() != wx.ID_OK:
                    print 'Vetoing'
                    event.Veto()
                    return

            self._panel.on_close()
            self.send_results(False, 'Window closed')
            
        except:
            pass

        self.Destroy()
        
    def on_exit(self, event):
        self.Close(True)

    def on_shutdown_timer(self, event):
        if rospy.is_shutdown():
            self.Close(True)

class FingerTipVerifyApp(wx.App):
    def __init__(self):
        wx.App.__init__(self, clearSigInt = False)
        
    def OnInit(self):
        self._frame = FingerTipVerifyFrame(None, -1, "Fingertip Sensor Check")
        self._frame.SetMinSize(self._frame.GetEffectiveMinSize())        
        self._frame.Layout()
        self._frame.Show()

        return True

if __name__ == '__main__':
    rospy.init_node('finger_verifier')

    app = FingerTipVerifyApp()
    try:
        app.MainLoop()
        #rospy.spin()
    except KeyboardInterrupt:
        raise
    except Exception, e:
        print 'Caught exception'
        import traceback
        rospy.logerr(traceback.format_exc())

    sys.exit(0)
    
    # Need try/catch, etc
    # Need service calls for pass/fail
