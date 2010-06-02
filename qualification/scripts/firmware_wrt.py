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

from pr2_self_test_msgs.srv import TestResult, TestResultRequest

import rospy 

NAME = 'config_ctr'

import os
import sys
from StringIO import StringIO
import subprocess

import wx
from wx import xrc
from wx import html

from threading import Thread

import time

##\brief Uses instructions panel to show how to power cycle wifi
class PowerCycleFrame(wx.Frame):
    def __init__(self, html_file, status_cb):
        wx.Frame.__init__(self, None, wx.ID_ANY, 'Power Cycle WRT 610n')

        self._status_cb = status_cb

        xrc_path = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'xrc/gui.xrc')
        xrc_res = xrc.XmlResource(xrc_path)
        self._panel = xrc_res.LoadPanel(self, 'instructions_panel')
        self._sizer = wx.BoxSizer(wx.VERTICAL)
        self._sizer.Add(self._panel)
        self.Layout()

        self._html_window = xrc.XRCCTRL(self._panel, 'html_window')
        self._html_window.LoadFile(html_file)

        self._cancel_button = xrc.XRCCTRL(self._panel, 'cancel_button')
        self._cancel_button.Bind(wx.EVT_BUTTON, self.on_cancel)

        self._continue_button = xrc.XRCCTRL(self._panel, 'continue_button')
        self._continue_button.Bind(wx.EVT_BUTTON, self.on_continue)
        self._continue_button.SetFocus()

        self._done = False
        self._ok = True

    def on_continue(self, event):
        self._status_cb(True)
        self.Close()

    def on_cancel(self, event):
        self._status_cb(False)
        self.Close()

class DlgThread(Thread):
    def __init__(self, html_file):
        Thread.__init__(self)
        self.app = wx.PySimpleApp()
        self._frame = PowerCycleFrame(html_file, self._status_cb)
        self._frame.SetSize(wx.Size(550, 750))
        self._frame.Layout()
        self._frame.Show(True)

        self._ok = False
        self._done = False

    def _status_cb(self, ok):
        self._ok = ok
        self._done = True

    def is_done(self):
        return self._done

    def is_ok(self):
        return self._done and self._ok

    def run(self):
        self.app.MainLoop()

if __name__ == "__main__":
    rospy.init_node(NAME)

    r = TestResultRequest()
    r.plots = []
    r.result = TestResultRequest.RESULT_PASS

    if len(rospy.myargv()) > 0 and os.path.exists(rospy.myargv()[1]):
        instructions_file = rospy.myargv()[1]
    else:
        print "Unable to start configuration, no instructions file loaded"
        rospy.wait_for_service('test_result')
        r.result = TestResultRequest.RESULT_FAIL
        r.text_summary = "Unable to load instructions file, no given"
        result_service = rospy.ServiceProxy('test_result', TestResult)
        result_service.call(r)



    ip = '192.168.1.1'

    r.html_result =  "<p>Checking firmware version...</p>\n"

    wrt610n_version_cmd = ['wrt610n','version','-i',ip]
    wrt610n_version = subprocess.Popen(wrt610n_version_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (o,e) = wrt610n_version.communicate()
    if wrt610n_version.returncode != 0:
        r.html_result = "<p>Invocation of %s failed with: %s</p>\n<p>Trying 10.68.0.5...</p>\n"%(wrt610n_version_cmd,e)
        ip = '10.68.0.5'
        wrt610n_version_cmd = ['wrt610n','version','-i',ip]
        wrt610n_version = subprocess.Popen(wrt610n_version_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (o,e) = wrt610n_version.communicate()

    if wrt610n_version.returncode != 0:
        r.html_result = r.html_result + "<p>Invocation of %s failed with: %s</p>\n"%(wrt610n_version_cmd,e)
        r.text_summary = "Utility failed"
        r.result = TestResultRequest.RESULT_FAIL

    elif 'Router has firmware version: DD-WRT v24-sp2 (09/30/09) big - build 13000M NEWD-2 Eko' in o:
        r.html_result =  r.html_result + "<pre>%s</pre>"%o
    else:
        r.html_result =  r.html_result + "<p>Upgrading firmware...</p>\n"
        wrt610n_firmware_cmd = ['wrt610n','--force','-i',ip,'-w', '--nowait', 'firmware','/usr/lib/wrt610n/dd-wrt.v24-13000_big-wrt610n.bin']
        wrt610n_firmware = subprocess.Popen(wrt610n_firmware_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (o,e) = wrt610n_firmware.communicate()


        if wrt610n_firmware.returncode != 0:
            r.html_result = r.html_result + "<p>Invocation of %s failed with: %s</p>\n"%(wrt610n_firmware_cmd,e)
            r.text_summary = "Utility failed"
            r.result = TestResultRequest.RESULT_FAIL
        # We don't wait for boot anymore
        #elif "Router has resumed successfully on: 192.168.1.1 with firmare: 'DD-WRT v24-sp2 (09/30/09) big - build 13000M NEWD-2 Eko'" not in o:
        #    r.html_result =  r.html_result + "<pre>%s</pre>"%o
        #    r.text_summary = "Resume failed"
        #    r.result = TestResultRequest.RESULT_FAIL


    if r.result != TestResultRequest.RESULT_FAIL:
        ip = "192.168.1.1"

        r.html_result =  r.html_result + "<p>Requesting Hard Reset</p>\n"

        # We just wait here for a while to be safe
        time.sleep(200)

        print 'Starting reset dialog'
        dlgthread = DlgThread(instructions_file)
        dlgthread.start()
        dlgthread.join()
        print 'Done with reset dialog'

        # Need to check return code of thread
        if not dlgthread.is_ok():
            r.result = TestResultRequest.RESULT_FAIL
            r.text_summary = 'Failed to restart WRT610n using reset button.'
            

        
    if r.result != TestResultRequest.RESULT_FAIL:
        wrt610n_version_cmd = ['wrt610n','-w','version','-i',ip]
        wrt610n_version = subprocess.Popen(wrt610n_version_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (o,e) = wrt610n_version.communicate()
        if wrt610n_version.returncode != 0:
            r.html_result =  r.html_result + "<p>Invocation of %s failed with: %s</p>\n"%(wrt610n_version_cmd,e)
            r.result = TestResultRequest.RESULT_FAIL
            r.text_summary = "Utility failed"
        elif "Router has resumed successfully on: 192.168.1.1 with firmare: 'DD-WRT v24-sp2 (09/30/09) big - build 13000M NEWD-2 Eko'" not in o:
            r.html_result =  r.html_result + "<pre>%s</pre>"%o
            r.text_summary = "Hard Reset Resume failed"
            r.result = TestResultRequest.RESULT_FAIL
        else:
            r.html_result = r.html_result +  "<pre>%s</pre>"%o
            r.result = TestResultRequest.RESULT_PASS
            r.text_summary = "Firmware Upgraded"


    # block until the test_result service is available
    rospy.wait_for_service('test_result')
    result_service = rospy.ServiceProxy('test_result', TestResult)
    result_service.call(r)

    rospy.spin()
