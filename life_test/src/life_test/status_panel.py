#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

# Author: Kevin Watts

PKG = 'life_test'
import roslib
roslib.load_manifest(PKG)

import wx
from wx import xrc
import rospy

import threading
import math
import os

from msg import TestInfoArray, TestInfo

def get_duration_str(duration):
    hrs = max(math.floor(duration / 3600), 0)
    min = max(math.floor(duration / 6), 0) / 10 - hrs * 60
    
    return "%dhr, %.1fm" % (hrs, min)

##\brief Displays test status, including bay, power, estop, elapsed time
class TestStatusPanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent, -1)

        self._frame = parent
        
        xrc_path = os.path.join(roslib.packages.get_pkg_dir(PKG), 'xrc/gui.xrc')
        self._panel = xrc.XmlResource(xrc_path).LoadPanel(self, 'status_panel')
        self._bay_status = xrc.XRCCTRL(self._panel, 'bay_status')
        self._power_status = xrc.XRCCTRL(self._panel, 'power_status')
        self._estop_status = xrc.XRCCTRL(self._panel, 'estop_status')
        self._test_name_text = xrc.XRCCTRL(self._panel, 'test_name_text')
        self._serial_text = xrc.XRCCTRL(self._panel, 'serial_text')
        self._elapsed_text = xrc.XRCCTRL(self._panel, 'elapsed_text')
        self._machine_text = xrc.XRCCTRL(self._panel, 'machine_text')

        #self._close_button = xrc.XRCCTRL(self._panel, 'close_button')
        #self._close_button.Bind(wx.EVT_BUTTON, self.on_close)
        #self._close_button.Enable(False)

        self._bay_status.SetValue("Bay: None")
        self._power_status.SetValue("No Power Data")
        self._estop_status.SetValue("No Power Data")
        self._test_name_text.SetValue("No Test")
        self._serial_text.SetValue("No Test")
        self._elapsed_text.SetValue("No Data")
        self._machine_text.SetValue("Machine: None")

        self._serial = None

    def on_close(self, event):
        self._frame.close_tab(self._serial)

    def get_test_name(self):
        return self._test_name_text.GetValue()

    def get_elapsed(self):
        return self._elapsed

    def set_stale(self):
        self._bay_status.SetBackgroundColour("Light Blue")
        self._bay_status.SetValue("Stale")

        self._power_status.SetBackgroundColour("Light Blue")
        self._estop_status.SetBackgroundColour("Light Blue")
        self._elapsed_text.SetBackgroundColour("Light Blue")

    ##\brief Update display with new status message
    def update(self, msg):
        if msg.test_status == -1:
            self._bay_status.SetBackgroundColour("White")
            self._bay_status.SetValue("OFF")
            
        elif msg.test_status == 127:
            self._bay_status.SetBackgroundColour("White")
            self._bay_status.SetValue("N/A")
        elif msg.test_status == 0:
            self._bay_status.SetBackgroundColour("Light Green")
            self._bay_status.SetValue(msg.bay_name)
        elif msg.test_status == 1:
            self._bay_status.SetBackgroundColour("Orange")
            self._bay_status.SetValue(msg.bay_name)
        elif msg.test_status == 2:
            self._bay_status.SetBackgroundColour("Red")
            self._bay_status.SetValue(msg.bay_name)
        elif msg.test_status == 3 or msg.test_status == 4:
            self._bay_status.SetBackgroundColour("Light Blue")
            self._bay_status.SetValue(msg.bay_name)

        if msg.board == 0:
            self._power_status.SetValue("No power board")
            self._power_status.SetBackgroundColour("White")
            self._estop_status.SetValue("No power board")
            self._estop_status.SetBackgroundColour("While")
        else:
            if msg.power_status == "Standby":
                self._power_status.SetBackgroundColour("Orange")
                self._power_status.SetValue("Standby - %d #%d" % (msg.board, msg.breaker))
            elif msg.power_status == "On":
                self._power_status.SetBackgroundColour("Light Green")
                self._power_status.SetValue("On - %d #%d" % (msg.board, msg.breaker))
            else:
                self._power_status.SetBackgroundColour("Red")
                self._power_status.SetValue("Disable - %d #%d" % (msg.board, msg.breaker))
            if msg.estop == 0:
                self._estop_status.SetBackgroundColour("Red")
                self._estop_status.SetValue("E-Stop Hit")
            else:
                self._estop_status.SetBackgroundColour("Light Green")
                self._estop_status.SetValue("E-Stop OK")
        
        self._test_name_text.SetValue(msg.test_name)
        self._serial = msg.serial
        self._serial_text.SetValue(self._serial)

        self._elapsed = msg.elapsed
        self._elapsed_text.SetBackgroundColour("White")
        self._elapsed_text.SetValue(get_duration_str(self._elapsed))
        if msg.machine == "":
            self._machine_text.SetValue("Not launched")
        else:
            self._machine_text.SetValue("Machine: %s" % msg.machine)

##\brief Holds status panels in scrolled window
##
## Subscribes to 'test_info'/TestInfoArray and creates/updates status panels
## from information. 
class TestContainerStatusPanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent)

        self._mutex = threading.Lock()
        
        xrc_path = os.path.join(roslib.packages.get_pkg_dir(PKG), 'xrc/gui.xrc')
        
        self._info_sub = rospy.Subscriber('test_info', TestInfoArray, self.cb)
        
        self._panel = xrc.XmlResource(xrc_path).LoadPanel(self, 'status_container_panel')
        self._sizer = wx.BoxSizer(wx.VERTICAL)
        self._sizer.Add(self._panel, 1, wx.EXPAND)
        self.SetSizer(self._sizer)
        self.Layout()

        self._scrolled_window = xrc.XRCCTRL(self._panel, 'scrolled_window')
        self._scrolled_sizer = wx.BoxSizer(wx.VERTICAL)
        self._scrolled_window.SetSizer(self._scrolled_sizer)
        self._scrolled_window.SetScrollRate(0, 20)
        
        self._status_panels = {}
        
        # Shows blue if we're stale
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self._timer.Start(1000)

        self.timeout = 5
        self.last_message_time = 0

    def on_timer(self, event):
        if rospy.get_time() - self.last_message_time < self.timeout:
            return

        for serial, panel in self._status_panels.iteritems():
            panel.set_stale()

    def close_tab(self, serial):
        if not self._status_panels.has_key(serial):
            return

        print 'Deleting panel %s' % serial
        panel = self._status_panels[serial]
        if self._scrolled_sizer.Detach(panel):
            print 'detach ok'
            self._scrolled_sizer.Layout()
        else:
            print 'Not detached'
        
        del self._status_panels[serial]

    def cb(self, msg):
        self._mutex.acquire()
        self.msg = msg
        self._mutex.release()
        wx.CallAfter(self.update)

    def update(self):
        self._mutex.acquire()
        self.last_message_time = rospy.get_time()
        serials = self._status_panels.keys()

        for test_info in self.msg.data:
            if not self._status_panels.has_key(test_info.serial):
                stat_panel = TestStatusPanel(self)
                stat_panel.SetSize(wx.Size(600, 160))
                self._status_panels[test_info.serial] = stat_panel
                self._scrolled_sizer.Add(stat_panel, 0, wx.EXPAND)
                self._scrolled_sizer.Layout()
                serials.append(test_info.serial)
            
            self._status_panels[test_info.serial].update(test_info)
            
            idx = serials.index(test_info.serial)
            serials.pop(idx)

        ## Update panels that have closed
        for srl in serials:
            panel = self._status_panels[srl]

            srl_info = TestInfo()
            srl_info.serial = srl
            srl_info.test_status = -1
            srl_info.board = 0
            srl_info.test_name = panel.get_test_name()
            srl_info.elapsed = panel.get_elapsed()
            srl_info.machine = "None"
            
            panel.update(srl_info)
            
        self._mutex.release()
        
        ## Todo - allow delete used panels
        ## put into test manager

