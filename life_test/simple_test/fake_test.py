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

##\author Kevin Watts
##\brief Tests Test Manager system, simulates PR2 hardware undergoing life testing

import roslib
roslib.load_manifest('life_test')
import wx
import sys

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import *
from pr2_mechanism_msgs.msg import MechanismStatistics, JointStatistics, ActuatorStatistics
from std_msgs.msg import Bool

import os
import time

import rospy

from time import sleep
from wx import xrc

import math
import signal

class FakeTestFrame(wx.Frame):
    def __init__(self, parent):
        wx.Frame.__init__(self, parent, wx.ID_ANY, 'Fake Test')
        
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)
        self.mech_pub = rospy.Publisher('mechanism_statistics', MechanismStatistics)
        self.motors_pub = rospy.Publisher('pr2_etherCAT/motors_halted', Bool)

        self._mech_timer = wx.Timer(self, 2)
        self.Bind(wx.EVT_TIMER, self.on_mech_timer, self._mech_timer)
        self._last_mech_pub = rospy.get_time()
        self._mech_timer.Start(50, True)

        self._diag_timer = wx.Timer(self, 1)
        self.Bind(wx.EVT_TIMER, self.on_timer, self._diag_timer)
        self._last_publish = rospy.get_time()
        self._diag_timer.Start(500, True)
        
        # Load XRC
        xrc_path = os.path.join(roslib.packages.get_pkg_dir('life_test'), 'xrc/gui.xrc')

        self._panel = xrc.XmlResource(xrc_path).LoadPanel(self, 'fake_panel')
        self._pub_check_box = xrc.XRCCTRL(self._panel, 'publish_check_box')
        self._level_choice = xrc.XRCCTRL(self._panel, 'level_choice')

        self.enum_param_ctrl = xrc.XRCCTRL(self._panel, 'enum_param_ctrl')
        self.range_param_ctrl = xrc.XRCCTRL(self._panel, 'range_param_ctrl')

        self._cal_box = xrc.XRCCTRL(self._panel, 'calibration_box')
        
        self._reset_srv = rospy.Service('pr2_etherCAT/reset_motors', Empty, self.on_reset)
        self._halt_srv = rospy.Service('pr2_etherCAT/halt_motors', Empty, self.on_halt)

        self.set_enum_ctrl()
        self.set_range_ctrl()

        self._start_time = rospy.get_time()

    def on_mech_timer(self, event = None):
        if not rospy.is_shutdown():
            self._mech_timer.Start(10, True)
        
        # Joint state is a sine, period 1s, Amplitude 2,
        trig_arg = rospy.get_time() - self._start_time

        sine = math.sin(trig_arg)
        cosine = math.cos(trig_arg)

        # Stop joint at zero if not running
        level =  self._level_choice.GetSelection()
        if level != 0:
            sine = 0
            cosine = 0

        jnt_st = JointStatistics()
        jnt_st.name = 'fake_joint'
        jnt_st.position = float(2 * sine)
        jnt_st.velocity = float(2 * cosine)
        jnt_st.is_calibrated = True

        cont_st = JointStatistics()
        cont_st.name = 'cont_joint'
        cont_st.position = 5 * float(0.5 * sine)
        cont_st.velocity = 2.5 * float(0.5 * cosine)
        cont_st.is_calibrated = True

        cont_act_st = ActuatorStatistics()
        cont_act_st.name = 'cont_motor'
        cont_act_st.calibration_reading = False
        wrapped_position = (cont_st.position % 6.28)
        if wrapped_position > 3.14 and self._cal_box.IsChecked():
            cont_act_st.calibration_reading = True
        
        act_st = ActuatorStatistics()
        act_st.name = 'fake_motor'
        act_st.calibration_reading = True
        if sine > 0.0 and self._cal_box.IsChecked():
            act_st.calibration_reading = False

        mech_st = MechanismStatistics()
        mech_st.actuator_statistics = [ act_st, cont_act_st ]
        mech_st.joint_statistics = [ jnt_st, cont_st ]

        self.mech_pub.publish(mech_st)
  
    def on_halt(self, srv):
        wx.CallAfter(self.set_level, 2)
        return EmptyResponse()

    def on_reset(self, srv):
        wx.CallAfter(self.set_level, 0)
        return EmptyResponse()

    def set_level(self, val):
        self._level_choice.SetSelection(val)

    def set_enum_ctrl(self):
        enum_param = rospy.get_param('test_choice')
        self.enum_param_ctrl.SetValue(enum_param)

    def set_range_ctrl(self):
        range_param = rospy.get_param('cycle_rate')
        self.range_param_ctrl.SetValue(range_param)

    def on_timer(self, event = None):
        # Has to sleep to pick up sigint or something.
        sleep(0.1)
        if not rospy.is_shutdown():
            self._diag_timer.Start(1000, True)

        level_dict = { "OK": 0, "Warn": 1, "Error": 2 }

        level =  self._level_choice.GetSelection()

        choice = self._level_choice.GetStringSelection()
     
        if self._pub_check_box.IsChecked():
            self.publish_diag(level, str(choice))

    def publish_diag(self, level, choice):
        msg = DiagnosticArray()
        stat = DiagnosticStatus()
        msg.status.append(stat)

        stat.level = level
        stat.name = 'EtherCAT Master'
        stat.message = choice 
     
        self.diag_pub.publish(msg)

        halted = Bool()
        halted.data = level != 0
        self.motors_pub.publish(halted)
            

class FakeTestApp(wx.App):
    def __init__(self):
        wx.App.__init__(self, clearSigInt = False)

    def OnInit(self):
        rospy.init_node("fake_test_node")
        self._frame = FakeTestFrame(None)
        self._frame.SetSize(wx.Size(224, 230))
        self._frame.Layout()
        self._frame.Centre()
        self._frame.Show(True)

        signal.signal(signal.SIGINT, signal.SIG_DFL)

        return True

if __name__ == '__main__':
    try:
        app = FakeTestApp()
        app.MainLoop()
    except:
        print 'Caught exception in FakeTest!'
        import traceback
        traceback.print_exc()
        rospy.logerr(traceback.format_exc())
