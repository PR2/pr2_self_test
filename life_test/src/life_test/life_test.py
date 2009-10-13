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
##\brief Panel for starting, stopping and logging life tests 

import roslib
roslib.load_manifest('life_test')

import sys, os, math, string
import csv
import traceback
from time import sleep, strftime, localtime
import threading
from socket import gethostname

import wx
from wx import xrc

import rospy

from std_srvs.srv import * # Empty

# Stuff from life_test package
from msg import TestStatus, TestInfo
from test_param import TestParam, LifeTest
from test_record import TestRecord

import runtime_monitor
from runtime_monitor.monitor_panel import MonitorPanel

import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.base import MIMEBase
from email import Encoders

import roslaunch
from roslaunch_caller import roslaunch_caller 
        

class TestMonitorPanel(wx.Panel):
    def __init__(self, parent, manager, test, serial):
        wx.Panel.__init__(self, parent)

        self._manager = manager

        self._mutex = threading.Lock()

        self._status_sub = None
        self._diags = []
        
        # Set up test and loggers
        self._test = test
        self._serial = serial
        self._record = TestRecord(test, serial)

        # Set up panel
        xrc_path = os.path.join(roslib.packages.get_pkg_dir('life_test'), 'xrc/gui.xrc')

        self._panel = xrc.XmlResource(xrc_path).LoadPanel(self, 'test_panel')
        self._test_desc = xrc.XRCCTRL(self._panel, 'test_desc')
        self._test_desc.SetValue(self._test._desc)

        self._launch_button = xrc.XRCCTRL(self._panel, 'launch_test_button')
        self._launch_button.Bind(wx.EVT_BUTTON, self.start_stop_test)

        self._test_bay_ctrl = xrc.XRCCTRL(self._panel, 'test_bay_ctrl')
        self._test_bay_ctrl.SetItems(self._manager.room.get_bay_names(self._test.needs_power()))
        
        self._end_cond_type = xrc.XRCCTRL(self._panel, 'end_cond_type')
        self._end_cond_type.SetStringSelection('Continuous')
        self._end_cond_type.Bind(wx.EVT_CHOICE, self.on_end_choice)

        self._end_cond_type_label = xrc.XRCCTRL(self._panel, 'duration_label')

        self._test_duration_ctrl = xrc.XRCCTRL(self._panel, 'test_duration_ctrl')
        
        self._close_button = xrc.XRCCTRL(self._panel, 'close_button')
        self._close_button.Bind(wx.EVT_BUTTON, self.on_close)
        
        self._status_bar = xrc.XRCCTRL(self._panel, 'test_status_bar')

        # Pause and reset
        self._reset_button = xrc.XRCCTRL(self._panel, 'reset_motors_button')
        self._reset_button.Bind(wx.EVT_BUTTON, self.on_reset_motors)

        self._halt_button = xrc.XRCCTRL(self._panel, 'halt_motors_button')
        self._halt_button.Bind(wx.EVT_BUTTON, self.on_halt_motors)

        # Logs and operator input
        self._user_log = xrc.XRCCTRL(self._panel, 'user_log_input')
        self._user_submit = xrc.XRCCTRL(self._panel, 'user_submit_button')
        self._user_submit.Bind(wx.EVT_BUTTON, self.on_user_entry)

        self._done_time_ctrl = xrc.XRCCTRL(self._panel, 'done_time_ctrl')
        
        self._elapsed_time_ctrl = xrc.XRCCTRL(self._panel, 'elapsed_time_ctrl')
        self._active_time_ctrl = xrc.XRCCTRL(self._panel, 'active_time_ctrl')

        self._log_ctrl = xrc.XRCCTRL(self._panel, 'test_log')

        self._test_log_window = xrc.XRCCTRL(self._panel, 'test_log_window')
        self._send_log_button = xrc.XRCCTRL(self._panel, 'send_test_log_button')
        self._send_log_button.Bind(wx.EVT_BUTTON, self.on_send_test_log)

        # Power control
        self._power_board_text = xrc.XRCCTRL(self._panel, 'power_board_text')
        self._power_board_text.SetBackgroundColour("White")
        self._power_board_text.SetValue("Test Not Running")

        self._estop_status = xrc.XRCCTRL(self._panel, 'estop_status')
        self._estop_status.SetValue("Test Not Running")

        self._power_run_button = xrc.XRCCTRL(self._panel, 'power_run_button')
        self._power_run_button.Bind(wx.EVT_BUTTON, self.on_power_run)

        self._power_standby_button = xrc.XRCCTRL(self._panel, 'power_standby_button')
        self._power_standby_button.Bind(wx.EVT_BUTTON, self.on_power_standby)

        self._power_disable_button = xrc.XRCCTRL(self._panel, 'power_disable_button')
        self._power_disable_button.Bind(wx.EVT_BUTTON, self.on_power_disable)

        
        # Bay data
        self._power_sn_text = xrc.XRCCTRL(self._panel, 'power_sn_text')
        self._power_breaker_text = xrc.XRCCTRL(self._panel, 'power_breaker_text')
        self._machine_text = xrc.XRCCTRL(self._panel, 'machine_text')

        # Add runtime to the panel...
        self._notebook = xrc.XRCCTRL(self._panel, 'test_data_notebook')
        wx.CallAfter(self.create_monitor)

        self._sizer = wx.BoxSizer(wx.HORIZONTAL)
        self._sizer.Add(self._panel, 1, wx.EXPAND)
        self.SetSizer(self._sizer)
        self.Layout()
        
        self._bay = None
        self._current_log = {}
        self._diag_msgs = {}

        self._is_running = False
        self._stat_level = 127 # Not launched
        self._test_msg = 'None'
        self._power_stat = "No data"
        self._estop_stat = False
        self._stop_count = 0

        # Launches test, call stop to kill it
        self._test_launcher = None

        # Test log data
        self._test_complete = False

        # Timeout for etherCAT diagnostics, starts when test launched
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.last_message_time = rospy.get_time()
        self.timeout_interval = 10.0
        self._is_stale = True

        # Timeout for powerboard status, starts if power comes up
        self.power_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_power_timer, self.power_timer)
        self.last_power_update = rospy.get_time()
        self.power_timeout = 5.0
        

        # Timer for invent logging
        self.invent_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_invent_timer, self.invent_timer)
        self._last_invent_time = rospy.get_time()
        self.invent_timeout = 600
        self.invent_timer.Start(self.invent_timeout * 500)
        self._is_invent_stale = True
        self._invent_note_id = None
 
        self.update_controls()
        self.on_end_choice()


    def create_monitor(self):
        # Give blank topic to prevent unregistering problems (ROS #1702)
        self._monitor_panel = MonitorPanel(self._notebook, 'none')
        self._monitor_panel.SetSize(wx.Size(400, 500))
        self._notebook.AddPage(self._monitor_panel, "Runtime Monitor")

    def __del__(self):
        if self.is_launched():
            self.stop_test()

        if self._status_sub is not None:
            self._status_sub.unregister()
        
    def is_launched(self):
        return self._test_launcher is not None

    def on_end_choice(self, event = None):
        choice = self._end_cond_type.GetStringSelection()

        # Set test_duration_ctrl units also
        label = choice.lower()
        if choice == 'Continuous':
            label = 'N/A'
        self._end_cond_type_label.SetLabel(choice.lower())

        # Set spin ctrl based on end type
        self._test_duration_ctrl.SetValue(0)
        active_time = self._record.get_cum_time()

        # Should probably add cycles back in somehow
        if choice == 'Hours':
            hrs = math.ceil((active_time / 3600))
            self._test_duration_ctrl.SetRange(hrs, 168) # Week
            self._test_duration_ctrl.SetValue(hrs)
        elif choice == 'Minutes':
            min = math.ceil((active_time / 60))
            self._test_duration_ctrl.SetRange(min, 600) # 10 Hrs
            self._test_duration_ctrl.SetValue(min + 10)
        else:
            self._test_duration_ctrl.SetRange(0, 0) # Can't change limits
            self._test_duration_ctrl.SetValue(0)

    def on_user_entry(self, event):
        entry = self._user_log.GetValue()
        msg = 'OPERATOR: ' + entry
        self.update_test_record(msg)
        self._user_log.Clear()
        self._user_log.SetFocus()

    def on_send_test_log(self, event):
        names = wx.GetTextFromUser('Enter recipient names, separated by commas: NAME1,NAME2 (without "@willowgarage.com").', 'Enter recipient', '', self)

        names = names.split(',')
        for name in names:
            if name.find('@') < 0:
                name = name + '@willowgarage.com'

        self.notify_operator(3, 'Log Requested.', string.join(names, ','))

    def on_close(self, event):
        try:
            self.update_test_record('Closing down test.')
            self.update_invent()
            self.record_test_log()
            self.notify_operator(1, 'Closing.')
        except:
            rospy.logerr('Exception on close: %s' % traceback.format_exc())

        self._manager.close_tab(self._serial)

    ##\brief Updates record, notifies operator if necessary
    def update_test_record(self, note = ''):
        alert, msg = self._record.update(self.is_launched(), self._is_running, self._is_stale, 
                                         note, self._test_msg)

        lst = [msg, note]
        message = string.join(lst, ' ')

        if alert > 0 or note != '':
            self._current_log[rospy.get_time()] = message
            if self._bay is not None:
                self._manager.log_test_entry(self._test._name, self._bay.name, message)
            else:
                self._manager.log_test_entry(self._test._name, 'None', message)
            self.display_logs()

        if alert > 0:
            self.notify_operator(alert, msg)


    def calc_run_time(self):
        end_condition = self._end_cond_type.GetStringSelection()
        
        duration = self._test_duration_ctrl.GetValue()

        if end_condition == 'Hours':
            return duration * 3600
        if end_condition == 'Minutes':
            return duration * 60
        if end_condition == 'Seconds':
            return duration
        else: #if end_condition == 'Continuous':
            return 10**10 # Roughly 300 years

    def calc_remaining(self):
        total_sec = self.calc_run_time()
        cum_sec = self._record.get_cum_time()
    
        return total_sec - cum_sec
        

    def on_invent_timer(self, event):
        if rospy.get_time() - self._last_invent_time > self.invent_timeout and self.is_launched():
            self.update_invent()
        
    def update_invent(self):
        self.invent_timer.Start()

        self._last_invent_time = rospy.get_time()

        # Don't log anything if we haven't launched
        if not self.is_launched() and not self._invent_note_id:
            return

        hrs_str = self._record.get_active_str()

        stats = "Stats: Total active time %s." % (hrs_str)
        
        if self.is_launched() and self._is_running:
            note = "Test running: %s. " % (self._test._name)
        elif self.is_launched() and not self._is_running:
            note = "%s is halted. " % self._test._name
        else:
            note = "%s finished. CSV name: %s. " % (self._test._name, os.path.basename(self._record.csv_filename()))

        self._invent_note_id = self._manager._invent_client.setNote(self._serial, note + stats, self._invent_note_id)

    # Should also be in notifier classs
    def record_test_log(self):
        try:
            # Adds log csv to invent
            if self._record.get_cum_time() == 0:
                return # Don't log test that hasn't run
                        
            f = open(self._record.csv_filename(), 'rb')
            csv_file = f.read()
            self._manager._invent_client.add_attachment(self._serial, os.path.basename(self._record.csv_filename()), 'text/csv', csv_file)
            f.close()
            
            summary_name = strftime("%m%d%Y_%H%M%S", localtime(self._record._start_time)) + '_summary.html'
            self._manager._invent_client.add_attachment(self._serial, summary_name, 'text/html', self.make_html_test_summary())
        except Exception, e:
            rospy.logerr('Unable to submit to invent. %s' % traceback.format_exc())
                                                
        
    def start_timer(self):
        self.timer.Start(1000 * self.timeout_interval, True)
        
    def on_timer(self, event):
        interval = rospy.get_time() - self.last_message_time
        
        if interval > self.timeout_interval: 
            # Make EtherCAT status stale
            self._is_running = False
            self._is_stale = True

            self.update_controls(4)
            self.update_test_record()
            self.stop_if_done()
        else:
            self._is_stale = False

    def on_status_check(self):
        msg = TestInfo()
        msg.serial = str(self._serial)
        msg.test_name = str(self._test._short)
        if self.is_launched():
            msg.test_status = int(self._stat_level)
            msg.bay_name = str(self._bay.name)
            msg.machine = str(self._bay.machine)
            if self._bay.board is not None and self._test.needs_power():
                msg.board = self._bay.board
                msg.breaker = self._bay.breaker
                msg.power_status = self._power_stat
                if self._estop_stat:
                    msg.estop = 1
                else:
                    msg.estop = 0
            else:
                msg.board = 0
                msg.breaker = 0
                                
        else:
            msg.test_status = 127
            msg.bay_name = "None"
            msg.board = 0
            msg.breaker = 0
            msg.machine = ""

        msg.elapsed = self._record.get_cum_time()
        msg.status_msg = self._test_msg
        
        return msg        

    def _test_power_cmd(self):
        if not self.is_launched():
            wx.MessageBox('Test is not launched. Unable to command power board', 'Test is not launched', wx.OK|wx.ICON_ERROR, self)
            return False

        if self._bay.board is None:
            wx.MessageBox('Test bay has no power board. Unable to command power board', 'No Power Board',
                          wx.OK|wx.ICON_ERROR, self)
            return False
        return True        

    def on_power_run(self, event):
        self.update_test_record("Turning on power.")
        if not self._test_power_cmd():
            return

        if not self._manager.power_run(self._bay):
            self.update_test_record("Unable to command power board.")
            wx.MessageBox('Unable to run power board. Board: %d, breaker: %d' % (self._bay.board, self._bay.breaker), wx.OK|wx.ICON_ERROR, self)

    def on_power_standby(self, event):
        self.update_test_record("Setting power to standby.")
        if not self._test_power_cmd():
            return
        
        if not self._manager.power_standby(self._bay):
            self.update_test_record("Unable to command power board.")
            wx.MessageBox('Unable to standby power board. Board: %d, breaker: %d' % (self._bay.board, self._bay.breaker), wx.OK|wx.ICON_ERROR, self)

    def on_power_disable(self, event):
        self.update_test_record("Setting power to disable.")
        if not self._test_power_cmd():
            return

        if not self._manager.power_disable(self._bay):
            self.update_test_record("Unable to command power board.")
            wx.MessageBox('Unable to disable power board. Board: %d, breaker: %d' % (self._bay.board, self._bay.breaker), wx.OK|wx.ICON_ERROR, self)

    def start_power_timer(self):
        self.power_timer.Start(1000 * self.power_timeout, True)

    def on_power_timer(self, event = None):
        if rospy.get_time() - self.last_power_update > self.power_timeout:
            self.update_board("Stale", False)

    ##\brief Updates power board control with status
    def update_board(self, value, estop):
        if not self.is_launched():
            return

        self._power_stat = value
        self._estop_stat = estop

        self.start_power_timer()

        if value == "Stale":
            self._power_board_text.SetBackgroundColour("Light Blue")
            self._power_board_text.SetValue("Stale")
            self._estop_status.SetBackgroundColour("Light Blue")
            self._estop_status.SetValue("E-Stop Stale")
            return

        elif value == "Standby":
            self._power_board_text.SetBackgroundColour("Orange")
            self._power_board_text.SetValue("Standby")
        elif value == "On":
            self._power_board_text.SetBackgroundColour("Light Green")
            self._power_board_text.SetValue("Running")
        else:
            self._power_board_text.SetBackgroundColour("Red")
            self._power_board_text.SetValue("Disabled")

        if not estop:
            self._estop_status.SetBackgroundColour("Red")
            self._estop_status.SetValue("E-Stop Hit")
        else:
            self._estop_status.SetBackgroundColour("Light Green")
            self._estop_status.SetValue("E-Stop OK")
            

    ##\brief Updates test status bar
    def _update_status_bar(self, level, msg):        
        if not self.is_launched():
            self._status_bar.SetValue("Launch to display status")
            self._status_bar.SetBackgroundColour("White")
        elif level == 0:
            self._status_bar.SetValue("Test Running: OK")
            self._status_bar.SetBackgroundColour("Light Green")
        elif level == 1:
            self._status_bar.SetValue("Test Warning! Warning: %s" % msg)
            self._status_bar.SetBackgroundColour("Orange")
        elif level == 2:
            self._status_bar.SetValue("Error in Test Monitor: %s" % msg)
            self._status_bar.SetBackgroundColour("Red")
        elif level == 3:
            self._status_bar.SetValue("Test Monitor reports stale: %s" % msg)
            self._status_bar.SetBackgroundColour("Light Blue")
        else:
            self._status_bar.SetBackgroundColour("Light Blue")
            self._status_bar.SetValue("Test Monitor Stale")        

    ##\brief Called after status message or timer callback
    def update_controls(self, level = 4, msg = 'None'):
        self._update_status_bar(level, msg)

        # These are updated here instead of the callback, because of the stale timer
        self._stat_level = level
        self._test_msg = msg

        ##\todo FIX
        remaining = self.calc_remaining()
        remain_str = "N/A" 
        if remaining < 10**6:
            remain_str = self._record.get_duration_str(remaining)
        self._done_time_ctrl.SetValue(remain_str)

        self._active_time_ctrl.SetValue(self._record.get_active_str())
        self._elapsed_time_ctrl.SetValue(self._record.get_elapsed_str())
        
    ##\brief Set pause/reset and power buttons on or off
    def _enable_controls(self):
        self._reset_button.Enable(self.is_launched())
        self._halt_button.Enable(self.is_launched())

        self._test_bay_ctrl.Enable(not self.is_launched())
        self._close_button.Enable(not self.is_launched())

        # Power buttons
        self._power_run_button.Enable(self.is_launched() and self._bay.board is not None)
        self._power_standby_button.Enable(self.is_launched() and self._bay.board is not None)
        self._power_disable_button.Enable(self.is_launched() and self._bay.board is not None)        
        
    
    def display_logs(self):
        kys = dict.keys(self._current_log)
        kys.sort()

        log_str = ''
        for ky in kys:
            log_str += strftime("%m/%d/%Y %H:%M:%S: ", 
                                localtime(ky)) + self._current_log[ky] + '\n'

        self._log_ctrl.AppendText(log_str)

        # Test log control in second panel
        self._test_log_window.Freeze()
        (x, y) = self._test_log_window.GetViewStart()
        self._test_log_window.SetPage(self.make_html_cycle_log_table())
        self._test_log_window.Scroll(x, y)
        self._test_log_window.Thaw()

        self._current_log = {}

    def stop_if_done(self):
        remain = self.calc_remaining()
        
        if remain < 0:
            self._stop_count += 1
        else:
            self._stop_count = 0

        # Make sure we've had five consecutive seconds of 
        # negative time before we shutdown
        # Can this be part of test record?
        if self._stop_count < 5 and not self._test_complete:
            self._test_complete = True
            self.stop_test()
            self._enable_controls()
        
    def status_callback(self, msg):
        self._mutex.acquire()
        self._status_msg = msg
        self._mutex.release()
        wx.CallAfter(self.new_msg)

    def new_msg(self):
        self._mutex.acquire()

        level_dict = { 0: 'OK', 1: 'Warn', 2: 'Error', 3: 'Stale' }

        test_level = self._status_msg.test_ok
        test_msg = self._status_msg.message

        self._mutex.release()

        self._is_running = (self._status_msg.test_ok == 0)
        self._is_stale = False

        self.start_timer()

        self.update_controls(test_level, test_msg)
        self.update_test_record()
        self.stop_if_done()



     
    
    def make_launch_script(self, bay, script, local_diag_topic):
        launch = '<launch>\n'
        launch += '<group ns="%s" >' % bay.name

        # Remap
        launch += '<remap from="/diagnostics" to="%s" />' % local_diag_topic
         
        # Init machine
        # Root on remote 
        launch += '<machine name="test_host_root" user="root" address="%s" ' % bay.machine
        launch += 'ros-root="$(env ROS_ROOT)" ros-package-path="$(env ROS_PACKAGE_PATH)" default="never"/>'

        # Default: User on remote
        launch += '<machine name="test_host" address="%s" default="true" ' % bay.machine
        launch += 'ros-root="$(env ROS_ROOT)" ros-package-path="$(env ROS_PACKAGE_PATH)"  />'
        
        # Local host
        launch += '<machine name="localhost" address="localhost" '
        launch += 'ros-root="$(env ROS_ROOT)" ros-package-path="$(env ROS_PACKAGE_PATH)" default="false"/>'

        # Include our launch file
        launch += '<include file="$(find life_test)/%s" />' % script

        # Rosrecord launches and records local diagnostics
        launch += ' <node machine="localhost" pkg="rosrecord" type="rosrecord" args="-f /hwlog/%s_life_test /diagnostics" name="test_logger" />' % self._serial
        
        launch += '</group>\n</launch>'

        return launch


    # Put in master file
    # Add subscriber to diagnostics
    # Launch file, subscribe diagnostics
    def start_stop_test(self, event):
        if self.is_launched():
            if not self.stop_test_user():
                return
        else:
            if not self.launch_test():
                return

        self.update_controls()
        self._enable_controls()

    ##\brief Called when user presses "stop" button
    def stop_test_user(self):
        dialog = wx.MessageDialog(self, 'Are you sure you want to stop this test?', 'Confirm Stop Test', wx.OK|wx.CANCEL)
        if dialog.ShowModal() != wx.ID_OK:
            return False

        self.stop_test()
        return True

    def stop_tes(self):
        if self.is_launched():
            self._launch_button.Enable(False)
            if self._bay.board is not None:
                if not self._manager.power_disable(self._bay):
                    wx.MessageBox("Power disable command failed. Unable to command power board", "Power command failed", wx.OK|wx.ICON_ERROR, self)
                    
            self._power_board_text.SetBackgroundColour("White")
            self._power_board_text.SetValue("Test Not Running")

            self._estop_status.SetBackgroundColour("White")
            self._estop_status.SetValue("Test Not Running")

            # Machine, board stats
            self._machine_text.SetValue("Not running")
            self._power_sn_text.SetValue("Not running")
            self._power_breaker_text.SetValue("Not running")
            self.on_halt_motors(None)
            self._test_launcher.shutdown()
            self._manager.test_stop(self._bay)
            self.update_test_record('Stopping test launch')
            self._is_running = False
            self._test_launcher = None  
            self._launch_button.Enable(True)
            self._launch_button.SetLabel("Launch")

        if self._status_sub:
            self._status_sub.unregister()
        self._status_sub = None

        self._is_running = False


    def launch_test(self):
        dialog = wx.MessageDialog(self, 'Are you sure you want to launch?', 'Confirm Launch', wx.OK|wx.CANCEL)
        if dialog.ShowModal() != wx.ID_OK:
            return False
        
        self._launch_button.Enable(False)

        bay_name = self._test_bay_ctrl.GetStringSelection()
        bay = self._manager.room.get_bay(bay_name)
        if bay is None:
            wx.MessageBox('Select test bay', 'Select Bay', wx.OK|wx.ICON_ERROR, self)
            self._launch_button.Enable(True)
            return False

        if not self._manager.test_start_check(bay, self._serial):
            wx.MessageBox('Test bay in use, select again!', 'Test bay in use', wx.OK|wx.ICON_ERROR, self)
            self._launch_button.Enable(True)
            return False
        
        self._bay = bay

        if self._bay.board is not None:
            if not self._manager.power_run(self._bay):
                wx.MessageBox('Unable to command power board. Check connections. Board: %s, Breaker %d' % (self._bay.board, self._bay.breaker), 'Power Command Failure', wx.OK|wx.ICON_ERROR, self)
                self._launch_button.Enable(True)
                self._manager.test_stop(self._bay)
                return False

        # Machine, board stats
        self._machine_text.SetValue(self._bay.machine)
        if self._bay.board is not None and self._test.needs_power():
            self._power_sn_text.SetValue(str(self._bay.board))
            self._power_breaker_text.SetValue(str(self._bay.breaker))
            self._power_board_text.SetValue("No data")
            self._power_board_text.SetBackgroundColour("Light Blue")
            self._estop_status.SetValue("No data")
            self._estop_status.SetBackgroundColour("Light Blue")
            self._power_stat = "No data"
            self._estop_stat = False
        else:
            self._power_sn_text.SetValue("No board")
            self._power_breaker_text.SetValue("No breaker")
            self._power_board_text.SetValue("No board")
            self._power_board_text.SetBackgroundColour("White")
            self._estop_status.SetBackgroundColour("White")
            self._estop_status.SetValue("No board")

        local_diag = '/' + self._bay.name + '/diagnostics'

        self.update_test_record('Launching test %s on bay %s, machine %s.' % (self._test._name, self._bay.name, self._bay.machine))

        # Clear namespace
        rospy.set_param(self._bay.name, {})
        self._test.set_params(self._bay.name)
        self._test_launcher = roslaunch_caller.ScriptRoslaunch(
            self.make_launch_script(self._bay, self._test._launch_script, local_diag))
        try:
            self._test_launcher.start()
        except roslaunch.RLException, e:
            traceback.print_exc()
            self._manager.test_stop(self._bay)
            self._bay = None
            self.update_test_record('Failed to launch script, caught exception.')
            self.update_test_record(traceback.format_exc())
            self._test_launcher.shutdown()
            self._test_launcher = None
            self._power_sn_text.SetValue("N/A")
            self._power_breaker_text.SetValue("N/A")
            self._power_board_text.SetValue("No board")
            self._power_board_text.SetBackgroundColour("White")
            self._estop_status.SetBackgroundColour("White")
            self._estop_status.SetValue("No board")

            wx.MessageBox('Failed to launch. Check machine for connectivity. See log for error message.', 'Failure to launch', wx.OK|wx.ICON_ERROR, self)
            self._launch_button.Enable(True)
            return False

        local_status = '/' + self._bay.name + '/test_status'
        self._is_running = True
        self.update_invent()
        self._monitor_panel.change_diagnostic_topic(local_diag)

        self.update_controls()

        self._status_sub = rospy.Subscriber(local_status, TestStatus, self.status_callback)
        self._launch_button.Enable(True)
        self._launch_button.SetLabel("Stop")
        return True
        
    def on_halt_motors(self, event = None):
        try:
            self.update_test_record('Pausing test.')
            halt_srv = rospy.ServiceProxy(self._bay.name + '/halt_test', Empty)
            halt_srv()

        except Exception, e:
            rospy.logerr('Exception on halt test.\n%s' % traceback.format_exc())

    def on_reset_motors(self, event = None):
         try:
             self.update_test_record('Resetting test.')
             reset = rospy.ServiceProxy(self._bay.name + '/reset_test', Empty)
             reset()

         except:
            rospy.logerr('Exception on reset test.\n%s' % traceback.format_exc())
      
    # 
    # Loggers and data processing -> Move to notifier class or elsewhere
    # Notifier class needs test record, that's it
    # 
    def get_test_team(self):
        # HACK!!! Don't email everyone if it's debugging on NSF
        if os.environ['USER'] == 'watts' and gethostname() == 'nsf':
            return 'watts@willowgarage.com'

        return 'test.team@lists.willowgarage.com'

    def line_summary(self, msg):
        if self._bay is not None:
            machine_str = self._bay.name
        else:
            machine_str = 'NONE'
        return "Test %s on bay %s. MSG: %s" % (self._test.get_title(self._serial), machine_str, msg)

    def notify_operator(self, level, alert_msg, recipient = None):
        # Don't notify if we haven't done anything
        if self._record.get_cum_time() == 0 and not self.is_launched() and level == 1:
            return

        sender = 'test.notify@willowgarage.com'
        if level == 2:
            sender = 'test.alerts@willowgarage.com'
        elif level == 3:
            sender = 'test.reports@willowgarage.com'
        
        try:
            if recipient is None:
                recipient = self.get_test_team()

            msg = MIMEMultipart('alternative')
            msg['Subject'] = self.line_summary(alert_msg)
            msg['From'] = sender
            msg['To'] = recipient

            msg.attach(MIMEText(self.make_html_test_summary(alert_msg), 'html'))
            
            log_csv = open(self._record.csv_filename(), 'rb')
            log_data = log_csv.read()
            log_csv.close()

            part = MIMEBase('application', 'octet-stream')
            part.set_payload(log_data)
            Encoders.encode_base64(part)
            part.add_header('Content-Disposition', 'attachment; filename="%s"' 
                            % os.path.basename(self._record.csv_filename()))
            
            msg.attach(part)

            s = smtplib.SMTP('localhost')
            s.sendmail(sender, recipient, msg.as_string())
            s.quit()

            return True
        except Exception, e:
            rospy.logerr('Unable to send mail! %s' % traceback.format_exc())
            self.update_test_record('Unable to send mail! %s' % traceback.format_exc())
            return False


    def make_html_cycle_log_table(self):
        log_csv = csv.reader(open(self._record.csv_filename(), 'rb'))
        
        is_first = True

        html = '<html>\n<table border="1" cellpadding="2" cellspacing="0">\n'
        for row in log_csv:
            html += '<tr>'
            for val in row:  
                if unicode(val).isnumeric():
                    val = "%.2f" % float(val)
                
                if is_first:
                    html += '<td><b>%s</b></td>' % val
                else:
                    html += '<td>%s</td>' % val

            is_first = False
            html += '</tr>\n'

        html += '</table>\n</html>'

        return html

    # Dump these into test_result class of some sort
    def make_html_test_summary(self, alert_msg = ''):
        html = '<html><head><title>Test Log: %s of %s</title>' % (self._test._name, self._serial)
        html += '<style type=\"text/css\">\
body { color: black; background: white; }\
div.error { background: red; padding: 0.5em; border: none; }\
div.warn { background: orange: padding: 0.5em; border: none; }\
div.pass { background: green; padding: 0.5em; border: none; }\
strong { font-weight: bold; color: red; }\
em { font-style:normal; font-weight: bold; }\
</style>\
</head>\n<body>\n'

        html += '<H2 align=center>Test Log: %s of %s</H2>\n' % (self._test._name, self._serial)
        
        if alert_msg != '':
            html += '<H3>Alert: %s</H3><br>\n' % alert_msg

        if self._test_complete:
            html += '<H3>Test Complete</H3>\n'
        else:
            if self.is_launched() and not self._is_running:
                html += '<H3>Test Status: Launched, Halted</H3>\n'
            elif self.is_launched() and self._is_running:
                html += '<H3>Test Status: Launched, Running</H3>\n'
            else:
                html += '<H3>Test Status: Shutdown</H3>\n'

        html += '<H4>Current Message: %s</H4>\n' % str(self._test_msg)

        # Table of test bay, etc
        html += '<hr size="3">\n'
        html += '<H4>Test Info</H4>\n'
        html += '<p>Description: %s</p>\n<br>' % self._test._desc
        html += self.make_test_info_table()

        # Parameter table
        html += '<hr size="3">\n'
        html += '<H4>Test Parameters</H4>\n'
        html += self.make_test_param_table()

        # Make results table
        html += '<hr size="3">\n'
        html += '<H4>Test Results</H4>\n'
        html += self.make_record_table()
        

        # Make log table
        html += '<hr size="3">\n'
        html += '<H4>Test Log</H4>\n'
        html += self.make_log_table()
        html += '<hr size="3">\n'
        html += '</body></html>'

        return html

    def make_test_info_table(self):
        html = '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += self.make_table_row(['Test Name', self._test._name])
        if self._bay:
            html += self.make_table_row(['Test Bay', self._bay])
            html += self.make_table_row(['Machine', self._bay.machine])
            html += self.make_table_row(['Powerboard', self._bay.board])
            html += self.make_table_row(['Breaker', self._bay.breaker])

        
        html += self.make_table_row(['Serial', self._serial])
        html += self.make_table_row(['Test Type', self._test._test_type])
        html += self.make_table_row(['Launch File', self._test._launch_script])
        html += self.make_table_row(['Trac Ticket', self._test._trac])
        html += '</table>\n'

        return html

    def make_test_param_table(self):
        if len(self._test._params) == 0:
            return '<p>No test parameters defined.</p>\n'

        html = '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += self.make_table_row(['Name', 'Value', 'Key', 'Description'], True)
        for param in self._test._params:
            html += self.make_table_row([param._name, param._value, param._param_name, param._desc])
        html += '</table>\n'

        return html


    def make_table_row(self, lst, bold = False):
        html = '<tr>'
        for val in lst:
            if bold:
                html += '<td><b>%s</b></td>' % val
            else:
                html += '<td>%s</td>' % val
        html += '</tr>\n'
        return html
    

    def make_record_table(self):
        if not self._record:
            return '<p>No test record, test may have been aborted.</p>\n'
            
        html = '<table border="1" cellpadding="2" cellspacing="0">\n'
        time_str = strftime("%m/%d/%Y %H:%M:%S", localtime(self._record._start_time))
        html += self.make_table_row(['Start Time', time_str])
        html += self.make_table_row(['Elapsed Time', self._record.get_elapsed_str()])
        html += self.make_table_row(['Active Time', self._record.get_active_str()])
        for ky in self._record.get_cum_data().keys():
            cum_name = "Cum. %s" % ky
            html += self.make_table_row([cum_name, self._record.get_cum_data()[ky]])        

        html += self.make_table_row(['Num Halts', self._record._num_halts])
        html += self.make_table_row(['Num Alerts', self._record._num_events])
        html += '</table>\n'

        return html

    def make_log_table(self):
        if self._record._log is None or len(dict.keys(self._record._log)) == 0:
            return '<p>No test log!</p>\n'

        html = '<p>CSV location: %s on machine %s.</p>\n' % (self._record.csv_filename(), gethostname())

        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Time</b></td><td><b>Entry</b></td></tr>\n'
        
        kys = dict.keys(self._record._log)
        kys.sort()
        for ky in kys:
            time_str = strftime("%m/%d/%Y %H:%M:%S", localtime(ky))
            html += self.make_table_row([time_str, self._record._log[ky]])
            
        html += '</table>\n'

        return html
