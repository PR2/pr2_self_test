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

## Author: Kevin Watts

PKG = 'life_test'
import roslib
roslib.load_manifest(PKG)

import rospy
import roslaunch
import roslaunch.pmon

import socket
import os
import sys
import datetime
import glob
import wx
import time
import traceback
import threading
from wx import xrc
from wx import html

import threading
from xml.dom import minidom

from invent_client.invent_client import Invent, is_serial_valid

from life_test import *
from test_param import *
from test_bay import *

from msg import TestInfoArray

from pr2_power_board.srv import PowerBoardCommand
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Empty

from roslaunch_caller import roslaunch_caller 

class TestManagerFrame(wx.Frame):
    def __init__(self, parent, debug = False):
        wx.Frame.__init__(self, parent, wx.ID_ANY, "Test Manager")
        
        self._mutex = threading.Lock()

        self.create_menu_bar()

        self._debug = debug

        # Load tests by short serial number
        self._tests_by_serial = {}

        # Load XRC
        xrc_path = os.path.join(roslib.packages.get_pkg_dir('life_test'), 'xrc/gui.xrc')
        self._xrc = xrc.XmlResource(xrc_path)

        # Get real username/password for invent here...
        self._invent_client = None 
        
        self._active_bays = []
        self._active_boards = {}

        self.load_tests_from_file()
        self.load_rooms_from_file()

        self._main_panel = self._xrc.LoadPanel(self, 'manager_panel')

        self._tab_ctrl = xrc.XRCCTRL(self._main_panel, 'test_tab_control')
        self._start_panel = xrc.XRCCTRL(self._tab_ctrl, 'start_panel')
        self._serial_text = xrc.XRCCTRL(self._start_panel, 'serial_text')
        self._start_button = xrc.XRCCTRL(self._start_panel, 'start_button')
        self._start_button.Bind(wx.EVT_BUTTON, self.on_start)

        self._log_text = xrc.XRCCTRL(self._start_panel, 'log_text')

        self.Bind(wx.EVT_CLOSE, self.on_close)
        self._serial_text.SetFocus()
        
        # Start panel...
        self._test_panels = {}
        self._active_serials = []

        self._current_log = []

        self._diag_sub = rospy.Subscriber('/diagnostics', DiagnosticArray, self._diag_cb)
        self._diags = []

        self._power_node = None
        self._power_cmd = rospy.ServiceProxy('power_board/control', PowerBoardCommand)

        self.log('Started Test Manager')

        self._info_pub = rospy.Publisher('test_info', TestInfoArray)

        self._heartbeat_pub = rospy.Publisher('/heartbeat', Empty)

        self._info_timer = wx.Timer(self, 1)
        self.Bind(wx.EVT_TIMER, self.on_info_timer, self._info_timer)
        self._info_timer.Start(500)

        self._heartbeat_timer = wx.Timer()
        self._heartbeat_timer.Bind(wx.EVT_TIMER, self.on_heartbeat_timer)
        self._heartbeat_timer.Start(1000)

    def shutdown(self):
        if self._power_node is not None:
            self._power_node.shutdown()
            self._power_node = None
        
    def __del__(self):
        self.shutdown()


    def on_info_timer(self, event):
        array = TestInfoArray()
        for serial, panel in self._test_panels.iteritems():
            array.data.append(panel.on_status_check())
        self._info_pub.publish(array)

    def on_heartbeat_timer(self, event):
        beat = Empty()
        self._heartbeat_pub.publish(beat)

    def _diag_cb(self, msg):
        self._mutex.acquire()
        try:
            self._diags.append(msg)
        except:
            rospy.logerr(traceback.format_exc())
            self._mutex.release()


        self._mutex.release()
        wx.CallAfter(self._new_diag)
        
    ##\brief Look for known power boards in diagnostics, update panels
    ##
    ##\todo This is causing deadlocks. Added timeout, see if it helps.
    ## Josh suggested try/except block in case exception is causing problem. Will see
    ## if that helps.
    def _new_diag(self):
        timeout = 0.5
        start = rospy.get_time()
        while not self._mutex.acquire(False):
            sleep(0.1)
            rospy.log_info('Waiting for mutex for /diagnostics')
            if rospy.get_time() - start > timeout:
                rospy.logwarn('Timeout for mutex for /diagnostics exceeded.')
                return

        try:
            # Search for power board messages. If we have them, update the status in the viewer
            for msg in self._diags:
                for status in msg.status:
                    if status.name.startswith("Power board"):
                        board_sn = int(status.name.split()[2])
                        if self._active_boards.has_key(board_sn):
                            runstop = False
                            runbutton = False
                            for val in status.values:
                                if val.key == "RunStop Button Status":
                                    runbutton = (val.value == "True")
                                if val.key == "RunStop Status":
                                    runstop = (val.value == "True")
                            estop = runstop and runbutton
                            
                            for val in status.values:
                                if val.key.startswith("Breaker "):
                                    for breaker in self._active_boards[board_sn].keys():
                                        if val.key == "Breaker %d State" % breaker:
                                            panel = self._active_boards[board_sn][breaker]
                                            self._test_panels[panel].update_board(val.value, estop)

            self._diags = []
        except:
            rospy.logerr(traceback.format_exc())
            self._mutex.release()

        self._mutex.release()


                

    def load_test(self, test, serial):
        # Make notebook page for panel
        # Add panel to notebook
        panel = wx.Panel(self._tab_ctrl, wx.ID_ANY)

        index = len(self._active_serials) + 1

        self._tab_ctrl.AddPage(panel, test.get_title(serial), True)

        test_panel = TestMonitorPanel(panel, self, test, serial)
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(test_panel, 1, wx.EXPAND)
        panel.SetSizer(sizer)
        panel.Fit()
        panel.Layout()
        
        self._test_panels[serial] = test_panel        
        self._active_serials.append(serial)

    def close_tab(self, serial):
        if not serial in self._active_serials:
            rospy.logerr('Requested serial not in active serials: %s' % serial)
            return

        idx = self._active_serials.index(serial)

        if self._tab_ctrl.DeletePage(idx + 1):
            #self._test_panels[serial].shutdown()
            del self._test_panels[serial]
            del self._active_serials[idx]
        
    ##\todo Needs to be fixed to only load boards for tests that need it.
    def test_start_check(self, bay, serial):
        if bay in self._active_bays:
            return False
        else:
            self._active_bays.append(bay)

            if bay.board is not None:
                if not self._active_boards.has_key(bay.board):
                    self._active_boards[bay.board] = {}
                self._active_boards[bay.board][bay.breaker] = serial

            if bay.board is not None and self._power_node is None:
                self._power_node = roslaunch_caller.ScriptRoslaunch('<launch><node pkg="pr2_power_board" type="power_node" name="power_board" /></launch>')
                self._power_node.start()

            return True

    def test_stop(self, bay):
        if bay in self._active_bays:
            idx = self._active_bays.index(bay)
            del self._active_bays[idx]

        if bay.board is not None:
            del self._active_boards[bay.board][bay.breaker]
            if len(self._active_boards[bay.board].keys()) == 0:
                del self._active_boards[bay.board]

        if len(self._active_boards.keys()) == 0 and self._power_node is not None:
            self._power_node.shutdown()
            self._power_node = None

    # Power commands
    def _reset_power_disable(self, bay):
        try:
            rospy.wait_for_service('power_board/control', 5)
            resp = self._power_cmd(bay.board, bay.breaker, 'reset', 0)
            if resp.retval != 0:
                rospy.logerr('Failed to reset board %s, breaker %d. Retval: %s' % (bay.board, bay.breaker, retval))
                return False

            time.sleep(1)
            return True
        except:
            rospy.logerr(traceback.format_exc())
            return False


    def power_run(self, bay):
        try:
            if not self._reset_power_disable(bay):
                return False

            resp = self._power_cmd(bay.board, bay.breaker, 'start', 0)
            return resp.retval == 0
        except:
            rospy.logerr(traceback.format_exc())
            return False

    def power_standby(self, bay):
        try:
            if not self._reset_power_disable(bay):
                return False

            resp = self._power_cmd(bay.board, bay.breaker, 'stop', 0)
            return resp.retval == 0
        except:
            return False

    def power_disable(self, bay):
        try:
            rospy.wait_for_service('power_board/control', 5)
            resp = self._power_cmd(bay.board, bay.breaker, 'disable', 0)
            return resp.retval == 0
        except:
            return False
             
    def load_invent_client(self):
        # Loads invent client. 
        # Like singleton, doesn't construct unless valid login
        if self._invent_client:
            return True

        invent = None
        dialog = self._xrc.LoadDialog(self, 'username_password_dialog')
        xrc.XRCCTRL(dialog, 'login_text').Wrap(300)
        dialog.Layout()
        dialog.Fit()
        username_ctrl = xrc.XRCCTRL(dialog, 'username')
        password_ctrl = xrc.XRCCTRL(dialog, 'password')
        username_ctrl.SetFocus()
        
        username_ctrl.SetMinSize(wx.Size(200, -1))
        password_ctrl.SetMinSize(wx.Size(200, -1))

        while not rospy.is_shutdown():
            if (dialog.ShowModal() == wx.ID_OK):
                username = username_ctrl.GetValue()
                password = password_ctrl.GetValue()
            
                invent = Invent(username, password)
                if (invent.login() == False):
                    wx.MessageBox('Please enter a valid username and password.',
                                  'Valid Login Required', wx.OK|wx.ICON_ERROR, 
                                  self)
                else:
                    self._invent_client = invent
                    return True
            else:
                return False # User doesn't want to log in

        return False
      
 
    def on_start(self, event):
        serial = self._serial_text.GetValue()
        
        if serial in self._active_serials:
            wx.MessageBox('Current component is already testing.', 'Already Testing', wx.OK|wx.ICON_ERROR, self)
            return

        if (not self._debug) and (not is_serial_valid(serial)):
            wx.MessageBox('Serial number "%s" appears to be invalid. Re-enter the serial number and try again. Debug: %s' % (serial, str(self._debug)), 'Invalid Serial Number', wx.OK|wx.ICON_ERROR, self)
            return



        test = self.select_test(serial)

        # If test is none, display message box and return
        if not test:
            wx.MessageBox('No test defined for that serial number or no test selected. Please try again.', 'No test', wx.OK|wx.ICON_ERROR, self)
            return
        
        if not self._debug and not self.load_invent_client():
            wx.MessageBox('You cannot proceed without a valid inventory login.', 'Valid Login Required', wx.OK|wx.ICON_ERROR, self)
            return 
        elif self._debug and not self.load_invent_client():
            wx.MessageBox('Warning: Without inventory access, a lot of things won\'t really work.',
                          'No Inventory Access', wx.OK|wx.ICON_ERROR, self)
            self._invent_client = Invent('', '')
            


        if (not self._debug) and (not self._invent_client.get_test_status(serial)):
            wx.MessageBox('Component %s has not passed qualification. Please re-test component and try again' % serial, 
                          'Bad Component', wx.OK|wx.ICON_ERROR, self)
            return

        self._serial_text.Clear()
        self.log('Starting test %s' % test._name)

        self.load_test(test, serial)

    ## Loads locations for tests
    def load_rooms_from_file(self):
        filepath = os.path.join(roslib.packages.get_pkg_dir(PKG), 'wg_test_rooms.xml')

        rooms = {}

        try:
            doc = minidom.parse(filepath)
            rooms_xml = doc.getElementsByTagName('room')
            for room_xml in rooms_xml:
                hostname = room_xml.attributes['hostname'].value
                room = TestRoom(hostname)
                for bay in room_xml.getElementsByTagName('bay'):
                    room.add_bay(TestBay(bay))
                rooms[hostname] = room

            self._rooms = rooms
        except:
            traceback.print_exc()
            wx.MessageBox('Unable to load test rooms and bays information from %s. Check the file and try again' % filepath,
                          'Unable to load rooms', wx.OK|wx.ICON_ERROR, self)
            return

        if len(self._rooms.keys()) == 0:
            wx.MessageBox('No test rooms found in %s. Check the file and try again' % filepath,
                          'Unable to load rooms', wx.OK|wx.ICON_ERROR, self)
            return
        
        if self._rooms.has_key(socket.gethostname()):
            self.room = self._rooms[socket.gethostname()]
        else:
            self.room = room # Last room
            
        

    # Loads tests from XML file
    def load_tests_from_file(self, test_xml_path =os.path.join(roslib.packages.get_pkg_dir('life_test'), 'tests.xml')):
        my_tests = {}

        try:
            doc = minidom.parse(test_xml_path)
        except IOError:
            rospy.logerr('Could not load tests from %s' % test_xml_path)
            sys.exit()

        try:
            tests = doc.getElementsByTagName('test')
            for test in tests:
                serial = test.attributes['serial'].value # Short serial only
                name = test.attributes['name'].value
                desc = test.attributes['desc'].value
                script = test.attributes['script'].value
                type = test.attributes['type'].value
                trac = test.attributes['trac'].value
                short = test.attributes['short'].value
                power = test.attributes['power'].value != 'false'
                
                # Add test parameters
                # Make param from XML element
                # Append to list, add to test
                test_params = []
                params_xml = test.getElementsByTagName('param')
                for param_xml in params_xml:
                    p_name = param_xml.attributes['name'].value
                    p_param_name = param_xml.attributes['param_name'].value
                    p_desc = param_xml.attributes['desc'].value
                    p_val = param_xml.attributes['val'].value
                    
                    p_rate = param_xml.attributes['rate'].value == 'true'


                    test_params.append(TestParam(p_name, p_param_name, 
                                                 p_desc, p_val, p_rate))

                        
                life_test = LifeTest(serial, name, short, trac, 
                                     desc, type, script, power, test_params)

                if my_tests.has_key(serial):
                    my_tests[serial].append(life_test)
                else:
                    my_tests[serial] = [ life_test ]
                    
            self._tests = my_tests
        except:
            traceback.print_exc()
            rospy.logerr('Caught exception parsing test XML.')
            wx.MessageBox('Unable to load test from file %s. Check the file and try again.' % test_xml_path,
                          'Unable to load', wx.OK|wx.ICON_ERROR, self)

    ##\brief Pop-up GUI that lets users select which test to load
    def select_string_from_list(self, msg, lst):
        # Load robot selection dialog
        dialog = self._xrc.LoadDialog(self, 'select_test_dialog')
        select_text = xrc.XRCCTRL(dialog, 'select_text')
        select_text.SetLabel(msg)
        test_box = xrc.XRCCTRL(dialog, 'test_list_box')
        test_box.InsertItems(lst, 0)
        
        select_text.Wrap(250)
        
        dialog.Layout()
        dialog.Fit()
        
        # return string of test folder/file to run
        if (dialog.ShowModal() == wx.ID_OK):
            desc = test_box.GetStringSelection()
            dialog.Destroy()
            return desc
        else: 
            dialog.Destroy()
            return None

    def select_test(self, serial):
        short_serial = serial[0:7]
        
        if not self._tests.has_key(short_serial):
            return None
        if len(self._tests[short_serial]) == 1:
            return self._tests[short_serial][0]
        
        # Load select_test_dialog
        tests_by_name = {}
        for test in self._tests[short_serial]:
            tests_by_name[test._name] = test
      
        msg = 'Select test to run.'

        descrips = dict.keys(tests_by_name)
        descrips.sort()
        
        choice = self.select_string_from_list(msg, descrips)
        if choice is None:
            return None
        return tests_by_name[choice]

    def log(self, msg):
        time_str = strftime("%m/%d/%Y %H:%M:%S: ", localtime(rospy.get_time()))

        self._current_log.append(time_str + msg)
        self.update_log_display()

    def log_test_entry(self, test_name, machine, message):
        if not machine:
            machine = 'NONE'

        time_str = strftime("%m/%d/%Y %H:%M:%S: ", localtime(rospy.get_time()))

        log_msg = time_str + 'Machine %s, Test %s. Message: %s' % (machine, test_name, message)

        self._current_log.append(log_msg)
        self.update_log_display()

    def update_log_display(self):
        for log in self._current_log:
            self._log_text.AppendText(log + '\n')
        self._current_log = []

    def on_close(self, event):
        if event.CanVeto() and len(self._test_panels) > 0:
                wx.MessageBox('Unable to close Test Manager. All component windows must be shut down first.', 
                              'Unable to close', wx.OK|wx.ICON_ERROR, self)
                event.Veto()
                return

        # Could just delete monitors
        for value in dict.values(self._test_panels):
            value.on_close(None)
            #value.shutdown()

        self.Destroy()
        
    ## Add stuff for invent login, loading test bays
    def on_menu(self, event):
        if (event.GetEventObject() == self._file_menu):
            if (event.GetId() == wx.ID_EXIT):
                self.Close()
                return
            if (event.GetId() == 1001):
                self._invent_client = None
                self.load_invent_client()
                return

        if (event.GetEventObject() == self._tests_menu):
            if (event.GetId() == 2001):
                self.load_tests_from_file()
                return
            if (event.GetId() == 2002):
                self.load_rooms_from_file()
                return

    def create_menu_bar(self):
        menubar = wx.MenuBar()
        
        # file menu
        self._file_menu = wx.Menu()
        self._file_menu.Append(1001, "Invent Login\tCTRL+l")
        self._file_menu.Append(wx.ID_EXIT, "E&xit")
        menubar.Append(self._file_menu, "&File")
                
        self._tests_menu = wx.Menu()
        self._tests_menu.Append(2001, "Reload tests (tests.xml)")
        self._tests_menu.Append(2002, "Reload rooms and bays (wg_test_rooms.xml)")
        menubar.Append(self._tests_menu, "&Tests")

        self.SetMenuBar(menubar)
        self.Bind(wx.EVT_MENU, self.on_menu)


class TestManagerApp(wx.App):
    def OnInit(self, debug = False):

        args = rospy.myargv()
        debug = len(args) > 1 and args[1] == '--debug'

        self._core_launcher = roslaunch_caller.launch_core()

        rospy.init_node("Test_Manager")
        self._frame = TestManagerFrame(None, debug)
        self._frame.SetSize(wx.Size(1600, 1100))
        self._frame.Layout()
        self._frame.Centre()
        self._frame.Show(True)

        return True

    def OnExit(self):
        self._core_launcher.stop()

