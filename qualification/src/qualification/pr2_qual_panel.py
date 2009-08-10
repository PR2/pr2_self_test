#!/usr/bin/env python
#
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

### Author: Kevin Watts

import roslib
import roslib.packages
roslib.load_manifest('qualification')

import rospy

import os
import sys
from datetime import datetime
import wx
import time
from wx import xrc
from wx import html

from xml.dom import minidom

from qualification.test import *
from qualification.qual_frame import *

import traceback

ONBOARD_DIR = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'onboard')

class PR2TestTreeItem(object):
  def __init__(self, subtest, tree_id):
    self.subtest = subtest
    self.tree_id = tree_id

## Used to pass robot name to pr2_qual system
class PR2TestRobot(QualTestObject):
  def __init__(self, name, start_pkg, startup, test_file, serial):
    QualTestObject.__init__(self, name, serial)
    self.start_pkg = start_pkg
    self.startup = startup
    self.test_file = test_file

## PR2 Qualification only
class PR2QualPanel(wx.Panel):
  ##@param parent wxPanel: Parent frame
  ##@param resource: XRC resource to load panel
  ##@param qualification_frame: Qualfication frame to pass test, serial to
  def __init__(self, parent, resource, qualification_frame):
    wx.Panel.__init__(self, parent)
    
    self._manager = qualification_frame
    
    self._res = resource

    self.load_robots_from_file()

    self._panel = resource.LoadPanel(self, 'pr2_qual_panel')
    self._sizer = wx.BoxSizer(wx.VERTICAL)

    self._sizer.Add(self._panel, 1, wx.EXPAND|wx.ALIGN_CENTER_VERTICAL)
    self.SetSizer(self._sizer)
    self.Layout()
    self.Fit()

    self._tree_control = xrc.XRCCTRL(self._panel, 'test_tree_ctrl')

    self._robot_box = xrc.XRCCTRL(self._panel, 'robot_list_box')
    robot_list = dict.keys(self._robots)
    robot_list.sort()

    self._robot_box.InsertItems(robot_list, 0)
    self._robot_box.Bind(wx.EVT_LISTBOX_DCLICK, self.load_onboards_from_file)
    
    self._robot_qual_button = xrc.XRCCTRL(self._panel, 'robot_qual_button')
    self._robot_qual_button.Bind(wx.EVT_BUTTON, self.on_onboard_test)

  ## Loads file which contains robots and all tests for each robot
  ## qualification/onboard/robots.xml
  def load_robots_from_file(self):
    robot_xml_path = os.path.join(ONBOARD_DIR, 'robots.xml')
    self._robots = {}
    try:
      doc = minidom.parse(robot_xml_path)
    except IOError:
      print >> sys.stderr, "Could not load robots from '%s'"%(robots_xml_path)
      sys.exit()

    bots = doc.getElementsByTagName('robot')
    for bot in bots:
      try:
        name = bot.attributes['name'].value
        start_pkg = bot.attributes['start_pkg'].value
        startup = bot.attributes['startup'].value
        test_file = bot.attributes['test_file'].value
        serial = bot.attributes['serial'].value
        
        self._robots[name] = PR2TestRobot(name, start_pkg, startup, test_file, serial)
      except:
        rospy.logerr('Caught exception parsing robots file.\n%s' % traceback.format_exc())
        
  ## Loads qualification tests from XML file specified by selected robot.
  def load_onboards_from_file(self, event):
    # Add robot startup file to test list
    robot_name = self._robot_box.GetStringSelection()
    if robot_name is None or robot_name == '':
      wx.MessageBox('No robot selected', 'No Robot')
      return
      
    self._test_robot = self._robots[robot_name]

    # Load onboard tests directory
    onboard_xml_path = os.path.join(ONBOARD_DIR, self._test_robot.test_file)
    self._onboards = {}
    try:
      doc = minidom.parse(onboard_xml_path)
    except IOError:
      rospy.logerr("Could not load onboard description from '%s'"%(onboard_xml_path))
      return
      
    self._tree_control.DeleteAllItems()

    self._onboard_root_name = 'PR2 Qualification'
    self._root_id = self._tree_control.AddRoot(self._onboard_root_name)
    item = PR2TestTreeItem(None, self._root_id)
    self._tree_control.SetPyData(self._root_id, item)
    self._onboard_key_count = 0
    
    self.insert_onboards_from_xml(doc, 'robot', self._root_id)

  ## Keys make onboard qual tests load in order
  def get_onboard_key(self):
    self._onboard_key_count = self._onboard_key_count + 1
    return self._onboard_key_count

  ## Loads tests into wxTreeCtrl
  def insert_onboards_from_xml(self, branch_xml_doc, parent_name, parent_id):
    on_bds = branch_xml_doc.getElementsByTagName(parent_name)
    for node in on_bds:
      node_key = self.get_onboard_key()

      label = node.attributes['label'].value
 
      node_attrs = node.attributes.keys()

      test         = None
      pre_test     = None
      post_test    = None
      timeout      = -1
      subTest      = None
      
      if 'pre' in node_attrs:
        pre_test = os.path.join(ONBOARD_DIR, node.attributes['pre'].value)
      
      if 'post' in node_attrs:
        post_test = os.path.join(ONBOARD_DIR, node.attributes['post'].value)

      if 'timeout' in node_attrs:
        timeout = int(node.attributes['timeout'].value)

      if 'test' in node_attrs:
        test = os.path.join(ONBOARD_DIR, node.attributes['test'].value)
        subTest = SubTest(test, node_key, label, timeout, pre_test, post_test)

      # Add to tree, store item data
      id = self._tree_control.AppendItem(parent_id, label)
      item = PR2TestTreeItem(subTest, id)
      self._tree_control.SetPyData(id, item)

      if parent_id == self._root_id:
        self._tree_control.Expand(id)

      if 'child_name' in node_attrs:
        child_name = node.attributes['child_name'].value      
        self.insert_onboards_from_xml(node, child_name, id)

  ## Prompts the user before running onboard qualification tests
  def verify_onboard_test(self):
    onboard_check = 'Are you sure you want to run this test?\n\n'
    onboard_check += 'This will shut down everything on the robot.\n'
    onboard_check += 'Make sure you have all your tests selected.\n\n'
    onboard_check += 'The wheels must be off the ground for caster tests.\n\n\n'
    onboard_check += 'DO NOT PROCEED IF YOU ARE NOT READY.'

    are_you_sure = wx.MessageDialog(self, onboard_check, 'Confirm Onboard Selection',
                                    wx.OK|wx.CANCEL)
    return are_you_sure.ShowModal() == wx.ID_OK

  ## Returns the subtests of any children in the tree control
  ##@param id: wxTreeItemId of selection or other tree item
  ##@param subtests: List of subtests to run, any subtests found added to this list
  def get_child_subtests(self, id, subtests):
    if not self._tree_control.ItemHasChildren(id):
      return 

    sibling, cookie = self._tree_control.GetFirstChild(id)
    
    while not rospy.is_shutdown():
      item = self._tree_control.GetPyData(sibling)
      if item.subtest:
        subtests.append(item.subtest)
      self.get_child_subtests(sibling, subtests)

      sibling, cookie = self._tree_control.GetNextChild(id, cookie)
      label = self._tree_control.GetItemText(sibling)
      if label == '': # Empty item
        break
      
  ## User presses the "Start" button. 
  ## Load tests and send item to manager
  def on_onboard_test(self, event):
    try:
      # Get selections returns tree items that haven't been added
      selections = self._tree_control.GetSelections() # returns wxTreeItemIds

      subtests = []
      
      robot_name = self._robot_box.GetStringSelection()
      
      if not robot_name or robot_name == '' or len(selections) == 0 or not self._robots.has_key(robot_name):
        wx.MessageBox('No subtests or no robot. Select robot and tests and retry.', 'No subtests', wx.OK|wx.ICON_ERROR, self)
        return

      if not self.verify_onboard_test():
        return

      # Load subtests that are selected by tree
      for id in selections:
        item = self._tree_control.GetPyData(id)
        if (item == None):
          continue
        
        if item.subtest:
          subtests.append(item.subtest)

        self.get_child_subtests(id, subtests)

      robot = self._robots[robot_name]

      pkg = self._robots[robot_name].start_pkg
      file = self._robots[robot_name].startup
    
      onboard_test = Test()
      onboard_test.generate_onboard_test(pkg, file, robot_name)
      onboard_test.add_subtests(subtests)

      self._manager.begin_test(onboard_test, robot)
    except Exception, e:
      traceback.print_exc()
      wx.MessageBox('Unable to load onboard test for that selection.','Error - Invalid test selection', wx.OK|wx.ICON_ERROR, self)

## Subclass of QualificationFrame, for PR2 qualification only
class PR2QualFrame(QualificationFrame):
  def __init__(self, parent):
    QualificationFrame.__init__(self, parent)

    self.create_menubar()

  ## Returns PR2QualPanel to load qualification tests
  def get_loader_panel(self):
    return PR2QualPanel(self._top_panel, self._res, self)

  def create_menubar(self):
    menubar = wx.MenuBar()
    self._file_menu = wx.Menu()
    self._file_menu.Append(1001, "Invent Login\tCTRL+l")
    self._file_menu.Append(wx.ID_EXIT, "E&xit")
    menubar.Append(self._file_menu, "&File")

    self._options_menu = wx.Menu()
    self._options_menu.AppendCheckItem(2001, "Always Show Results")
    menubar.Append(self._options_menu, "&Options")

    self.SetMenuBar(menubar)
    self.Bind(wx.EVT_MENU, self.on_menu)

  def on_menu(self, event):
    if (event.GetEventObject() == self._file_menu):
      if (event.GetId() == wx.ID_EXIT):
        self.Close()
        return
      if (event.GetId() == 1001):
        self.login_to_invent()
        return

    if (event.GetEventObject() == self._options_menu):
      if (event.GetId() == 2001):
        self._show_results_always = self._options_menu.IsChecked(2001)

## Starts roscore, qualification app for PR2. 
class PR2QualificationApp(wx.App):
  def OnInit(self):
    try:
      self._core_launcher = roslaunch_caller.launch_core()
    except Exception, e:
      sys.stderr.write('Failed to launch core. Another core may already be running.\n\n')
      traceback.print_exc()
      sys.exit(5)

    rospy.init_node("Qualification")
    
    self._frame = PR2QualFrame(None)
    self._frame.SetSize(wx.Size(700,1000))
    self._frame.Layout()
    self._frame.Centre()
    self._frame.Show(True)
    
    return True

  def OnExit(self):
    self._core_launcher.stop()
