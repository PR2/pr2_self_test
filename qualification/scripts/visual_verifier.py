#!/usr/bin/python
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

##\author Josh Faust

import os
import sys

WXVER = '2.8'
import wxversion
if wxversion.checkInstalled(WXVER):
  wxversion.select(WXVER)
else:
  print >> sys.stderr, "This application requires wxPython version %s"%(WXVER)
  sys.exit(1)

import wx

PKG = 'qualification'
import roslib
roslib.load_manifest(PKG)

from optparse import OptionParser
import shutil
import glob
import traceback

import roslib.packages
import rospy

import rviz
import ogre_tools

from pr2_self_test_msgs.srv import ScriptDone, ScriptDoneRequest

from time import sleep

import threading

SRV_NAME = 'visual_check'
def call_done_service(result, msg):
  result_service = rospy.ServiceProxy(SRV_NAME, ScriptDone)
  r = ScriptDoneRequest()
  r.result = result
  r.failure_msg = msg
  try:
    rospy.wait_for_service(SRV_NAME, 5)
    result_service.call(r)
  except Exception, e:
    print >> sys.stderr, "Unable to call %s service. Make sure the service is up" % SRV_NAME
    

# Calls the "OK" service after a timeout
class VisualRunner(threading.Thread):
  def __init__(self, app, timeout = None):
    threading.Thread.__init__(self)
    self.app = app
    
  def run(self):
    start = rospy.get_time()
    if timeout is None or timeout < 0:
      return
    while not rospy.is_shutdown():
      if rospy.get_time() - start > timeout:
        wx.CallAfter(self.app.frame.Close)
        call_done_service(ScriptDoneRequest.RESULT_OK, 'Visual check passed by automatic runner.')
        break
      sleep(1.0)


class VisualizerFrame(wx.Frame):
  def __init__(self, parent, id=wx.ID_ANY, title='Qualification Visualizer', pos=wx.DefaultPosition, size=(800, 600), style=wx.DEFAULT_FRAME_STYLE):
    wx.Frame.__init__(self, parent, id, title, pos, size, style)
    
    ogre_tools.initializeOgre()
    
    visualizer_panel = rviz.VisualizationPanel(self)
    
    self._visualizer_panel = visualizer_panel
    manager = visualizer_panel.getManager()
    
    self.Bind(wx.EVT_CLOSE, self.on_close)
    
    main_sizer = wx.BoxSizer(wx.VERTICAL)
    
    top_sizer = wx.BoxSizer(wx.VERTICAL)
    main_sizer.Add(top_sizer, 1, wx.EXPAND, 5)
    
    bottom_sizer = wx.BoxSizer(wx.HORIZONTAL)
    main_sizer.Add(bottom_sizer, 0, wx.ALIGN_RIGHT|wx.EXPAND, 5)
    
    top_sizer.Add(visualizer_panel, 1, wx.EXPAND, 5)
    
    self._instructions_ctrl = wx.TextCtrl(self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_MULTILINE|wx.TE_READONLY );
    self._pass_button = wx.Button(self, wx.ID_ANY, "Pass")
    self._fail_button = wx.Button(self, wx.ID_ANY, "Fail")
    bottom_sizer.Add(self._instructions_ctrl, 1, wx.ALL, 5)
    bottom_sizer.Add(self._pass_button, 0, wx.ALL|wx.ALIGN_BOTTOM, 5)
    bottom_sizer.Add(self._fail_button, 0, wx.ALL|wx.ALIGN_BOTTOM, 5)
    
    self.SetSizer(main_sizer)
    self.Layout()
    
    self._pass_button.Bind(wx.EVT_BUTTON, self.on_pass)
    self._fail_button.Bind(wx.EVT_BUTTON, self.on_fail)
    
          
  def on_close(self, event):
    self.Destroy()
    
  def on_pass(self, event):
    call_done_service(ScriptDoneRequest.RESULT_OK, 'Visual check passed by operator.')
    self.Destroy()

  def on_fail(self, event):
    call_done_service(ScriptDoneRequest.RESULT_FAIL, 'Visual check failed by operator.')
    self.Destroy()
      
  def load_config_from_path(self, path):
    manager = self._visualizer_panel.getManager()
    manager.removeAllDisplays()
    self._visualizer_panel.loadGeneralConfig(path)
    self._visualizer_panel.loadDisplayConfig(path)
    
  def set_instructions(self, instructions):
    self._instructions_ctrl.SetValue(instructions)
      
  def on_open(self, event):
    dialog = wx.FileDialog(self, "Choose a file to open", self._save_location, wildcard="*."+self._CONFIG_EXTENSION, style=wx.FD_OPEN)
    if dialog.ShowModal() == wx.ID_OK:
      path = dialog.GetPath()
      self.load_config_from_path(path)

class VisualizerApp(wx.App):
  def __init__(self, file):
    self._filepath = file
    self._instructions = 'Move joints and verify robot is OK.'
    
    wx.App.__init__(self)
  
  def OnInit(self):
    try:
      self.frame = VisualizerFrame(None, wx.ID_ANY, "Visual Verifier", wx.DefaultPosition, wx.Size( 800, 600 ) )
    
      if (not os.path.exists(self._filepath)):
        call_done_service(ScriptDoneRequest.RESULT_ERROR, 'Visual check recorded error, file does not exist!')
        rospy.logerr('Visual check recorded error, file does not exist!')
        return False
    
      self.frame.load_config_from_path(self._filepath)
      self.frame.set_instructions(self._instructions)
      self.frame.Show(True)
      return True
    except:
      traceback.print_exc()
      rospy.logerr('Error initializing rviz: %s' % traceback.format_exc())
      call_done_service(ScriptDoneRequest.RESULT_ERROR, 'Visual check recorded error on initialization: %s' % traceback.format_exc())
      return False

  def OnExit(self):
    ogre_tools.cleanupOgre()

if __name__ == "__main__":
  if (len(sys.argv) < 2):
    call_done_service(ScriptDoneRequest.RESULT_ERROR, "Visual verifier not given path to config file.\nusage: visual_verifier.py 'path to config file'")
    print >> sys.stderr, 'Usage: visual_verifier.py \'path to config file\''
    sys.exit(1)

  if not os.path.exists(sys.argv[1]):
    call_done_service(ScriptDoneRequest.RESULT_ERROR, "Visual verifier not given path to actual config file.\nusage: visual_verifier.py 'path to config file'\nFile %s does not exist" % sys.argv[1])
    print >> sys.stderr, 'Usage: visual_verifier.py \'path to config file\'.\nGiven file does not exist'
    sys.exit(1)
  
  rospy.init_node('visual_verifier')

  try:
    app = VisualizerApp(sys.argv[1])

    # Uses this timeout to automatically pass visual verifier
    # Used in automated testing.
    timeout = rospy.get_param('visual_runner_timeout', -1)
    if timeout > 0:
      runner = VisualRunner(app, timeout)
      runner.start()

    app.MainLoop()

    rospy.spin()
  except:
    call_done_service(ScriptDoneRequest.RESULT_ERROR, 'Visual check recorded error: %s' % traceback.format_exc())
    rospy.logerr(traceback.format_exc())
