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

import roslib
roslib.load_manifest('life_test')

import wx

import life_test.ui
import socket, sys, os

def get_robot_name():
  while True:
    robot = str(wx.GetTextFromUser('Enter robot computer name (ex: "prk")', 'Select Robot', ''))
    
    if robot == '':
      dlg = wx.MessageDialog(None, 'Press "Cancel" to abort or "OK" to retry.', 'Unable to set Robot', wx.OK|wx.CANCEL)
      if dlg.ShowModal() != wx.ID_OK:
        return None
      else:
        continue

    try:
      machine_addr = socket.gethostbyname(robot)
      return robot
    except:
      dlg = wx.MessageDialog(None, 'Unable to set robot name to "%s". Press OK to retry or Cancel to abort', 'Invalid Robot Name', wx.OK|wx.CANCEL)
      if dlg.ShowModal() != wx.ID_OK:
        return None

  return None

class PR2TestManagerApp(wx.App):
  def OnInit(self, debug = False):
    args = rospy.myargv()
    debug = len(args) > 1 and args[1] == '--debug'

    # Ask for robot name here
    robot = get_robot_name()
    if not robot:
      print >> sys.stderr "Unable to set robot name"
      sys.exit(-1)

    os.environ['ROS_MASTER_URI'] = 'http://%s:11311' % robot

    rospy.init_node("Test_Manager")
    self._frame = life_test.ui.TestManagerFrame(None, debug)
    self._frame.SetSize(wx.Size(1600, 1100))
    self._frame.Layout()
    self._frame.Centre()
    self._frame.Show(True)



if __name__ == '__main__':
  try:
    app = PR2TestManagerApp(0)
    app.MainLoop()
  except Exception, e:
    print "Caught exception in TestManagerApp Main Loop"
    import traceback
    traceback.print_exc()
