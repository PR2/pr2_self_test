#!/usr/bin/env python

import wx
import sys, subprocess

import roslib; roslib.load_manifest('qualification')

app = wx.PySimpleApp()

dialog = wx.MessageDialog(None, "Are you are you want to kill ROS? This will kill the Qualification system and Test Manager. Press \"Cancel\" to abort.", "Confirm Kill ROS", wx.OK|wx.CANCEL)
if dialog.ShowModal() != wx.ID_OK:
    sys.exit(1)

whitelist_path = os.path.join(roslib.packages.get_pkg_dir('ckill'), 'whitelist')
cmd = ['rosrun', 'ckill', 'ckill.py', 'kill', '--sig=9', '--whitelist=%s' % whitelist_path]

retcode = subprocess.call(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE)

if retcode != 0:
    wx.MessageBox("Unable to kill ROS. Retry by pressing \"Kill ROS\".", "Unable to kill ROS", wx.OK)
else:
    wx.MessageBox("ROS processes killed. Close any remaining windows", "ROS Killed", wx.OK)

