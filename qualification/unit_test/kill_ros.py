#!/usr/bin/env python

import wx
import sys, subprocess

app = wx.PySimpleApp()

dialog = wx.MessageDialog(None, "Are you are you want to kill ROS? This will kill the Qualification system and Test Manager", "Confirm Kill ROS", wx.OK|wx.CANCEL)
if dialog.ShowModal() != wx.ID_OK:
    sys.exit(1)

cmd = ['ckill', 'kill', '--regex=.*qual.*', '--sig=9']

retcode = subprocess.call(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE)

if retcode != 0:
    wx.MessageBox("Unable to kill ROS. Retry by pressing \"Kill ROS\".", "Unable to kill ROS", wx.OK)
else:
    wx.MessageBox("ROS processes killed. Close any remaining windows", "ROS Killed", wx.OK)

