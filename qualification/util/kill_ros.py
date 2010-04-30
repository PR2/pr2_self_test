#!/usr/bin/env python

PKG = 'qualification'

import wx
import sys, subprocess, os

import roslib; roslib.load_manifest(PKG)

from time import sleep

app = wx.PySimpleApp()

dialog = wx.MessageDialog(None, "Are you are you want to kill ROS? This will kill the Qualification system and Test Manager. Press \"Cancel\" to abort.", "Confirm Kill ROS", wx.OK|wx.CANCEL)
if dialog.ShowModal() != wx.ID_OK:
    sys.exit(1)

#cmd = [ 'kkill' ]

p = subprocess.Popen('sudo kkill', stdout = subprocess.PIPE, 
                     stderr = subprocess.PIPE, shell=True)
retcode = p.returncode

if retcode != 0:
    wx.MessageBox("Unable to kill ROS. Retry by pressing \"Kill ROS\".", "Unable to kill ROS", wx.OK)
else:
    wx.MessageBox("ROS processes killed. Close any remaining windows", "ROS Killed", wx.OK)

