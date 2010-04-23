#!/usr/bin/env python

"""
Kills all ROS processes. Used as utilty on qual station

Script must be run as root, or:
 * 'ckill kill --regex=.*ros.* must be able to run as user
 * ckill needs to be modified not to check userid=0
 * Command needs to be added to sudoers file
"""


import wx
import sys, subprocess

app = wx.PySimpleApp()

dialog = wx.MessageDialog(None, "Are you are you want to kill ROS? This will kill the Qualification system and Test Manager", "Confirm Kill ROS", wx.OK|wx.CANCEL)
if dialog.ShowModal() != wx.ID_OK:
    sys.exit(1)

cmd = ['ckill', 'kill', '--regex=.*production_qual_trees.*', '--sig=9']

retcode = subprocess.call(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE)

if retcode != 0:
    wx.MessageBox("Unable to kill ROS. Retry by pressing \"Kill ROS\".", "Unable to kill ROS", wx.OK)
else:
    wx.MessageBox("ROS processes killed. Close any remaining windows", "ROS Killed", wx.OK)

