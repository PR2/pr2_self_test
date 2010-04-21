#!/usr/bin/env python
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
#


## Prompts the user with a Y/N message if MCB's aren't configured
# Called from configure/program_mcb.py through the mcb_conf_results service

PKG = 'qualification' 
NAME = 'confirm_conf'

import roslib
roslib.load_manifest(PKG) 
import wx 
from wx import xrc
from qualification.srv import ConfirmConf, ConfirmConfRequest, ConfirmConfResponse
import rospy 
import time
import os

import signal

prev_handler = None


app = wx.PySimpleApp(clearSigInt = True)
process_done = False
prompt_done = False
prompt_click =False
frame=wx.Frame(None)

#def shutdown(sig, stackframe):
#    rospy.logerr('confirm conf shutting down')
#    frame.Close()
#    app.Destroy()
#    if prev_handler is not None:
#        prev_handler(signal.SIGINT, None)
  
 
def msg_detail_prompt(msg, details):
  # Load MCB conf dialog box from gui.xrc
  xrc_path = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'xrc/gui.xrc')
  xrc_resource = xrc.XmlResource(xrc_path)
  # See if it works to set a parent
  dialog = xrc_resource.LoadDialog(None, 'confirm_conf_dialog')
  # Set text in message text
  xrc.XRCCTRL(dialog, 'message_text').SetLabel(msg)
  xrc.XRCCTRL(dialog, 'message_text').Wrap(400)  
  xrc.XRCCTRL(dialog, 'detail_text').AppendText(details)

  global prompt_click, prompt_done
  
  dialog.Layout()
  #dialog.Fit()

  prompt_click = (dialog.ShowModal() == wx.ID_OK)
  prompt_done = True
  dialog.Destroy()
   
def check_w_user(req):
  global prompt_done, prompt_click

  wx.CallAfter(msg_detail_prompt, req.message, req.details)

  while(not prompt_done and not rospy.is_shutdown()):
    rospy.loginfo("Waiting for retry prompt . . .")
    for i in range(0, 20):
      time.sleep(0.5) 

  resp = ConfirmConfResponse()
  if prompt_click:
    resp.retry = ConfirmConfResponse.RETRY
  else:
    resp.retry = ConfirmConfResponse.FAIL

  prompt_done = False
  prompt_click = False
  return resp

 
def confirm_conf():
  rospy.init_node(NAME)
  s = rospy.Service('mcb_conf_results', ConfirmConf, check_w_user)  

  # Allow it to kill on exit w/o SIGKILL
  signal.signal(signal.SIGINT, signal.SIG_DFL)

  app.MainLoop()

  rospy.spin()
  time.sleep(1)
  

if __name__ == "__main__":  
  confirm_conf()
