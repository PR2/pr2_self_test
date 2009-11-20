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

##\author Kevin Watts
##\brief Monitors dropped packets for given motors, and prompts user when drop test done

PKG='qualification'

import roslib; roslib.load_manifest(PKG)

import wx
from wx import xrc
from wx import html


from qualification.srv import TestResult, TestResultRequest

import rospy

from diagnostic_msgs.msg import DiagnosticArray

import threading

from optparse import OptionParser

import os

##\todo Handle SIGINT so it'll die nicely

def bool_to_msg(val):
    if val:
        return 'PASS'
    return 'FAIL'

##\brief Contains params, data for drops
class Drop:
    ##\param name str : Name of drop (ex: 'Side Drop')
    ##\param file str : Instructions file of drop
    ##\param count int : Number of times to complete drop
    def __init__(self, name, file, count):
        self.count = count
        self.file = file
        self.name = name

        self.packets = []
        self.halted = []
        for i in range(0, self.count):
            self.packets.append('N/A')
            self.halted.append('N/A')

        self._current = 0
    
    ##\brief Stores data from each drop
    def update(self, packets, halted):
        self.packets[self._current] = bool_to_msg(packets)
        self.halted[self._current] = bool_to_msg(halted)

        self._current += 1

##\brief Displays instructions for user to run test, makes sure part is working
class DropTestFrame(wx.Frame):
    def __init__(self, parent, test_name, pre, post, drops):
        wx.Frame.__init__(self, parent, wx.ID_ANY, 'Drop Test')
        
        self._mutex = threading.Lock()

        rospy.init_node('drop_tester')

        self._diag_sub = rospy.Subscriber('/diagnostics', DiagnosticArray, self._diag_cb)
        self.result_service = rospy.ServiceProxy('test_result', TestResult)
        
        self._msgs = []
        
        self._eth_master_ok = False
        self._drop_packets = -1

        xrc_path = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'xrc/gui.xrc')
        xrc_res = xrc.XmlResource(xrc_path)
        self._panel = xrc_res.LoadPanel(self, 'instructions_panel')
        self._sizer = wx.BoxSizer(wx.VERTICAL)
        self._sizer.Add(self._panel)
        self.Layout()

        self._html_window = xrc.XRCCTRL(self._panel, 'html_window')

        self._cancel_button = xrc.XRCCTRL(self._panel, 'cancel_button')
        self._cancel_button.Bind(wx.EVT_BUTTON, self.on_cancel)

        self._continue_button = xrc.XRCCTRL(self._panel, 'continue_button')
        self._continue_button.Bind(wx.EVT_BUTTON, self.on_continue)
        self._continue_button.SetFocus()
        
        self.data_sent = False

        self.predrop = pre
        self.postdrop = post
        self.drops = drops
        self.test_name = test_name

        self.complete = 0
        self.total = 0
        for drop in self.drops:
            self.total += drop.count

        self._current_drop = 0
        self._current_drop_count = 0
        
        self._state = -1 # -1: PRE, 0: DROP, 1: POST

        self._canceled = False

        self.start_test()
        
    ##\brief Proceed to next drop, or pass if done
    def on_continue(self, event):
        if self._state == -1:
            self._state += 1
            self.display_drop_structs()
        elif self._state == 0:
            self.check_drop()
        else: # PASS
            r = TestResultRequest()
            r.result = TestResultRequest.RESULT_PASS
            r.plots = []
            r.html_result = self._write_result()
            r.text_summary = self._write_summary()
            self.send_results(r)

    ##\brief Cancel test, report failure
    def on_cancel(self, event):
        self._canceled = True

        wx.MessageBox('Canceling drop test. Press OK to terminate.', 'Canceling drop test', wx.OK|wx.ICON_ERROR, self)
        r = TestResultRequest()
        r.result = TestResultRequest.RESULT_FAIL
        r.plots = []
        r.html_result = self._write_result('User pressed cancel.')
        r.text_summary = self._write_summary('Canceled.')
        self.send_results(r)
 
    ##\brief Error, send failure data to manager
    def test_error_service_call(self, except_str = ''):
        rospy.logerr(except_str)
        r = TestResultRequest()
        r.html_result = except_str
        r.text_summary = 'Caught exception, automated test failure.'
        r.plots = []
        r.result = TestResultRequest.RESULT_FAIL
        self.send_results(r)

    ##\brief Send results to qualification manager
    def send_results(self, test_result):
        if not self.data_sent:
            rospy.wait_for_service('test_result', 10)
            self.result_service.call(test_result)
            self.data_sent = True
        self.Destroy()

    ##\brief Callback for diagnostics msgs
    def _diag_cb(self, msg):
        self._mutex.acquire()

        # Record that we have data
        if self._drop_packets == -1:
            self._eth_master_ok = True
            self._drop_packets = 0

        for stat in msg.status:
            self._msgs.append(stat)
        self._mutex.release()
        wx.CallAfter(self._check_msgs)

    ##\brief Checks diagnostics for EtherCAT Master, makes sure OK
    def _check_msgs(self):
        self._mutex.acquire()
        for msg in self._msgs:
            if msg.name == 'EtherCAT Master':
                if msg.level != 0:
                    self._eth_master_ok = False
                for kv in msg.values:
                    if kv.key == 'Dropped Packets':
                        if int(kv.value) != 0:
                            self._drop_packets = int(kv.value)
        self._msgs = []
        self._mutex.release()

    ##\brief OK if no dropped packets and motors running
    def is_ok(self):
        return self._eth_master_ok and self._drop_packets == 0

    ##\brief Pass if OK and we've completed all drops w/o canceling
    def is_pass(self):
        return self.is_ok() and self._current_drop >= len(self.drops) and not self._canceled
    
    ##\brief Write data from each drop
    def _write_drop_data(self):
        table = '<table border="1" cellpadding="2" cellspacing="0">\n'
        table += '<tr><td><b>Drop</b></td><td><b>Count</b></td><td><b>Dropped Packets</b></td><td><b>Halted</b></td></tr>\n'
        for drop in self.drops:
            for i in range(0, drop.count):
                table += '<tr><td>%s</td><td>%d</td><td>%s</td><td>%s</td></tr>\n' % (drop.name, i+1, drop.packets[i], drop.halted[i])
        table += '</table>\n'
        
        return table
    
    ##\brief Write data from entire drop test in HTML
    def _write_result(self, msg=''):
        data = '<p align=center><b>%s Data</b></p>\n' % (self.test_name)
        data += '<p>Result: %s. Completed: %d of %d</p>\n' % (bool_to_msg(self.is_pass()), self.complete, self.total)
        if len(msg) > 0:
            data += '<p>' + msg + '</p>\n'

        data += self._write_drop_data()

        return data
    
    ##\brief Write summary of drop test
    def _write_summary(self, msg=''):
        return '%s, result: %s. Completed %d/%d drops. %s' % (self.test_name, bool_to_msg(self.is_pass()), self.complete, self.total, msg)
                  
    ##\brief Load pre-drop instructions if any, or proceed
    def start_test(self):
        if self.predrop is not None:
            self._html_window.LoadFile(self.predrop)
        else:
            self._state += 1 # Start dropping
            self.display_drop_structs()
                                                          
    ##\brief Load post-drop instructions, or pass drop test
    def post_drop_check(self):
        if self.postdrop is not None:
            self._html_window.LoadFile(self.postdrop)
        else: # PASS
            r = TestResultRequest.RESULT_PASS
            r.plots = []
            r.html_result = self._write_result()
            r.text_summary = self._write_summary()
            self.send_results(r)
                      
    ##\brief Verify that drop was OK, move to next drop
    def check_drop(self):
        self.complete += 1
        self.drops[self._current_drop].update(self._drop_packets == 0, self._eth_master_ok)

        if not self.is_ok():
            r = TestResultRequest()
            r.result = TestResultRequest.RESULT_FAIL
            r.plots = []
            r.html_result = self._write_result()
            r.text_summary = self._write_summary()

            wx.MessageBox('Drop test failed. Press OK to finish and analyze.', 'Drop test failed', wx.OK|wx.ICON_ERROR, self)
            self.send_results(r)
        
        self._current_drop_count += 1   
        if self._current_drop_count >= self.drops[self._current_drop].count:
            self._current_drop += 1
            self._current_drop_count = 0

        if self._current_drop >= len(self.drops): # PASS
            self._state += 1 # Post test
            self.post_drop_check()
            return

        self.display_drop_structs()
    
    ##\brief Display instructions for drop in window. 
    def display_drop_structs(self):
        drop = self.drops[self._current_drop]

        structs = '<html><header>\n'
        structs += '<H2 align=center>%s</H2>\n' % self.test_name
        structs += '</header><hr size="3">\n<body>\n'
        structs += '<H3>%s. Count: %d of %d</H3>\n' % (drop.name, self._current_drop_count + 1, drop.count)
        structs += '<H4>Drop number %d of %d</H4>\n' % (self._current_drop + 1, len(self.drops))
        structs += '<hr size=2>\n'

        structs += open(drop.file).read()
        structs += '</body></html>'
                                         
        self._html_window.SetPage(structs)

    
##\brief App for displaying drop test GUI
class DropTestApp(wx.App):
    def __init__(self, name, predrop, postdrop, drops):
        self.pre = predrop
        self.post = postdrop
        self.drops = drops
        self.name = name
        wx.App.__init__(self, clearSigInt = False)


    def OnInit(self):
        self._frame = DropTestFrame(None, self.name, self.pre, self.post, self.drops)
        self._frame.SetSize(wx.Size(550, 750))
        self._frame.Layout()
        self._frame.Show(True)
        
        return True
        


if __name__ == '__main__':
    parser = OptionParser(usage='./%prog --pre PREDROP --post POSTDROP NAME1,DROP1,# NAME2,DROP2,# ...', prog="drop_test.py")
    parser.add_option("--pre", action="store", dest="predrop", metavar="PREDROP",
                      help="Predrop instructions file", default=None)
    parser.add_option("--post", action="store", dest="postdrop", metavar="POSTDROP",
                      help="Postdrop instructions file and checklist", default=None)
    parser.add_option("--name", action="store", dest="name", metavar="TESTNAME",
                      help="Name of drop test (ex: Gripper Drop Test)", default='Drop Test')

    options, args = parser.parse_args()

    

    drops = []
    for arg in args:
        if arg.find(',') < 0:
            continue

        name, file, count = arg.split(',')
        count = int(count)
        drops.append(Drop(name, file, count))

    if len(drops) == 0:
        parser.error('No drop instructions given. Unable to drop component')
        
        
    
    app = DropTestApp(options.name, options.predrop, options.postdrop, drops)
    try:
        app.MainLoop()
    except KeyboardInterrupt:
        raise
    except:
        import traceback
        traceback.print_exc()
        r = TestResultRequest()
        r.text_summary = 'Caught exception'
        r.html_result = '<p>Caught exception, automated test failure. %s</p>\n' % traceback.format_exc()
        r.plots = []
        r.result = TestResultRequest.RESULT_FAIL
        rospy.wait_for_service('test_result', 10)
        self.result_service.call(test_result)
        self.data_sent = True
                                                              
