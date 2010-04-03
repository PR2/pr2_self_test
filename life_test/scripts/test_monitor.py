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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

##\author Kevin Watts
##\brief Loads listeners, monitors status of PR2 hardware tests

PKG = 'life_test'

import roslib
roslib.load_manifest(PKG)

import rospy

from life_test.msg import TestStatus

from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import *
import std_msgs.msg

import traceback
import sys

TIMEOUT = 60 # If no heartbeat, shut down
IGNORE_TIME = 30 # Ignore status for first few seconds

def create_listener(params, listeners):
    if not (params.has_key('type') and params.has_key('file')):
        rospy.logerr('Params "type" and "file" weren\'t found!')
        return False
    
    file = params['file']
    type = params['type']

    try:    
        import_str = '%s.%s' % (PKG, file)
        __import__(import_str)
        pypkg = sys.modules[import_str]
        listener_type = getattr(pypkg, type)
    except:
        rospy.logerr('Couldn\'t load listener %s from %s.\nException: %s' % (type, file, traceback.format_exc()))
        return False
    
    try:
        listener = listener_type()
    except:
        rospy.logerr('Listener %s failed to construct.\nException: %s' % (type, traceback.format_exc()))
        return False
                     
    if not listener.create(params):
        return False

    listeners.append(listener)
    return True

##\todo Add slots here
class StatusData:
    __slots__ = ['level', 'messages', 'array']
    def __init__(self):
        self.level = TestStatus.RUNNING
        self.messages = []
        self.array = DiagnosticArray()

# Listeners need: reset, halt, init, check_ok
class TestMonitor:
    def __init__(self):
        rospy.init_node('test_monitor')

        self._listeners = []

        my_params = rospy.get_param("~")

        self._listeners_ok = True

        for ns, listener_param in my_params.iteritems():
            if not create_listener(listener_param, self._listeners):
                rospy.logerr('Listener failed to initialize. Namespace: %s' % ns)
                self._listeners_ok = False


        self._status_pub = rospy.Publisher('test_status', TestStatus)
        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)

        self.reset_srv = rospy.Service('reset_test', Empty, self.reset_test)
        self.halt_srv = rospy.Service('halt_test', Empty, self.halt_test)

        self._heartbeat_time = rospy.get_time()
        self._heartbeat_sub = rospy.Subscriber('/heartbeat', std_msgs.msg.Empty, self.on_heartbeat)
        self._heartbeat_halted = False

        self._was_ok = True
        self._start_time = rospy.get_time() # Ignore failures for a bit after start

    def on_heartbeat(self, msg):
        self._heartbeat_time = rospy.get_time()

    def reset_test(self, srv):
        self._heartbeat_halted = False
        self._was_ok = True
        self._start_time = rospy.get_time()

        for listener in self._listeners:
            try:
                listener.reset()
            except:
                rospy.logerr('Listener failed to reset!')
        return EmptyResponse()

    def halt_test(self, srv):
        self._halt_listeners()

        return EmptyResponse()

    def _halt_listeners(self):
        for listener in self._listeners:
            try:
                listener.halt()
            except:
                rospy.logerr('Listener failed to halt!')

    def _check_status(self):
        stat_data = StatusData()
        
        for listener in self._listeners:
            try:
                lvl, msg, diags = listener.check_ok()
            except:
                rospy.logerr('Listener failed to check status. %s' % traceback.format_exc())
                stat, msg, diags = (TestStatus.ERROR, 'Error', None)

            stat_data.level = max(stat_data.level, lvl)
            if msg is not None and msg != '':
                stat_data.messages.append(msg)
            if diags is not None:
                stat_data.array.status.extend(diags)
        
        if len(self._listeners) == 0:
            stat_data.level = TestStatus.STALE
            stat_data.messages = [ 'No listeners' ]
            return stat_data

        if not self._listeners_ok:
            stat_data.level = TestStatus.ERROR
            stat_data.messages = ['Listener Startup Error']

        if rospy.get_time() - self._start_time > IGNORE_TIME and (self._was_ok and stat_data.level > 1):
            self._halt_listeners()
            rospy.logerr('Halted test after failure. Failure message: %s' % ', '.join(stat_data.messages))
            self._was_ok = False
            

        if not self._heartbeat_halted and rospy.get_time() - self._heartbeat_time > TIMEOUT:
            rospy.logerr('No heartbeat from Test Manager received, halting test')
            self._halt_listeners()
            self._heartbeat_halted = True


        if self._heartbeat_halted:
            stat_data.status = TestStatus.STALE
            stat_data.messages = ['No heartbeat']
            

        return stat_data

    def publish_status(self):
        stat_data = self._check_status()

        if len(stat_data.array.status) > 0:
            stat_data.array.header.stamp = rospy.get_rostime()
            self._diag_pub.publish(stat_data.array)

        test_stat = TestStatus()
        test_stat.test_ok = int(stat_data.level)
        test_stat.message = ', '.join(stat_data.messages)
        if test_stat.test_ok == TestStatus.RUNNING:
            test_stat.message = 'OK'

        self._status_pub.publish(test_stat)

if __name__ == '__main__':
    try:
        tm = TestMonitor()
        
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            rate.sleep()
            tm.publish_status()
    except:
        traceback.print_exc()
        rospy.logerr(traceback.format_exc())
