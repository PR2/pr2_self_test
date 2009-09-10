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

# Author: Kevin Watts

PKG = 'life_test'

import roslib
roslib.load_manifest(PKG)

import rospy

from life_test.msg import TestStatus

from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import *

import traceback
import sys

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
        rospy.logerr('Couldn\'t load listener %s from %s. Exception: %s' % (type, file, traceback.format_exc()))
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

# Listeners need: reset, halt, init, check_ok
class TestMonitor:
    def __init__(self):
        rospy.init_node('test_monitor')

        self._listeners = []

        my_params = rospy.get_param("~")

        for ns, listener_param in my_params.iteritems():
            if not create_listener(listener_param, self._listeners):
                rospy.logfatal('Listener failed to initialize. Exiting.')
                sys.exit()

        self._status_pub = rospy.Publisher('test_status', TestStatus)
        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)

        self.reset_srv = rospy.Service('reset_test', Empty, self.reset_test)
        self.halt_srv = rospy.Service('halt_test', Empty, self.halt_test)

    def reset_test(self, srv):
        for listener in self._listeners:
            listener.reset()
        return EmptyResponse()

    def halt_test(self, srv):
        for listener in self._listeners:
            listener.halt()
        return EmptyResponse()

    def publish_status(self):
        status = 0
        messages = []

        array = DiagnosticArray()
        for listener in self._listeners:
            stat, msg, diags = listener.check_ok()
            status = max(status, stat)
            if msg != '':
                messages.append(msg)
            if diags:
                array.status.extend(diags)
        
        if len(self._listeners) == 0:
            status = 3
            messages = [ 'No listeners' ]

        if len(array.status) > 0:
            self._diag_pub.publish(array)

        test_stat = TestStatus()
        test_stat.test_ok = int(status)
        test_stat.message = ', '.join(messages)
        if test_stat.test_ok == 0:
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
