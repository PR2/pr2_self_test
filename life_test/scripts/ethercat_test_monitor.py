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

import roslib
roslib.load_manifest('life_test')

from mechanism_msgs.msg import MechanismState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from std_srvs.srv import *

from life_test import *
from life_test.msg import TestStatus
from life_test.trans_monitor import TransmissionMonitor

import rospy

import sys, os

import threading
import traceback
import csv


class EtherCATTestMonitorNode():
    def __init__(self):
        rospy.init_node('test_monitor', anonymous = True)

        self._trans_monitors = []
        if len(rospy.myargv()) > 1:
            csv_filename = rospy.myargv()[1] # CSV of device data
            self.create_trans_monitors(csv_filename)

        self._mutex = threading.Lock()

        self._ethercat_ok          = True
        self._transmissions_ok     = True
        self._ethercat_update_time = 0
        self._mech_update_time     = 0
        self._diags                = []
        self._trans_status         = [] 

        self.test_status_pub = rospy.Publisher('test_status', TestStatus)
        self.diag_sub = rospy.Subscriber('/diagnostics', DiagnosticArray, self.diag_callback)
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)

        self.reset_motors = rospy.ServiceProxy('reset_motors', Empty)
        self.halt_motors = rospy.ServiceProxy('halt_motors', Empty)
    
        self.reset_srv = rospy.Service('reset_test', Empty, self.reset_test)
        self.halt_srv = rospy.Service('halt_test', Empty, self.halt_test)

        self.mech_state_sub = rospy.Subscriber('mechanism_state', MechanismState, self.mech_state_callback)
        
        # Publish status at 1Hz
        self._timer = threading.Timer(1.0, self.publish_status)
        self._timer.start()
        
        rospy.spin()

    def __del__(self):
        self.test_status_pub.unregister()
        self.diag_sub.unregister()
        self.mech_state_sub.unregister()

        self.reset_srv.unregister()
        self.halt_srv.unregister()
        

    def create_trans_monitors(self, csv_filename):

        trans_csv = csv.reader(open(csv_filename, 'rb'))
        for row in trans_csv:
            actuator = row[0].lstrip().rstrip()
            joint = row[1].lstrip().rstrip()
            ref = float(row[2])
            deadband = float(row[3])
            continuous = (int(row[4]) == 1)
            positive = (int(row[5]) == 1)
            self._trans_monitors.append(TransmissionMonitor(
                    actuator, joint, ref, 
                    deadband, continuous, positive))

    def publish_status(self):
        self._mutex.acquire()

        self.check_diags()

        status = TestStatus()
      
        if rospy.get_time() - self._ethercat_update_time > 3:
            status.test_ok = TestStatus.TEST_STALE
            status.message = 'EtherCAT Master Stale'
        elif rospy.get_time() - self._mech_update_time > 1:
            status.message = 'Mechanism state stale'
            status.test_ok = TestStatus.TEST_STALE
        elif self._ethercat_ok and self._transmissions_ok:
            status.test_ok = TestStatus.TEST_RUNNING
            status.message = 'OK'
        else:
            status.test_ok = TestStatus.TEST_ERROR
            if not self._transmissions_ok:
                status.message = 'Transmission Broken'
            else:
                status.message = 'EtherCAT Halted'

        self.test_status_pub.publish(status)

        array = DiagnosticArray()
        array.status = self._trans_status
        self.diag_pub.publish(array)

        # DISABLE FOR TESTING
        # Halt the test if we have a bad transmission
        #if self._ethercat_ok and not self._transmissions_ok:
        #    self.halt_motors()

        if not rospy.is_shutdown():
            timer = threading.Timer(0.5, self.publish_status)
            timer.start()

        self._mutex.release()


    def reset_test(self, srv):
        self._mutex.acquire() # Need this here?

        for monitor in self._trans_monitors:
            monitor.reset()
        self.reset_motors()

        self._mutex.release()
        return EmptyResponse()

    def halt_test(self, srv = None):
        self.halt_motors()
        return EmptyResponse()
        
    def mech_state_callback(self, mech_state):
        self._mutex.acquire()

        self._mech_update_time = rospy.get_time()

        self._transmissions_ok = True
        self._trans_status = [] 
        for monitor in self._trans_monitors:
            diag, state = monitor.update(mech_state)
            self._trans_status.append(diag)
            self._transmissions_ok = self._transmissions_ok and state

        self._mutex.release()
            
    def diag_callback(self, msg):
        self._mutex.acquire()
        self._diags.append(msg)
        self._mutex.release()

    def check_diags(self):
        # Checks EtherCAT Master diagnostics to see if test running
        etherCAT_master_name = 'EtherCAT Master'
        try:
            for msg in self._diags:
                for stat in msg.status:
                    if stat.name == 'EtherCAT Master':
                        self._ethercat_ok = (stat.level == 0)
                        self._ethercat_update_time = rospy.get_time()

            self._diags = []

        except Exception, e:
            rospy.logerr('Caught exception processing diagnostic msg.\nEx: %s' % traceback.format_exc())

        

if __name__ == '__main__':
    try:
        my_node = EtherCATTestMonitorNode()
    except:
        print 'Caught exception in test monitor node'
        traceback.print_exc()
        sys.exit(2)
