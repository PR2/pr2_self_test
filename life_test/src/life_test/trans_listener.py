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
# Shamelessly copied from joint_calibration_monitor/generic_joint_monitor by Vijay Pradeep

PKG = 'life_test'

import roslib
roslib.load_manifest(PKG)

from pr2_mechanism_msgs.msg import MechanismState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import *

import math

import rospy

import threading

class JointTransmissionListener():
    def __init__(self):
        self._ok = True
        self._num_errors = 0
        self._num_errors_since_reset = 0
        self._rx_count = 0
        self._max_position = -10000000
        self._min_position = 100000000
        
    ## Mandatory params: actuator, joint, deadband
    def create(self, params):
        if not params.has_key('deadband'):
            rospy.logerr('Parameter "deadband" not found! Aborting.')
            return False
        self._deadband = params['deadband']

        if not params.has_key('actuator'):
            rospy.logerr('Parameter "actuator" not found! Aborting.')
            return False
        self._actuator = params['actuator']

        if not params.has_key('joint'):
            rospy.logerr('Parameter "joint" not found! Aborting.')
            return False
        self._joint = params['joint']

        ## Calibration flag references
        if not params.has_key('up_ref'):
            self._up_ref = None
        else:
            self._up_ref = params['up_ref']
        
        if not params.has_key('down_ref'):
            self._down_ref = None
        else:
            self._down_ref = params['down_ref']

        if self._up_ref is None and self._down_ref is None:
            rospy.logerr('Neither up or down reference was given! Aborting.')
            return False

        # For continuous joints
        if not params.has_key('wrap'):
            self._wrap = None
        else:
            self._wrap = params['wrap']
        
        if not params.has_key('max'):
            self._max = None
        else:
            self._max = params['max']

        if not params.has_key('min'):
            self._min = None
        else:
            self._min = params['min']

        return True

    def reset(self):
        self._ok = True
        self._num_errors_since_reset = 0

    def check_device(self, position, cal_reading, calibrated):
        if self._up_ref is not None:
            if abs(position - self._up_ref) < self._deadband:
                return True # Don't know b/c in deadband
        if self._down_ref is not None:
            if abs(position - self._down_ref) < self._deadband:
                return True # Don't know b/c in deadband

        if not calibrated:
            return True # Don't bother with uncalibrated joints

        if self._wrap is not None and self._wrap > 0:
            position = position % (self._wrap)

        # Check limits
        if self._max is not None and position > self._max:
            return False
        if self._min is not None and position < self._min:
            return False

        # Reverse for joints that have negative search velocities
        cal_bool = cal_reading % 2 == 0
        
        ## Has both up and down limit
        if (self._up_ref is not None) and (self._down_ref is not None):
            if self._up_ref > self._down_ref:
                if position < self._down_ref and cal_bool:
                    return True
                if (position > self._down_ref) and (position < self._up_ref) and not cal_bool:
                    return True
                if position > self._up_ref and cal_bool:
                    return True
                else:
                    return False
            else: # Down > Up
                if position < self._up_ref and cal_bool:
                    return True
                if (position > self._up_ref) and (position < self._down_ref) and cal_bool:
                    return True
                if (position > self._down_ref) and not cal_bool:
                    return True
                else:
                    return False

        ## Has only up limit
        if self._up_ref is not None:
            if position > self._up_ref and cal_bool:
                return True
            if position < self._up_ref and not cal_bool:
                return True
            else:
                return False

        ## Has only down limit
        if self._down_ref is not None:
            if position > self._down_ref and not cal_bool:
                return True
            if position < self._down_ref and cal_bool:
                return True
            else:
                return False
        
        return False


    def update(self, mech_state):
        self._rx_count += 1
        diag = DiagnosticStatus()
        diag.level = 0  # Default the level to 'OK'
        diag.name = "Trans. Monitor: %s" % self._joint
        diag.message = "OK"
        diag.values = [ ]
 
        diag.values.append(KeyValue("Joint", self._joint))
        diag.values.append(KeyValue("Actuator", self._actuator))
        diag.values.append(KeyValue("Up position", str(self._up_ref)))
        diag.values.append(KeyValue("Down position", str(self._down_ref)))
        diag.values.append(KeyValue("Wrap", str(self._wrap)))
        diag.values.append(KeyValue("Max Limit", str(self._max)))
        diag.values.append(KeyValue("Min Limit", str(self._min)))

        diag.values.append(KeyValue("Deadband", str(self._deadband)))
        diag.values.append(KeyValue("Mech State RX Count", str(self._rx_count)))


        # Check if we can find both the joint and actuator
        act_names = [x.name for x in mech_state.actuator_states]
        act_exists = self._actuator in act_names ;
        act_exists_msg = 'Error'

        if act_exists :
            cal_reading = mech_state.actuator_states[act_names.index(self._actuator)].calibration_reading

        joint_names = [x.name for x in mech_state.joint_states]
        joint_exists = self._joint in joint_names
        jnt_exists_msg = 'Error'
        if joint_exists :
            position = mech_state.joint_states[joint_names.index(self._joint)].position
            calibrated = (mech_state.joint_states[joint_names.index(self._joint)].is_calibrated == 1)
            

        diag.values.append(KeyValue('Actuator Exists', str(act_exists)))
        diag.values.append(KeyValue('Joint Exists', str(joint_exists)))

        # First check existance of joint, actuator
        if not (act_exists and joint_exists):
            diag.level = 2
            diag.message = 'Actuators, Joints missing'
            self._ok = False
            return diag, False
        
        # Monitor current state
        reading_msg = 'OK'
        if not self.check_device(position, cal_reading, calibrated):
            self._ok = False
            self._num_errors += 1
            self._num_errors_since_reset += 1
            reading_msg = 'Broken'
        
        # If we've had an error since the last reset, we're no good
        if not self._ok:
            diag.message = 'Broken'
            diag.level = 2
        
        diag.values.insert(0, KeyValue('Transmission Status', diag.message))
        diag.values.insert(1, KeyValue('Current Reading', reading_msg))
        diag.values.append(KeyValue('Is Calibrated', str(calibrated)))
        diag.values.append(KeyValue('Calibration Reading',str(cal_reading)))
        diag.values.append(KeyValue('Joint Position', str(position)))
        
        diag.values.append(KeyValue('Total Errors', str(self._num_errors)))
        diag.values.append(KeyValue('Errors Since Reset', str(self._num_errors_since_reset)))

        self._max_position = max(self._max_position, position)
        diag.values.append(KeyValue('Max Obs. Position', str(self._max_position)))

        self._min_position = min(self._min_position, position)
        diag.values.append(KeyValue('Min Obs. Position', str(self._min_position)))
        
        return self._ok, diag

# Loads individual joint listeners
class TransmissionListener:
    def __init__(self):
        self._joint_monitors = []
        self._mech_sub = rospy.Subscriber('mechanism_state', MechanismState, self._callback)
        self._halt_motors = rospy.ServiceProxy('halt_motors', Empty)

        self._mutex = threading.Lock()
        self._diag_stats = []
        self._ok = True
        self._last_msg_time = 0
        
    def create(self, params):
        for joint, joint_param in params.iteritems():
            # Ignore setup params
            if joint == 'type' or joint == 'file':
                continue

            joint_mon = JointTransmissionListener()
            if not joint_mon.create(joint_param):
                return False
            self._joint_monitors.append(joint_mon)
        return True
        
    def _callback(self, msg):
        self._mutex.acquire()
        self._last_msg_time = rospy.get_time()
        self._diag_stats = []

        was_ok = self._ok

        for joint_mon in self._joint_monitors:
            ok, stat = joint_mon.update(msg)
            self._diag_stats.append(stat)
            self._ok = ok and self._ok

        self._mutex.release()

        # Halt if broken
        if not self._ok and was_ok:
            rospy.logerr('Halting motors, broken transmission')
            #self._halt_motors()
        

    def reset(self):
        self._mutex.acquire()
        self._ok = True
        for joint_mon in self._joint_monitors:
            joint_mon.reset()
        self._mutex.release()
    
    def halt(self):
        pass
            
    def check_ok(self):
        self._mutex.acquire()
        
        if self._last_msg_time == 0:
            self._mutex.release()
            return 3, "No mech stat messages", None
        if rospy.get_time() - self._last_msg_time > 3:
            self._mutex.release()
            return 3, "Mech state stale", None

        if self._ok:
            status = 0
            msg = ''
        else:
            status = 2
            msg = 'Transmission Broken'

        diag_stats = self._diag_stats
        self._mutex.release()

        return status, msg, diag_stats
