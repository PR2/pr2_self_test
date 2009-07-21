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

import roslib
roslib.load_manifest('life_test')

from mechanism_msgs.msg import *
from diagnostic_msgs.msg import *

import math

import rospy


class TransmissionMonitor():
    def __init__(self, actuator_name, joint_name, ref_position, deadband, continuous, positive):
        # Should probably strip these or something for no spaces
        self._actuator     = actuator_name
        self._joint        = joint_name
        self._ref_position = ref_position
        self._positive     = positive
        self._continuous   = continuous
        self._deadband     = deadband
        
        self._ok = True

        self._num_errors = 0
        self._num_errors_since_reset = 0
        self._rx_count = 0
        self._max_position = -1000000
        self._min_position = 10000000

    def reset(self):
        self._ok = True
        self._num_errors_since_reset = 0

    def check_device(self, position, cal_reading, calibrated):
        if abs(position - self._ref_position) < self._deadband:
            return True # Don't know b/c in deadband
        if not calibrated:
            return True # Don't bother with uncalibrated joints

        if self._continuous: # Reset position to home value
            position = position % (2*math.pi) - math.pi # Between -pi, pi

        # Reverse for joints that have negative search velocities
        cal_bool = cal_reading % 2 == 0
        if not self._positive:
            cal_bool = not cal_bool
            
        if (position > self._ref_position and not cal_bool):
            return True
        if (position < self._ref_position and cal_bool):
            return True
        else:
            return False

    def update(self, mech_state):
        self._rx_count += 1
        diag = DiagnosticStatus()
        diag.level = 0  # Default the level to 'OK'
        diag.name = "Trans. Monitor: %s" % self._joint
        diag.message = "OK"
        diag.values = [ ]
        diag.strings = [ ]
 
        diag.strings.append(DiagnosticString(value=self._joint, label="Joint"))
        diag.strings.append(DiagnosticString(value=self._actuator, label="Actuator"))
        diag.strings.append(DiagnosticString(value=str(self._positive), label="Positive Joint"))
        diag.strings.append(DiagnosticString(value=str(self._continuous), label="Continuous Joint"))
        diag.values.append(DiagnosticValue(value=float(self._ref_position), label="Reference Position"))
        diag.values.append(DiagnosticValue(value=float(self._deadband), label="Deadband"))
        diag.values.append(DiagnosticValue(value=float(self._rx_count), label="Mech State RX Count"))


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
            

        diag.strings.append(DiagnosticString(value=str(act_exists), label='Actuator Exists'))
        diag.strings.append(DiagnosticString(value=str(joint_exists), label='Joint Exists'))



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
        
        diag.strings.insert(0, DiagnosticString(value=diag.message, label='Transmission Status'))
        diag.strings.insert(1, DiagnosticString(value=reading_msg, label='Current Reading'))
        diag.strings.append(DiagnosticString(value=str(calibrated), label='Is Calibrated'))
        diag.values.append(DiagnosticValue(value=cal_reading, label='Calibration Reading'))
        diag.values.append(DiagnosticValue(value=position, label='Joint Position'))
        
        diag.values.append(DiagnosticValue(value=self._num_errors, label='Total Errors'))
        diag.values.append(DiagnosticValue(value=self._num_errors_since_reset, label='Errors Since Reset'))

        self._max_position = max(self._max_position, position)
        diag.values.append(DiagnosticValue(value = self._max_position, label='Max Obs. Position'))

        self._min_position = min(self._min_position, position)
        diag.values.append(DiagnosticValue(value = self._min_position, label='Min Obs. Position'))
        
        return diag, self._ok

