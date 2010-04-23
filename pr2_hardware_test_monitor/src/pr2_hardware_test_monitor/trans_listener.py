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

##\brief Listens to transmissions of specified joints, halts motors if error detected.

from __future__ import with_statement
PKG = 'pr2_hardware_test_monitor'

import roslib
roslib.load_manifest(PKG)

from pr2_mechanism_msgs.msg import MechanismStatistics
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import *

import rospy

import threading

GRACE_HITS = 5 # Max number of errors before halt motors called

TURRET_NAME = 'fl_caster_rotation_joint'
L_WHEEL_NAME = 'fl_caster_l_wheel_joint'
R_WHEEL_NAME = 'fl_caster_r_wheel_joint'

# From URDF
WHEEL_RADIUS = 0.074792
WHEEL_OFFSET = 0.049

ALLOWED_SLIP = 0.015 # (1.5cm/interval)
UPDATE_INTERVAL = 0.25 

class CasterPosition:
    def __init__(self, msg):
        self.turret = None
        self.l_wheel = None
        self.r_wheel = None
        self.turret_cal = False

        if msg is None:
            return

        for js in msg:
            if js.name == TURRET_NAME:
                self.turret_cal = js.is_calibrated
                self.turret = js.position
            if js.name == L_WHEEL_NAME:
                self.l_wheel = js.position
            if js.name == R_WHEEL_NAME:
                self.r_wheel = js.position
    
    def has_data(self):
        return self.turret_cal and self.turret is not None and self.l_wheel is not None and self.r_wheel is not None

def check_position(new, old):
    if not new.has_data() or not old.has_data():
        return True, None, None

    # Calculate turret rotation
    turret = new.turret - old.turret
    
    # Calculate wheel travel from offset
    wheel_dx = turret * WHEEL_OFFSET

    # Distances wheels actually moved
    r_dx = (new.r_wheel - old.r_wheel) * WHEEL_RADIUS
    l_dx = -1 * (new.l_wheel - old.l_wheel) * WHEEL_RADIUS

    # Error
    r_err = r_dx - wheel_dx
    l_err = l_dx - wheel_dx

    if (abs(r_err) > ALLOWED_SLIP) or (abs(l_err) > ALLOWED_SLIP):
        return False, r_err, l_err
    return True, r_err, l_err

##\brief Makes sure caster doesn't slip or drive forward
class CasterSlipListener:
    def __init__(self):
        self._ok = True
        self._update_time = 0
                
        self.last_position = CasterPosition(None)

        self._max_l_err_pos = None
        self._max_r_err_pos = None

        self._max_l_err_neg = None
        self._max_r_err_neg = None

        self._max_l_err_pos_reset = None
        self._max_r_err_pos_reset = None

        self._max_l_err_neg_reset = None
        self._max_r_err_neg_reset = None

        self._num_errors = 0
        self._num_errors_since_reset = 0

        self.diag = DiagnosticStatus()
        self.stat = 0
        self.msg = ''
            
    def create(self, params):
        return True

    def reset(self):
        self._ok = True
        self._num_errors_since_reset = 0

        self._max_l_err_pos_reset = None
        self._max_r_err_pos_reset = None

        self._max_l_err_neg_reset = None
        self._max_r_err_neg_reset = None

    def update(self, msg):
        if rospy.get_time() - self._update_time < UPDATE_INTERVAL:
            return self._ok

        self._update_time = rospy.get_time()

        position = CasterPosition(msg.joint_statistics)

        ok, r_err, l_err = check_position(position, self.last_position)
        if not ok:
            self._ok = False
            self._num_errors += 1
            self._num_errors_since_reset += 1
        
        if r_err is None:
            pass
        elif r_err > 0:
            self._max_r_err_pos = max(self._max_r_err_pos, abs(r_err))
            self._max_r_err_pos_reset = max(self._max_r_err_pos_reset, abs(r_err))
        else:
            self._max_r_err_neg = max(self._max_r_err_neg, abs(r_err))
            self._max_r_err_neg_reset = max(self._max_r_err_neg_reset, abs(r_err))

        if l_err is None:
            pass
        elif l_err > 0:
            self._max_l_err_pos = max(self._max_l_err_pos, abs(l_err))
            self._max_l_err_pos_reset = max(self._max_l_err_pos_reset, abs(l_err))
        else:
            self._max_l_err_neg = max(self._max_l_err_neg, abs(l_err))
            self._max_l_err_neg_reset = max(self._max_l_err_neg_reset, abs(l_err))

        self.last_position = position

        return self._ok

    def get_status(self):
        stat = 0
        if not self._ok:
            stat = 2

        if rospy.get_time() - self._update_time > 3:
            stat = 3
                
        diag = DiagnosticStatus()
        diag.level = stat
        diag.name = 'Caster Slip Listener'
        diag.message = 'OK'
        if diag.level == 2:
            diag.message = 'Caster Slipping'
        if diag.level == 3:
            diag.message = 'Caster Stale'
            diag.level = 2

        diag.values.append(KeyValue("Turret", str(TURRET_NAME)))
        diag.values.append(KeyValue("R Wheel", str(R_WHEEL_NAME)))
        diag.values.append(KeyValue("L Wheel", str(L_WHEEL_NAME)))
        diag.values.append(KeyValue("Turret Position", str(self.last_position.turret)))
        diag.values.append(KeyValue("R Wheel Position", str(self.last_position.r_wheel)))
        diag.values.append(KeyValue("L Wheel Position", str(self.last_position.l_wheel)))
        diag.values.append(KeyValue("Max Pos. Left Slip", str(self._max_l_err_pos)))
        diag.values.append(KeyValue("Max Neg. Left Slip", str(self._max_l_err_neg)))
        diag.values.append(KeyValue("Max Pos. Right Slip", str(self._max_r_err_pos)))
        diag.values.append(KeyValue("Max Neg. Right Slip", str(self._max_r_err_neg)))
        diag.values.append(KeyValue("Max Pos. Left Slip (Reset)", str(self._max_l_err_pos_reset)))
        diag.values.append(KeyValue("Max Neg. Left Slip (Reset)", str(self._max_l_err_neg_reset)))
        diag.values.append(KeyValue("Max Pos. Right Slip (Reset)", str(self._max_r_err_pos_reset)))
        diag.values.append(KeyValue("Max Neg. Right Slip (Reset)", str(self._max_r_err_neg_reset)))
        diag.values.append(KeyValue("Wheel Offset", str(WHEEL_OFFSET)))
        diag.values.append(KeyValue("Wheel Diameter", str(WHEEL_RADIUS)))
        diag.values.append(KeyValue("Allowed Slip", str(ALLOWED_SLIP)))
        diag.values.append(KeyValue("Update Interval", str(UPDATE_INTERVAL)))
        diag.values.append(KeyValue("Total Errors", str(self._num_errors)))
        diag.values.append(KeyValue("Errors Since Reset", str(self._num_errors_since_reset)))

        return diag


##\brief Monitors transmission of single PR2 joint
class JointTransmissionListener():
    def __init__(self):
        self._ok = True
        self._num_errors = 0
        self._num_hits = 0
        self._num_errors_since_reset = 0
        self._rx_count = 0
        self._max_position = -10000000
        self._min_position = 100000000

        self._broke_count = 0

        self.act_exists = False
        self.joint_exists = False

        self.calibrated = False
        self.cal_reading = None
        self.position = None
        self.level = 0
        self.message = 'OK'
        self.reading_msg = 'No data'

        self._last_rising = 0
        self._last_falling = 0
        self._last_bad_reading = 0


    ## Mandatory params: actuator, joint, deadband
    def create(self, params):
        if not params.has_key('actuator'):
            rospy.logerr('Parameter "actuator" not found! Aborting.')
            return False
        self._actuator = params['actuator']

        if not params.has_key('joint'):
            rospy.logerr('Parameter "joint" not found! Aborting.')
            return False
        self._joint = params['joint']

        if not params.has_key('deadband') and self._joint.find("gripper") < 0:
            rospy.logerr('Parameter "deadband" not found! Aborting.')
            return False
        if params.has_key('deadband'):
            self._deadband = params['deadband']
        else:
            self._deadband = None

        ## Calibration flag references
        if not params.has_key('up_ref'):
            self._up_ref = None
        else:
            self._up_ref = params['up_ref']
        
        if not params.has_key('down_ref'):
            self._down_ref = None
        else:
            self._down_ref = params['down_ref']

        # Must have either up/down, except grippers
        if self._up_ref is None and self._down_ref is None and self._joint.find("gripper") < 0:
            rospy.logerr('Neither up or down reference was given! Aborting.')
            return False

        # For continuous joints
        if not params.has_key('wrap'):
            self._wrap = None
        else:
            self._wrap = params['wrap']
        
        # Max limit
        if not params.has_key('max'):
            self._max = None
        else:
            self._max = params['max']

        # Min limit
        if not params.has_key('min'):
            self._min = None
        else:
            self._min = params['min']

        return True

    def reset(self):
        self._ok = True
        self._num_errors_since_reset = 0

    def check_device(self, position, cal_reading, calibrated):
        if not calibrated:
            return True # Don't bother with uncalibrated joints

        if self._wrap is not None and self._wrap > 0:
            position = position % (self._wrap)

        if self._up_ref is not None:
            if abs(position - self._up_ref) < self._deadband:
                return True # Don't know b/c in deadband
            if self._wrap is not None:
                if abs((self._wrap - position) - self._up_ref) < self._deadband:
                    return True # This checks that we're not really close to the flag on the other side of the wrap
        if self._down_ref is not None:
            if abs(position - self._down_ref) < self._deadband:
                return True
            if self._wrap is not None:
                if abs((self._wrap - position) - self._down_ref) < self._deadband:
                    return True 

        # Check limits
        if self._max is not None and position > self._max:
            return False
        if self._min is not None and position < self._min:
            return False

        # Grippers only have max/min. No other stuff.
        if self._joint.find("gripper") > 0:
            return True

        # cal_bool is True if the flag is closed
        # cal_bool = True for "13"
        cal_bool = cal_reading % 2 == 1
        
        
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
                    rospy.logwarn('Broken transmission reading for %s. Position: %f (wrapped), cal_reading: %d. Up ref: %s, down ref: %s' % (self._joint, position, cal_reading, str(self._up_ref), str(self._down_ref)))
                    return False
            else: # Down > Up
                if position < self._up_ref and not cal_bool:
                    return True
                if (position > self._up_ref) and (position < self._down_ref) and cal_bool:
                    return True
                if (position > self._down_ref) and not cal_bool:
                    return True
                else:
                    rospy.logwarn('Broken transmission reading for %s. Position: %f (wrapped), cal_reading: %d. Up ref: %s, down ref: %s' % (self._joint, position, cal_reading, str(self._up_ref), str(self._down_ref)))
                    return False

        ## Has only up limit
        if self._up_ref is not None:
            if position > self._up_ref and cal_bool:
                return True
            if position < self._up_ref and not cal_bool:
                return True
            else:
                rospy.logwarn('Broken transmission reading for %s. Position: %f (wrapped), cal_reading: %d. Up ref: %s, down ref: %s' % (self._joint, position, cal_reading, str(self._up_ref), str(self._down_ref)))
                return False

        ## Has only down limit
        if self._down_ref is not None:
            if position > self._down_ref and not cal_bool:
                return True
            if position < self._down_ref and cal_bool:
                return True
            else:
                rospy.logwarn('Broken transmission reading for %s. Position: %f (wrapped), cal_reading: %d. Up ref: %s, down ref: %s' % (self._joint, position, cal_reading, str(self._up_ref), str(self._down_ref)))
                return False
        
        rospy.logwarn('Broken transmission reading for %s. (No case handled.) Position: %f (wrapped), cal_reading: %d. Up ref: %s, down ref: %s' % (self._joint, position, cal_reading, str(self._up_ref), str(self._down_ref)))
        return False


    def update(self, mech_state):
        self._rx_count += 1

        # Check if we can find both the joint and actuator
        act_names = [x.name for x in mech_state.actuator_statistics]
        self.act_exists = self._actuator in act_names ;
        
        if self.act_exists:
            self.cal_reading = mech_state.actuator_statistics[act_names.index(self._actuator)].calibration_reading
            self._last_rising = mech_state.actuator_statistics[act_names.index(self._actuator)].last_calibration_rising_edge
            self._last_falling = mech_state.actuator_statistics[act_names.index(self._actuator)].last_calibration_falling_edge

        joint_names = [x.name for x in mech_state.joint_statistics]
        self.joint_exists = self._joint in joint_names
        if self.joint_exists :
            self.position = mech_state.joint_statistics[joint_names.index(self._joint)].position
            self.calibrated = (mech_state.joint_statistics[joint_names.index(self._joint)].is_calibrated == 1)
            
        # First check existance of joint, actuator
        if not (self.act_exists and self.joint_exists):
            self.level = 2
            self.message = 'Actuators, Joints missing'
            self._ok = False
            return self._ok
        
        # Monitor current state
        # Give it a certain number of false positives before reporting error
        self.reading_msg = 'OK'
        if not self.check_device(self.position, self.cal_reading, self.calibrated):
            self._broke_count += 1
            self._num_hits += 1
            self._last_bad_reading = self.position
        else:
            self._broke_count = 0

        if self._broke_count > GRACE_HITS:
            self._ok = False
            self._num_errors += 1
            self._num_errors_since_reset += 1
            self.reading_msg = 'Broken'
        
        # If we've had an error since the last reset, we're no good
        if not self._ok:
            self.message = 'Broken'
            self.level = 2
        else:
            self.message = 'OK'
            self.level = 0
        
        if self.calibrated:
            self._max_position = max(self._max_position, self.position)
            self._min_position = min(self._min_position, self.position)
        
        return self._ok

    def get_status(self):
        diag = DiagnosticStatus()
        diag.level = self.level
        diag.name = "Trans. Listener: %s" % self._joint
        diag.message = self.message
        diag.values = []

        diag.values.append(KeyValue('Transmission Status', self.message))
        diag.values.append(KeyValue('Current Reading', self.reading_msg)) 
        diag.values.append(KeyValue("Joint", self._joint))
        diag.values.append(KeyValue("Actuator", self._actuator))
        diag.values.append(KeyValue("Up position", str(self._up_ref)))
        diag.values.append(KeyValue("Down position", str(self._down_ref)))
        diag.values.append(KeyValue("Wrap", str(self._wrap)))
        diag.values.append(KeyValue("Max Limit", str(self._max)))
        diag.values.append(KeyValue("Min Limit", str(self._min)))
        diag.values.append(KeyValue("Deadband", str(self._deadband)))

        diag.values.append(KeyValue('Actuator Exists', str(self.act_exists)))
        diag.values.append(KeyValue('Joint Exists', str(self.joint_exists)))

        diag.values.append(KeyValue("Mech State RX Count", str(self._rx_count)))

        diag.values.append(KeyValue('Is Calibrated', str(self.calibrated)))
        diag.values.append(KeyValue('Calibration Reading', str(self.cal_reading)))
        diag.values.append(KeyValue('Joint Position', str(self.position)))
        
        diag.values.append(KeyValue('Total Errors', str(self._num_errors)))
        diag.values.append(KeyValue('Errors Since Reset', str(self._num_errors_since_reset)))
        diag.values.append(KeyValue('Total Bad Readings', str(self._num_hits)))
        diag.values.append(KeyValue('Max Obs. Position', str(self._max_position)))
        diag.values.append(KeyValue('Min Obs. Position', str(self._min_position)))

        diag.values.append(KeyValue('Last Rising Edge', str(self._last_rising)))
        diag.values.append(KeyValue('Last Falling Edge', str(self._last_falling)))
        diag.values.append(KeyValue('Last Bad Reading', str(self._last_bad_reading)))

        return diag

        

##\brief Loads individual joint listeners, monitors all robot transmissions
class TransmissionListener:
    def __init__(self):
        self._joint_monitors = []
        self._mech_sub = rospy.Subscriber('mechanism_statistics', MechanismStatistics, self._callback)
        self._halt_motors = rospy.ServiceProxy('pr2_etherCAT/halt_motors', Empty)

        self._mutex = threading.Lock()
        self._ok = True
        self._last_msg_time = 0
        
    def create(self, params):
        for joint, joint_param in params.iteritems():
            # Ignore setup params
            if joint == 'type' or joint == 'file':
                continue

            if joint == 'caster_slip':
                joint_mon = CasterSlipListener()
            else:
                joint_mon = JointTransmissionListener()
            if not joint_mon.create(joint_param):
                rospy.logerr('Unable to create JointTransmissionListener')
                return False
            self._joint_monitors.append(joint_mon)
        return True
        
    def _callback(self, msg):
        with self._mutex:
            self._last_msg_time = rospy.get_time()
            self._diag_stats = []
            
            was_ok = self._ok
            
            for joint_mon in self._joint_monitors:
                ok = joint_mon.update(msg)
                self._ok = ok and self._ok
                

        # Halt if broken
        if not self._ok and was_ok:
            rospy.logerr('Halting motors, broken transmission.')
            try:
                self._halt_motors()
            except:
                import traceback
                rospy.logerr('Caught exception trying to halt motors: %s', traceback.format_exc())

    def reset(self):
        with self._mutex:
            self._ok = True
            for joint_mon in self._joint_monitors:
                joint_mon.reset()
    
    def halt(self):
        pass
            
    def check_ok(self):
        with self._mutex:
            if self._last_msg_time == 0:
                return 3, "No mech state", None
            if rospy.get_time() - self._last_msg_time > 3:
                return 3, "Mech state stale", None

            if self._ok:
                status = 0
                msg = ''
            else:
                status = 2
                msg = 'Transmission Broken'
        
            diag_stats = []
            for joint_mon in self._joint_monitors:
                diag_stats.append(joint_mon.get_status())
                
                
        return status, msg, diag_stats
