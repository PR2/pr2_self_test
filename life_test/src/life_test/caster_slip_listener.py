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

##\author Kevin Watts
##\brief Listens to joint_state, makes sure caster isn't slipping


PKG = 'life_test'
import roslib
roslib.load_manifest(PKG)

from pr2_mechanism_msgs.msg import MechanismState, JointState
from std_srvs.srv import *
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

import rospy

import threading

TURRET_NAME = 'fl_caster_rotation_joint'
L_WHEEL_NAME = 'fl_caster_l_wheel_joint'
R_WHEEL_NAME = 'fl_caster_r_wheel_joint'

# From URDF
WHEEL_RADIUS = 0.074792
WHEEL_OFFSET = 0.049

ALLOWED_SLIP = 0.04 # Probably too big for update rate
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
        return True, -1000000000, -1000000000

    # Calculate turret rotation
    turret = new.turret - old.turret
    
    # Calculate wheel travel from offset
    wheel_dx = turret * WHEEL_OFFSET

    # Distances wheels actually moved
    r_dx = (new.r_wheel - old.r_wheel) * WHEEL_RADIUS
    l_dx = (new.l_wheel - old.l_wheel) * WHEEL_RADIUS

    # Error
    r_err = abs(r_dx - wheel_dx)
    l_err = abs(l_dx - wheel_dx)

    if (r_err > ALLOWED_SLIP) or (l_err > ALLOWED_SLIP):
        return False, r_err, l_err
    return True, r_err, l_err

##\brief Need to put in transmission monitor
class CasterSlipListener:
    def __init__(self):
        self._diag_sub = rospy.Subscriber('mechanism_state', MechanismState, self._ms_callback)
        self._mutex = threading.Lock()

        self._ok = True
        self._update_time = 0
        self._halt_motors = rospy.ServiceProxy('halt_motors', Empty)
                
        self.last_position = CasterPosition(None)

        self._max_l_err = -1000000000
        self._max_r_err = -1000000000

        self._num_errors = 0
        self._num_errors_since_reset = 0
        
    
    def create(self, params):
        try:
            rospy.wait_for_service('halt_motors', 10)
        except:
            rospy.logerr('Wait for service \'halt_motors\' timed out in Caster Slip Listener.')
            return False
        return True

    def halt(self):
        pass

    def reset(self):
        self._mutex.acquire()
        self._ok = True
        self._num_errors_since_reset = 0
        self._mutex.release()

    def _ms_callback(self, msg):
        self._mutex.acquire()

        if rospy.get_time() - self._update_time < UPDATE_INTERVAL:
            self._mutex.release()
            return 

        was_ok = self._ok

        self._update_time = rospy.get_time()

        position = CasterPosition(msg.joint_states)

        ok, r_err, l_err = check_position(position, self.last_position)
        if not ok:
            self._ok = False
            self._num_errors += 1
            self._num_errors_since_reset += 1
        
        self._max_l_err = max(self._max_l_err, l_err)
        self._max_r_err = max(self._max_r_err, r_err)

        self.last_position = position

        self._mutex.release()

        # Halt if broken
        if not self._ok and was_ok:
            rospy.logerr('Halting motors, broken transmission.')
            try:
                self._halt_motors()
            except:
                import traceback
                rospy.logerr('Caught exception trying to halt motors: %s', traceback.format_exc())

 
    def check_ok(self):
        self._mutex.acquire()
        msg = ''
        stat = 0
        if not self._ok:
            stat = 2
            msg = 'Caster Slipping'

        if rospy.get_time() - self._update_time > 3:
            stat = 3
            msg = 'Caster Data Stale'
            if self._update_time == 0:
                msg = 'No Caster Data'
                
        diag = DiagnosticStatus()
        diag.level = stat
        diag.name = 'Caster Slip Listener'
        diag.message = msg
        if msg == '':
            diag.message = 'OK'

        diag.values.append(KeyValue("Turret", str(TURRET_NAME)))
        diag.values.append(KeyValue("R Wheel", str(R_WHEEL_NAME)))
        diag.values.append(KeyValue("L Wheel", str(L_WHEEL_NAME)))
        diag.values.append(KeyValue("Turret Position", str(self.last_position.turret)))
        diag.values.append(KeyValue("R Wheel Position", str(self.last_position.r_wheel)))
        diag.values.append(KeyValue("L Wheel Position", str(self.last_position.l_wheel)))
        diag.values.append(KeyValue("Max Obs. Right Slip", str(self._max_r_err)))
        diag.values.append(KeyValue("Max Obs. Left Slip", str(self._max_l_err)))
        diag.values.append(KeyValue("Wheel Offset", str( WHEEL_OFFSET)))
        diag.values.append(KeyValue("Wheel Diameter", str(WHEEL_RADIUS)))
        diag.values.append(KeyValue("Allowed Slip", str(ALLOWED_SLIP)))
        diag.values.append(KeyValue("Update Interval", str(UPDATE_INTERVAL)))
        diag.values.append(KeyValue("Total Errors", str(self._num_errors)))
        diag.values.append(KeyValue("Errors Since Reset", str(self._num_errors_since_reset)))

        self._mutex.release()
        return stat, msg, [ diag ]
