#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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
##\brief Prints output of PR2 counterbalance check

PKG = 'pr2_counterbalance_check'
import roslib
roslib.load_manifest(PKG)

import os, sys
import rospy

from pr2_self_test_msgs.srv import *
from std_msgs.msg import Bool
from joint_qualification_controllers.msg import CounterbalanceTestData

from pr2_counterbalance_check.counterbalance_analysis import *

from optparse import OptionParser

def dir(turns):
    return 'CW' if turns > 0 else 'CCW'

SPRING_TOL = 2.0
BAR_TOL = 0.8

def _check_valid(params):
    if params.timeout_hit:
        print("Counterbalance controller timeout hit. Increase the timeout and retry.", file=sys.stderr)
        return False
    
    return True

class CounterbalanceCheck(object):
    def __init__(self, model_file):
        self._motors_halted = True
        self._data = None

        self.motors_topic = rospy.Subscriber('pr2_ethercat/motors_halted', Bool, self._motors_cb)
        self.data_topic = rospy.Subscriber('cb_test_data', CounterbalanceTestData, self._data_callback)

        self._model_file = model_file

        self._ok = True

    @property
    def has_data(self): return self._data is not None

    def _motors_cb(self, msg):
        self._motors_halted = msg.data
        
        if self._motors_halted:
            print("Motors halted. Adjustment data will be invalid.", file=sys.stderr)

        self._ok = self._ok and not self._motors_halted

    @property
    def ok(self): return self._ok

    def _data_callback(self, msg):
        self._data = msg
        
    def process_results(self):
        data = CounterbalanceAnalysisData(self._data)
        params = CounterbalanceAnalysisParams(self._data)

        if not _check_valid(params):
            print("Unable to calculate adjustment, invalid data", file=sys.stderr)
            return

        (secondary, cb_bar) = calc_cb_adjust(data, self._model_file)

        if (abs(secondary) > 25 or abs(cb_bar) > 25):
            print("Unable to calculate CB adjustment. This could mean the counterbalance is extremely out of adjustment, or your training data is invalid.", file=sys.stderr)
            return

        print('Calculated counterbalance adjustment recommendations:')
        print('\tSecondary Spring: %.1f (%s)' % (abs(secondary), dir(secondary)))
        print('\tArm Gimbal Shaft: %.1f (%s)' % (abs(cb_bar), dir(cb_bar)))
        print('Make sure to follow proper adjustment procedures if you choose to make an adjustment.\n')
        print('Your counterbalance state is up to you, these adjustments are only recommendations.\n')

            
if __name__ == '__main__':
    parser = OptionParser("./cb_check.py model_file")
    parser.add_option("-t", "--tol", action="store_true",
                      dest="tolerance", default=False,
                      help="Print recommended tolerances and exit")
    
    options, args = parser.parse_args(rospy.myargv())
    
    if options.tolerance:
        print('Recommended CB tolerances for secondary spring, arm gimbal shaft')
        print('\tSecondary spring: +/-%.1f' % SPRING_TOL)
        print('\tArm Gimbal Shaft: +/-%.1f' % BAR_TOL)
        print('If your recommended adjustment is less than this many turns, do not adjust your CB')
        sys.exit()

    if len(args) < 2:
        parser.error("No model file specified. Please give training data file to calculate adjustment")
        sys.exit(2)

    if not os.path.exists(args[1]):
        parser.error("Model file does not exist. Please give training data file to calculate adjustment")
        sys.exit(2)

    rospy.init_node('cb_analysis')
    app = CounterbalanceCheck(args[1])
    try:
        my_rate = rospy.Rate(5)
        while app.ok and not app.has_data and not rospy.is_shutdown():
            my_rate.sleep()

        if not app.ok:
            print("Unable to calculate adjustment. Check motors halted. Please retry.", file=sys.stderr)
            sys.exit(2)

        if not rospy.is_shutdown():
            app.process_results()
    except KeyboardInterrupt, e:
        pass
    except Exception, e:
        print('Caught Exception in CB application check')
        import traceback
        traceback.print_exc()
