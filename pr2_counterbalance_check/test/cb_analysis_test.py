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

##\brief Unit tests of counterbalance analysis code

from __future__ import division

PKG = 'pr2_counterbalance_check'
import roslib

from pr2_counterbalance_check import *

import copy
import os, sys
import rostest, unittest

# Dummy classes to store data
class DummyCBAnalysisParams(object): pass
class DummyCBAnalysisData(object): pass
class DummyCBRunAnalysisData(object): pass
class DummyCBPositionAnalysisData(object): pass
class DummyJointPositionAnalysisData(object): pass

def get_position(minv, maxv, num_pts, i):
    return minv + (maxv - minv) * i / num_pts

def generate_data(params):
    data = DummyCBAnalysisData()
    data.lift_data = []
    for i in range(params.num_lifts):
        lift_data = DummyCBRunAnalysisData()
        lift_data.lift_position = get_position(params.min_lift, params.max_lift, params.num_lifts, i)
        lift_data.flex_data = []
        
        for j in range(params.num_flexes):
            fd = DummyCBPositionAnalysisData()
            fd.flex_position = get_position(params.min_flex, params.max_flex, params.num_flexes, j)
            
            fd.lift_hold = DummyJointPositionAnalysisData()
            fd.lift_hold.position_avg = lift_data.lift_position
            fd.lift_hold.position_sd  = 0.0
            fd.lift_hold.effort_avg   = j / 100.0 - params.num_flexes/200.0
            fd.lift_hold.effort_sd    = 0.0

            fd.flex_hold = DummyJointPositionAnalysisData()
            fd.flex_hold.position_avg = fd.flex_position
            fd.flex_hold.position_sd  = 0.0
            fd.flex_hold.effort_avg   = j / 100.0 - params.num_flexes/200.0
            fd.flex_hold.effort_sd    = 0.0
            
            lift_data.flex_data.append(fd)

        data.lift_data.append(lift_data)

    return data

class TestCounterbalanceAnalysis(unittest.TestCase):
    def setUp(self):
        # Set up parameters
        self.params = DummyCBAnalysisParams()
        self.params.lift_dither  = 5.0
        self.params.flex_dither  = 5.0
        self.params.lift_joint   = 'lift_joint'
        self.params.flex_joint   = 'flex_joint'
        self.params.timeout_hit  = False
        self.params.flex_test    = True # Set to False for lift-only test
        self.params.lift_mse     = 1.0
        self.params.lift_avg_abs = 1.0
        self.params.lift_avg_eff = 1.0
        self.params.flex_mse     = 1.0
        self.params.flex_avg_abs = 1.0
        self.params.flex_avg_eff = 1.0

        self.params.screw_tol = 2.0
        self.params.bar_tol   = 0.8

        self.params.num_lifts = 8
        self.params.min_lift = -0.2
        self.params.max_lift = 1.2

        self.params.num_flexes = 9
        self.params.min_flex = -1.8
        self.params.max_flex = -0.2

        # Set up data
        self.data = generate_data(self.params)

        # Model file for analyzing data
        self.model_file = os.path.join(roslib.packages.get_pkg_dir(PKG), 'cb_data/counterbalance_model.dat')
            

    def test_lift_effort(self):
        result = analyze_lift_efforts(self.params, self.data)
        self.assert_(result.result, "Lift effort result wasn't OK. %s\n%s" % (result.summary, result.html))

    def test_flex_effort(self):
        result = analyze_flex_efforts(self.params, self.data)
        self.assert_(result.result, "Flex effort result wasn't OK. %s\n%s" % (result.summary, result.html))


    
    
    def test_adjustment(self):
        """
        Test that CB adjustment runs successfully
        """
        (secondary, bar) = calc_cb_adjust(self.data, self.model_file)

        self.assert_(abs(secondary) < self.params.screw_tol and abs(bar) < self.params.bar_tol,
                     "Calculated adjustment didn't match. Adjustment: %.2f, %.2f" % (secondary, bar))

        adjust_result = check_cb_adjustment(self.params, self.data, self.model_file)
        self.assert_(adjust_result.result, "Adjustment result was unsuccessful! %s\n%s" % (adjust_result.summary, adjust_result.html))
        
        # Bad data has invalid number of dimensions
        bad_params = copy.deepcopy(self.params)
        bad_params.num_lifts = 7
        bad_params.max_lift = 1.0
        
        bad_data = generate_data(bad_params)

        (secondary, bar) = calc_cb_adjust(bad_data, self.model_file)

        self.assert_(abs(secondary) > 50 and abs(bar) > 50,
                     "Calculated adjustment successful on bad data. Adjustment: %.2f, %.2f" % (secondary, bar))
        
            
    def test_plots(self):
        p_contout_lift = plot_effort_contour(self.params, self.data, True)
        p_contout_flex = plot_effort_contour(self.params, self.data, False)

        p_eff = plot_efforts_by_lift_position(self.params, self.data)


if __name__ == '__main__':
    rostest.unitrun(PKG, 'test_cb_analysis', TestCounterbalanceAnalysis)
