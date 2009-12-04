#!/usr/bin/env python
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

##\author Kevin Watts

PKG = "qualification"

import roslib; roslib.load_manifest(PKG)

import numpy
import math

import sys
import os
from time import sleep

import rospy

import matplotlib
import matplotlib.pyplot as plot
from StringIO import StringIO

from qualification.msg import Plot
from qualification.srv import TestResult, TestResultRequest
from qualification.analysis import *

from joint_qualification_controllers.srv import WristDiffData, WristDiffDataResponse

import traceback

class WristRollHysteresisData(HysteresisData):
    def __init__(self, srv):
        left_min = int(0.05 * len(srv.left_turn.roll_position))
        left_max = int(0.95 * len(srv.left_turn.roll_position))
        right_min = int(0.05 * len(srv.right_turn.roll_position))
        right_max = int(0.95 * len(srv.right_turn.roll_position))

        self.pos_flex_effort = numpy.array(srv.left_turn.flex_effort) [left_min: left_max]
        self.neg_flex_effort = numpy.array(srv.right_turn.flex_effort)[right_min: right_max]

        self.positive = HysteresisDirectionData(srv.left_turn.roll_position, srv.left_turn.roll_effort, srv.left_turn.roll_velocity)
        self.negative = HysteresisDirectionData(srv.right_turn.roll_position, srv.right_turn.roll_effort, srv.right_turn.roll_velocity)


class WristRollHysteresisParams(HysteresisParameters):
    def __init__(self, srv):
        self.joint_name  = srv.roll_joint
        self.p_gain      = srv.roll_pid[0]
        self.i_gain      = srv.roll_pid[1]
        self.d_gain      = srv.roll_pid[2]
        self.i_clamp     = srv.roll_pid[3]

        self.velocity    = srv.arg_value[1]
        self.tolerance   = srv.arg_value[2]
        self.sd_max      = srv.arg_value[3]

        self.timeout     = srv.arg_value[4]
        self.pos_effort  = srv.arg_value[5]
        self.neg_effort  = srv.arg_value[6]

        self.range_max   = 0
        self.range_min   = 0
        self.slope       = 0

        # Subclass params only
        self.flex_joint   = srv.flex_joint
        self.flex_tol     = srv.arg_value[7]
        self.flex_max     = srv.arg_value[8]
        self.flex_sd      = srv.arg_value[9]
        self.flex_p_gain  = srv.flex_pid[0]
        self.flex_i_gain  = srv.flex_pid[1]
        self.flex_d_gain  = srv.flex_pid[2]
        self.flex_i_clamp = srv.flex_pid[3]

    def get_test_params(self):
        test_params = HysteresisParameters.get_test_params(self)

        test_params.append(TestParam("Flex Joint", self.flex_joint))
        test_params.append(TestParam("Flex Tolerance", str(self.flex_tol)))
        test_params.append(TestParam("Flex Max", str(self.flex_max)))
        test_params.append(TestParam("Flex SD", str(self.flex_sd)))
        test_params.append(TestParam("Flex P Gain", str(self.flex_p_gain)))
        test_params.append(TestParam("Flex I Gain", str(self.flex_i_gain)))
        test_params.append(TestParam("Flex D Gain", str(self.flex_d_gain)))
        test_params.append(TestParam("Flex I Clamp", str(self.flex_i_clamp)))

        return test_params

class WristDiffAnalysis:
    def __init__(self):
        self.data_sent = False
        rospy.init_node('wrist_diff_analysis')
        self.data_srv = rospy.Service('/test_data', WristDiffData, self.on_wrist_data)
        self.result_service = rospy.ServiceProxy('test_result', TestResult)
        
    def test_failed_service_call(self, except_str = ''):
        rospy.logerr(except_str)
        r = TestResultRequest()
        r.html_result = except_str
        r.text_summary = 'Caught exception, automated test failure.'
        r.plots = []
        r.result = TestResultRequest.RESULT_FAIL
        self.send_results(r)

    def send_results(self, test_result):
        if not self.data_sent:
            rospy.wait_for_service('test_result', 5)
            self.result_service.call(test_result)
            self.data_sent = True

    def on_wrist_data(self, srv):
        r = TestResultRequest()
        r.html_result = ''
        r.text_summary = 'No data.'
        r.plots = []
        r.result = TestResultRequest.RESULT_FAIL

        params_html = self.controller_params(srv)

        if srv.arg_value[4] < 0:
            r.text_summary = 'Wrist difference controller timed out'
            r.result = TestResultRequest.RESULT_FAIL
            r.html_result = params_html
            self.send_results(r)
            return WristDiffDataResponse()

        data = WristRollHysteresisData(srv)

        if not self.hysteresis_data_present(data):
            r.text_summary = 'Wrist difference controller didn\'t generate enough data.'
            r.result = TestResultRequest.RESULT_FAIL
            r.html_result = params_html
            self.send_results(r)
            return WristDiffDataResponse()

        params = WristRollHysteresisParams(srv)
        roll_result = effort_analysis(params, data)
        
        flex_stat, flex_html = self.flex_analysis(params, data)

        roll_plot = plot_effort(params, data)
        flex_plot = self.plot_flex_effort(params, data)
        r.plots.append(roll_plot)
        r.plots.append(flex_plot)
        
        r.text_summary = self.make_summary(roll_result.result, flex_stat)

        if roll_result.result and flex_stat:
            r.result = TestResultRequest.RESULT_PASS
        elif flex_stat: ##\todo Remove this after first 5 units
            r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
        else:
            r.result = TestResultRequest.RESULT_FAIL

        result_html = ['<H4 align=center>Flex Effort</H4>\n']
        result_html.append('<p>Flex effort should be close to zero during roll hysteresis.</p>\n')
        result_html.append('<img src=\"IMG_PATH/%s.png\", width = 640, height = 480/>\n' % flex_plot.title)
        result_html.append(flex_html)
        result_html.append('<hr size="2">\n')
        result_html.append('<H4 align=center>Roll Hysteresis</H4>\n')
        result_html.append('<p>Wrist roll hysteresis. The efforts should be close to the expected values.</p>\n')
        result_html.append('<img src=\"IMG_PATH/%s.png\", width = 640, height = 480/>\n' % roll_plot.title)
        result_html.append(roll_result.html)
        result_html.append('<hr size="2">\n')
        result_html.append(params_html)

        r.html_result = ''.join(result_html)

        r.params = params.get_test_params()
        r.values = roll_result.values

        self.send_results(r)

        return WristDiffDataResponse()

    def make_summary(self, roll_stat, flex_stat):
        if flex_stat and roll_stat:
            return "Wrist Difference check OK. Motors are symmetric."
        if flex_stat and not roll_stat:
            return "Wrist roll hysteresis failed. Wrist flex effort OK."
        if not flex_stat and roll_stat:
            return "Wrist roll hysteresis OK. Wrist flex effort failed."
        return "Wrist roll hystereis and flex effort failed."

    # Add table of all test params
    def controller_params(self, srv):
        param_html = ['<p>Test parameters from controller.</p><br>\n']
        param_html.append('<table border="1" cellpadding="2" cellspacing="0">\n')
        param_html.append('<tr><td><b>Name</b></td><td><b>Value</b></td></tr>\n')
        param_html.append('<tr><td>Flex joint</td><td>%s</td></tr>\n' % srv.flex_joint)
        param_html.append('<tr><td>Roll joint</td><td>%s</td></tr>\n' % srv.roll_joint)
        for i in range(0, len(srv.arg_name)):
            param_html.append('<tr><td>%s</td><td>%.2f</td></tr>\n' % (srv.arg_name[i], srv.arg_value[i]))
            
        param_html.append('<tr><td>Flex P</td><td>%.2f</td></tr>\n' % srv.flex_pid[0])
        param_html.append('<tr><td>Flex I</td><td>%.2f</td></tr>\n' % srv.flex_pid[1])
        param_html.append('<tr><td>Flex D</td><td>%.2f</td></tr>\n' % srv.flex_pid[2])
        param_html.append('<tr><td>Flex I Clamp</td><td>%.2f</td></tr>\n' % srv.flex_pid[3])
        param_html.append('<tr><td>Roll P</td><td>%.2f</td></tr>\n' % srv.roll_pid[0])
        param_html.append('<tr><td>Roll I</td><td>%.2f</td></tr>\n' % srv.roll_pid[1])
        param_html.append('<tr><td>Roll D</td><td>%.2f</td></tr>\n' % srv.roll_pid[2])
        param_html.append('<tr><td>Roll I Clamp</td><td>%.2f</td></tr>\n' % srv.roll_pid[3])
        
        param_html.append('</table>\n')
        
        return ''.join(param_html)

    def plot_flex_effort(self, params, data):
        # Can move to separate plotting function
        # Plot the analyzed data
        flex_max_tol = params.flex_max

        fig = plot.figure(1)
        axes1 = fig.add_subplot(211)
        axes2 = fig.add_subplot(212)
        axes2.set_xlabel('Roll Position')
        axes1.set_ylabel('Effort (+ dir)')
        axes2.set_ylabel('Effort (- dir)')

        axes1.plot(data.positive.position, data.pos_flex_effort, 'b--', label='_nolegend_')
        axes1.axhline(y = flex_max_tol, color = 'y', label='Max Value')
        axes1.axhline(y = - flex_max_tol, color = 'y', label='_nolegend_')
        axes1.axhline(y = 0, color = 'r', label='_nolegend_')

        axes2.plot(data.negative.position, data.neg_flex_effort, 'b--', label='_nolegend_')
        axes2.axhline(y = flex_max_tol, color = 'y', label='Max Value')
        axes2.axhline(y = - flex_max_tol, color = 'y', label='_nolegend_')
        axes2.axhline(y = 0, color = 'r', label='_nolegend_')

        fig.text(.4, .95, 'Wrist Flex Effort')
                
        stream = StringIO()
        plot.savefig(stream, format = "png")
        image = stream.getvalue()
        
        p = Plot()
        p.title = "wrist_diff_flex_effort"
        p.image = image
        p.image_format = "png"
        
        plot.close()
        
        return p

    def hysteresis_data_present(self, data):
        return data.positive.position.size > 100 and data.negative.position.size > 100

    def flex_analysis(self, params, data):
        flex_max = numpy.average(data.pos_flex_effort)
        flex_max_sd = numpy.std(data.pos_flex_effort)
        flex_min = numpy.average(data.neg_flex_effort)
        flex_min_sd = numpy.std(data.neg_flex_effort)
        flex_max_val = max(numpy.max(data.pos_flex_effort), numpy.max(data.neg_flex_effort))
        flex_min_val = min(numpy.min(data.pos_flex_effort), numpy.min(data.neg_flex_effort))

        max_msg = '<div class="pass">OK</div>'
        if abs(flex_max) > params.flex_tol:
            max_msg = '<div class="error">FAIL</div>'

        min_msg = '<div class="pass">OK</div>'
        if abs(flex_min) > params.flex_tol:
            min_msg = '<div class="error">FAIL</div>'

        max_val_msg = '<div class="pass">OK</div>'
        if abs(flex_max_val) > params.flex_max:
            max_val_msg = '<div class="error">FAIL</div>'

        min_val_msg = '<div class="pass">OK</div>'
        if abs(flex_min_val) > params.flex_max:
            min_val_msg = '<div class="error">FAIL</div>'

        max_sd_msg = '<div class="pass">OK</div>'
        if abs(flex_max_sd) > params.flex_sd:
            max_sd_msg = '<div class="error">FAIL</div>'

        min_sd_msg = '<div class="pass">OK</div>'
        if abs(flex_min_sd) > params.flex_sd:
            min_sd_msg = '<div class="error">FAIL</div>'

        ok = abs(flex_max) < params.flex_tol and abs(flex_min) < params.flex_tol
        ok = ok and abs(flex_min_val) < params.flex_max and abs(flex_max_val) < params.flex_max

        if ok:
            html = ['<p>Wrist flex effort is OK.</p>\n']
        else:
            html = ['<p>Wrist flex effort failed. Check graphs.</p>\n']

        html.append('<p>Flex effort maximum values. Allowed maximum: %.2f</p>\n' % (params.flex_max))
        html.append('<table border="1" cellpadding="2" cellspacing="0">\n')
        html.append('<tr><td></td><td><b>Effort</b></td><td><b>Status</b></td></tr>\n')
        html.append('<tr><td><b>Max Value</b></td><td>%.2f</td><td>%s</td></tr>\n' % (flex_max_val, max_val_msg))
        html.append('<tr><td><b>Min Value</b></td><td>%.2f</td><td>%s</td></tr>\n' % (flex_min_val, min_val_msg))
        html.append('</table>\n')

        html.append('<br>\n')
        html.append('<p>Flex effort average and noise, both directions. Allowed absolute average: %.2f. Allowed noise: %.2f</p>\n' % (params.flex_tol, params.flex_sd))
        html.append('<table border="1" cellpadding="2" cellspacing="0">\n')
        html.append('<tr><td><b>Effort Avg</b></td><td><b>Status</b></td><td><b>Effort SD</b></td><td><b>SD Status</b></td></tr>\n')
        html.append('<tr><td>%.2f</td><td>%s</td><td>%.2f</td><td>%s</td></tr>\n' % (flex_max, max_msg, flex_max_sd, max_sd_msg))
        html.append('<tr><td>%.2f</td><td>%s</td><td>%.2f</td><td>%s</td></tr>\n' % (flex_min, min_msg, flex_min_sd, min_sd_msg))
        html.append('</table>\n')
        
        return ok, ''.join(html)
         
if __name__ == '__main__':
    wda = WristDiffAnalysis()
    try:
        rospy.spin()
    except KeyboardInterrupt, e:
        pass
    except:
        rospy.logerr("Exception in analysis: %s", traceback.format_exc())
        wda.test_failed_service_call(traceback.format_exc())
