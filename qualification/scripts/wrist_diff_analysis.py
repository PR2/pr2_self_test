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
import string
from time import sleep

import rospy

import matplotlib
import matplotlib.pyplot as plot
from StringIO import StringIO

from qualification.msg import Plot
from qualification.srv import TestResult, TestResultRequest

from joint_qualification_controllers.srv import WristDiffData, WristDiffDataResponse

import traceback

class WristRollHysteresisData:
    def __init__(self, srv):
        left_min = int(0.05 * len(srv.left_turn.roll_position))
        left_max = int(0.95 * len(srv.left_turn.roll_position))
        right_min = int(0.05 * len(srv.right_turn.roll_position))
        right_max = int(0.95 * len(srv.right_turn.roll_position))

        self.pos_roll_position = numpy.array(srv.left_turn.roll_position)[left_min: left_max]
        self.pos_roll_effort   = numpy.array(srv.left_turn.roll_effort)  [left_min: left_max]
        self.pos_flex_effort   = numpy.array(srv.left_turn.flex_effort)  [left_min: left_max]

        self.neg_roll_position = numpy.array(srv.right_turn.roll_position)[right_min: right_max]
        self.neg_roll_effort   = numpy.array(srv.right_turn.roll_effort)  [right_min: right_max]
        self.neg_flex_effort   = numpy.array(srv.right_turn.flex_effort)  [right_min: right_max]

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

        roll_stat, roll_html = self.roll_hysteresis_analysis(srv, data)
        
        flex_stat, flex_html = self.flex_analysis(srv, data)

        roll_plot = self.plot_roll_hysteresis(srv, data)
        flex_plot = self.plot_flex_effort(srv, data)
        r.plots.append(roll_plot)
        r.plots.append(flex_plot)
        
        r.text_summary = self.make_summary(roll_stat, flex_stat)

        if roll_stat and flex_stat:
            r.result = TestResultRequest.RESULT_PASS
        elif flex_stat: ##\todo Remove this
            r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
        else:
            r.result = TestResultRequest.RESULT_FAIL

        html = '<H4 align=center>Flex Effort</H4>\n'
        html += '<p>Flex effort should be close to zero during roll hysteresis.</p>\n'
        html += '<img src=\"IMG_PATH/%s.png\", width = 640, height = 480/>\n' % flex_plot.title
        html += flex_html
        html += '<hr size="2">\n'
        html += '<H4 align=center>Roll Hysteresis</H4>\n'
        html += '<p>Wrist roll hysteresis. The efforts should be close to the expected values.</p>\n'
        html += '<img src=\"IMG_PATH/%s.png\", width = 640, height = 480/>\n' % roll_plot.title
        html += roll_html
        html += '<hr size="2">\n'
        html += params_html

        r.html_result = html 

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
       html = '<p>Test parameters from controller.</p><br>\n'
       html += '<table border="1" cellpadding="2" cellspacing="0">\n'
       html += '<tr><td><b>Name</b></td><td><b>Value</b></td></tr>\n'
       html += '<tr><td>Flex joint</td><td>%s</td></tr>\n' % srv.flex_joint
       html += '<tr><td>Roll joint</td><td>%s</td></tr>\n' % srv.roll_joint
       for i in range(0, len(srv.arg_name)):
           html += '<tr><td>%s</td><td>%.2f</td></tr>\n' % (srv.arg_name[i], srv.arg_value[i])

       html += '<tr><td>Flex P</td><td>%.2f</td></tr>\n' % srv.flex_pid[0]
       html += '<tr><td>Flex I</td><td>%.2f</td></tr>\n' % srv.flex_pid[1]
       html += '<tr><td>Flex D</td><td>%.2f</td></tr>\n' % srv.flex_pid[2]
       html += '<tr><td>Flex I Clamp</td><td>%.2f</td></tr>\n' % srv.flex_pid[3]

       html += '<tr><td>Roll P</td><td>%.2f</td></tr>\n' % srv.roll_pid[0]
       html += '<tr><td>Roll I</td><td>%.2f</td></tr>\n' % srv.roll_pid[1]
       html += '<tr><td>Roll D</td><td>%.2f</td></tr>\n' % srv.roll_pid[2]
       html += '<tr><td>Roll I Clamp</td><td>%.2f</td></tr>\n' % srv.roll_pid[3]

       html += '</table>\n'
           
       return html

    def plot_flex_effort(self, srv, data):
        # Can move to separate plotting function
        # Plot the analyzed data
        flex_max_tol = srv.arg_value[8]

        fig = plot.figure(1)
        axes1 = fig.add_subplot(211)
        axes2 = fig.add_subplot(212)
        axes2.set_xlabel('Roll Position')
        axes1.set_ylabel('Effort (+ dir)')
        axes2.set_ylabel('Effort (- dir)')

        axes1.plot(data.pos_roll_position, data.pos_flex_effort, 'b--', label='_nolegend_')
        axes1.axhline(y = flex_max_tol, color = 'y', label='Error')
        axes1.axhline(y = - flex_max_tol, color = 'y', label='_nolegend_')
        axes1.axhline(y = 0, color = 'r', label='_nolegend_')

        axes2.plot(data.neg_roll_position, data.neg_flex_effort, 'b--', label='_nolegend_')
        axes2.axhline(y = flex_max_tol, color = 'y', label='Error')
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
        print 'Right size: %d, Left size: %d' % (data.pos_roll_position.size, data.neg_roll_position.size)
        if data.pos_roll_position.size < 100 or data.neg_roll_position.size < 100:
            return False
        return True

    def flex_analysis(self, srv, data):
        flex_tol = srv.arg_value[7] # Average values
        flex_max_tol = srv.arg_value[8] # Max obs. val
        flex_sd_tol = srv.arg_value[9] # Std. Dev. of effort

        flex_max = numpy.average(data.pos_flex_effort)
        flex_max_sd = numpy.std(data.pos_flex_effort)
        flex_min = numpy.average(data.neg_flex_effort)
        flex_min_sd = numpy.std(data.neg_flex_effort)
        flex_max_val = max(numpy.max(data.pos_flex_effort), numpy.max(data.neg_flex_effort))
        flex_min_val = min(numpy.min(data.pos_flex_effort), numpy.min(data.neg_flex_effort))

        max_msg = '<div class="pass">OK</div>'
        if abs(flex_max) > flex_tol:
            max_msg = '<div class="error">FAIL</div>'

        min_msg = '<div class="pass">OK</div>'
        if abs(flex_min) > flex_tol:
            min_msg = '<div class="error">FAIL</div>'

        max_val_msg = '<div class="pass">OK</div>'
        if abs(flex_max_val) > flex_max_tol:
            max_val_msg = '<div class="error">FAIL</div>'

        min_val_msg = '<div class="pass">OK</div>'
        if abs(flex_min_val) > flex_max_tol:
            min_val_msg = '<div class="error">FAIL</div>'

        max_sd_msg = '<div class="pass">OK</div>'
        if abs(flex_max_sd) > flex_sd_tol:
            max_sd_msg = '<div class="error">FAIL</div>'

        min_sd_msg = '<div class="pass">OK</div>'
        if abs(flex_min_sd) > flex_sd_tol:
            min_sd_msg = '<div class="error">FAIL</div>'

        ok = abs(flex_max) < flex_tol and abs(flex_min) < flex_tol
        ok = ok and abs(flex_min_val) < flex_max_tol and abs(flex_max_val) < flex_max_tol

        if ok:
            html = '<p>Wrist flex effort is OK.</p>\n'
        else:
            html = '<p>Wrist flex effort failed. Check graphs.</p>\n'
        html += '<p>Flex effort maximum values. Allowed maximum: %.2f</p>\n' % (flex_max_tol)
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td></td><td><b>Effort</b></td><td><b>Status</b></td></tr>\n'
        html += '<tr><td><b>Max Value</b></td><td>%.2f</td><td>%s</td></tr>\n' % (flex_max_val, max_val_msg)
        html += '<tr><td><b>Min Value</b></td><td>%.2f</td><td>%s</td></tr>\n' % (flex_min_val, min_val_msg)
        html += '</table>\n'

        html += '<br>\n'
        html += '<p>Flex effort average and noise, both directions. Allowed absolute average: %.2f. Allowed noise: %.2f</p>\n' % (flex_tol, flex_sd_tol)
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Effort Avg</b></td><td><b>Status</b></td><td><b>Effort SD</b></td><td><b>SD Status</b></td></tr>\n'
        html += '<tr><td>%.2f</td><td>%s</td><td>%.2f</td><td>%s</td></tr>\n' % (flex_max, max_msg, flex_max_sd, max_sd_msg)
        html += '<tr><td>%.2f</td><td>%s</td><td>%.2f</td><td>%s</td></tr>\n' % (flex_min, min_msg, flex_min_sd, min_sd_msg)
        html += '</table>\n'
        

        return ok, html

  
    def roll_hysteresis_analysis(self, srv, data):
        max_avg = numpy.average(data.pos_roll_effort)
        min_avg = numpy.average(data.neg_roll_effort)
        max_sd = numpy.std(data.pos_roll_effort)
        min_sd = numpy.std(data.neg_roll_effort)

        tol_percent = srv.arg_value[2]
        sd_percent = srv.arg_value[3]

        max_exp = srv.arg_value[5]
        min_exp = srv.arg_value[6]
        tolerance = tol_percent * abs(max_exp - min_exp)
        sd_max = sd_percent * abs(max_exp - min_exp)

        max_ok = abs(max_avg - max_exp) < tolerance
        min_ok = abs(min_avg - min_exp) < tolerance
        
        max_even = max_sd < sd_max
        min_even = min_sd < sd_max
        
        if max_ok and max_even:
            max_msg = '<div class="pass">OK</div>'
        elif max_ok and not max_even:
            max_msg = '<div class="warning">UNEVEN</div>'
        else:
            max_msg = '<div class="error">FAIL</div>'

        if min_ok and min_even:
            min_msg = '<div class="pass">OK</div>'
        elif min_ok and not min_even:
            min_msg = '<div class="warning">UNEVEN</div>'
        else:
            min_msg = '<div class="error">FAIL</div>'

        html = '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Name</b></td><td><b>Value</b></td><td><b>Expected</b></td><td><b>Tolerance</b></td><td><b>SD Percent</b></td><td><b>SD Max</b></td><td><b>Status</b></td></tr>\n'
        html += '<tr><td>%s</td><td>%.2f</td><td>%.2f</td><td>%.2f</td><td>%.2f</td><td>%.2f</td><td>%s</td></tr>\n' % ('Max Effort', max_avg, max_exp, tolerance, max_sd, sd_max, max_msg)
        html += '<tr><td>%s</td><td>%.2f</td><td>%.2f</td><td>%.2f</td><td>%.2f</td><td>%.2f</td><td>%s</td></tr>\n' % ('Min Effort', min_avg, min_exp, tolerance, min_sd, sd_max, min_msg)
        html += '</table>\n'

        if max_ok and min_ok and min_even and max_even:
            return True, "<p>Hysteresis data was OK</p>\n" + html
        if max_ok and min_ok:
            return False, "<p>Hysteresis effort was uneven.</p>\n" + html
        else:
            return False, "<p>Roll hysteresis failed.</p>\n" + html

    def plot_roll_hysteresis(self, srv, data):
        max_sd = srv.arg_value[3]
        max_avg = srv.arg_value[5]
        min_avg = srv.arg_value[6]
 
        # Can move to separate plotting function
        # Plot the analyzed data
        fig = plot.figure(1)
        axes1 = fig.add_subplot(211)
        axes2 = fig.add_subplot(212)
        axes2.set_xlabel('Position')
        axes1.set_ylabel('Effort (+ dir)')
        axes2.set_ylabel('Effort (- dir)')
        axes1.plot(data.pos_roll_position, data.pos_roll_effort, 'b--', label='_nolegend_')
        axes2.plot(data.neg_roll_position, data.neg_roll_effort, 'b--', label='_nolegend_')
        # Add error bars for SD
        axes1.axhline(y = max_avg, color = 'r', label='Average')
        axes1.axhline(y = max_avg + max_sd, color = 'y', label='Error bars')
        axes1.axhline(y = max_avg - max_sd, color = 'y', label='_nolegend_')
        
        axes2.axhline(y = min_avg, color = 'r', label='Average')
        axes2.axhline(y = min_avg + max_sd, color = 'y', label='Error')
        axes2.axhline(y = min_avg - max_sd, color = 'y', label='_nolegend_')
        
        # Add expected efforts to both plots
        axes1.axhline(y = max_avg, color = 'g', label='_nolegend_')
        axes2.axhline(y = min_avg, color = 'g', label='Expected')
        
        fig.text(.3, .95, 'Wrist Roll Hysteresis')
        axes1.legend(shadow=True)
        
        stream = StringIO()
        plot.savefig(stream, format = "png")
        image = stream.getvalue()
        
        p = Plot()
        p.title = "wrist_diff_roll_hysteresis"
        p.image = image
        p.image_format = "png"
        
        plot.close()
        
        return p

        
if __name__ == '__main__':
    wda = WristDiffAnalysis()
    sleep(1)
    rospy.loginfo("Starting analysis node")
    try:
        rospy.loginfo("Spinning")
        rospy.spin()
        rospy.loginfo("Analysis node stopped spinning")
    except KeyboardInterrupt, e:
        rospy.loginfo("Keyboard interrupt in analysis")
        pass
    except:
        print 'Exception'
        rospy.logerr("Exception in analysis: %s", traceback.format_exc())
        wda.test_failed_service_call(traceback.format_exc())
