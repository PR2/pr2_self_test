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

from qualification.msg import Plot, TestParam, TestValue
from qualification.srv import TestResult, TestResultRequest
from qualification.analysis import *

from joint_qualification_controllers.srv import WristDiffData, WristDiffDataResponse

import traceback

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
        self._analyze_wrist_data(srv)

        return WristDiffDataResponse()

    def _analyze_wrist_data(self, srv):
        r = TestResultRequest()
        r.html_result = ''
        r.text_summary = 'No data.'
        r.plots = []
        r.result = TestResultRequest.RESULT_FAIL

        params = WristRollHysteresisParams(srv)
        r.params = params.get_test_params()

        if srv.timeout:
            r.text_summary = 'Wrist difference controller timed out'
            r.result = TestResultRequest.RESULT_FAIL
            self.send_results(r)
            return 

        data = WristRollHysteresisData(srv)

        if not wrist_hysteresis_data_present(data):
            r.text_summary = 'Wrist difference controller didn\'t generate enough data.'
            r.result = TestResultRequest.RESULT_FAIL
            self.send_results(r)
            return 

        # Normal hysteresis analysis on roll
        roll_result = effort_analysis(params, data)
        # Check flex efforts 
        flex_result = wrist_flex_analysis(params, data)

        roll_plot = plot_effort(params, data)
        flex_plot = plot_flex_effort(params, data)
        r.plots.append(roll_plot)
        r.plots.append(flex_plot)
        
        r.text_summary = make_wrist_test_summary(roll_result.result, flex_result.result)

        if roll_result.result and flex_result.result:
            r.result = TestResultRequest.RESULT_PASS
        elif flex_result.result: ##\todo Remove this after first 5 units
            r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
        else:
            r.result = TestResultRequest.RESULT_FAIL

        result_html = ['<H4 align=center>Flex Effort</H4>']
        result_html.append('<p>Flex effort should be close to zero during roll hysteresis.</p>')
        result_html.append('<img src=\"IMG_PATH/%s.png\", width = 640, height = 480/>' % flex_plot.title)
        result_html.append(flex_result.html)
        result_html.append('<hr size="2">')
        result_html.append('<H4 align=center>Roll Hysteresis</H4>')
        result_html.append('<p>Wrist roll hysteresis. The efforts should be close to the expected values.</p>')
        result_html.append('<img src=\"IMG_PATH/%s.png\", width = 640, height = 480/>' % roll_plot.title)
        result_html.append(roll_result.html)
        result_html.append('<hr size="2">')

        r.html_result = '\n'.join(result_html)

        r.values = roll_result.values + flex_result.values

        self.send_results(r)
         
if __name__ == '__main__':
    wda = WristDiffAnalysis()
    try:
        rospy.spin()
    except KeyboardInterrupt, e:
        pass
    except:
        rospy.logerr("Exception in analysis: %s", traceback.format_exc())
        wda.test_failed_service_call(traceback.format_exc())
