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
##\brief Analyzes results from counterbalance test controller

PKG = 'pr2_counterbalance_check'
import roslib
roslib.load_manifest(PKG)

import os
import rospy

from pr2_self_test_msgs.srv import TestResult, TestResultRequest
from std_msgs.msg import Bool
from joint_qualification_controllers.msg import CounterbalanceTestData

from pr2_counterbalance_check.counterbalance_analysis import *


class CounterbalanceAnalyzer:
    def __init__(self):
        self._sent_results = False
        self._motors_halted = True
        self._data = None

        self.motors_topic = rospy.Subscriber('pr2_ethercat/motors_halted', Bool, self._motors_cb)
        self.data_topic = rospy.Subscriber('cb_test_data', CounterbalanceTestData, self._data_callback)
        self._result_service = rospy.ServiceProxy('test_result', TestResult)

        self._model_file = rospy.get_param('~model_file', None)


    def has_data(self):
        return self._data is not None

    def send_results(self, r):
        if not self._sent_results:
            try:
                rospy.wait_for_service('test_result', 10)
            except:
                rospy.logerr('Wait for service \'test_result\' timed out! Unable to send results.')
                return False
                
            self._result_service.call(r)
            self._sent_results = True

            return True
            
    def test_failed_service_call(self, except_str = ''):
        rospy.logerr(except_str)
        r = TestResultRequest()
        r.html_result = except_str
        r.text_summary = 'Caught exception, automated test failure.'
        r.result = TestResultRequest.RESULT_FAIL
        self.send_results(r)

    def _motors_cb(self, msg):
        self._motors_halted = msg.data

    def _data_callback(self, msg):
        self._data = msg
        
    def process_results(self):
        msg = self._data
        try:
            data = CounterbalanceAnalysisData(msg)
            params = CounterbalanceAnalysisParams(msg)

            lift_effort_result = analyze_lift_efforts(params, data)
            lift_effort_plot = plot_efforts_by_lift_position(params, data)

            if params.flex_test:
                flex_effort_result = analyze_flex_efforts(params, data)
                lift_effort_contour = plot_effort_contour(params, data)
                flex_effort_contour = plot_effort_contour(params, data, False)

                if self._model_file and os.path.exists(self._model_file):
                    adjust_result = check_cb_adjustment(params, data, self._model_file)
                elif self._model_file:
                    adjust_result = CounterbalanceAnalysisResult()
                    adjust_result.result = False
                    adjust_result.html = '<p>CB model file is missing. File %s does not exist. This file is used for testing the CB adjustment.</p>\n' % self._model_file
                    adjust_result.summary = 'CB model file missing, unable to analyze'
                else: # Don't check CB adjustment
                    adjust_result = CounterbalanceAnalysisResult()
                    adjust_result.result = True
                    adjust_result.html = '<p>Did not check counterbalance adjustment.</p>'
                                        

            html = []
            if params.flex_test:
                if self._model_file:
                    html.append('<H4>CB Adjustment Recommendations and Analysis</H4>')
                    html.append(adjust_result.html)
                    html.append('<p>Further information is for debugging and analysis information only.</p><br><hr size="2" />')

                html.append('<H4>Lift Effort Contour Plot</H4>')
                html.append('<img src=\"IMG_PATH/%s.png\" width=\"640\" height=\"480\" />' % (lift_effort_contour.title))
                html.append('<H4>Flex Effort Contour Plot</H4>')
                html.append('<img src=\"IMG_PATH/%s.png\" width=\"640\" height=\"480\" />' % (flex_effort_contour.title))
                
                html.append('<H4>Flex Effort Analysis</H4>')
                html.append(flex_effort_result.html)

            html.append('<H4>Lift Effort Analysis</H4>')
            html.append(lift_effort_result.html)
            html.append('<img src=\"IMG_PATH/%s.png\" width=\"640\" height=\"480\" />' % (lift_effort_plot.title))

            html.append('<p>Test Parameters</p>')
            html.append('<table border="1" cellpadding="2" cellspacing="0">')
            html.append('<tr><td><b>Joint</b></td><td><b>Dither Amplitude</b></td></tr>')
            html.append('<tr><td>%s</td><td>%s</td></tr>' % (params.lift_joint, params.lift_dither))
            if params.flex_test:
                html.append('<tr><td>%s</td><td>%s</td></tr>' % (params.flex_joint, params.flex_dither))
            html.append('</table>')
            html.append('<hr size=2>')
           
            r = TestResultRequest()
            r.html_result = '\n'.join(html)
            r.text_summary = ' '.join([lift_effort_result.summary])
            if params.flex_test:
                r.text_summary = ' '.join([r.text_summary, flex_effort_result.summary])

            r.result = TestResultRequest.RESULT_HUMAN_REQUIRED


            if params.flex_test and (lift_effort_result.result and 
                                     flex_effort_result.result and 
                                     adjust_result.result):
                r.result = TestResultRequest.RESULT_PASS
            elif not params.flex_test and lift_effort_result.result:
                r.result = TestResultRequest.RESULT_PASS


            r.plots = [ lift_effort_plot ]
            if params.flex_test:
                r.plots.append(lift_effort_contour)
                r.plots.append(flex_effort_contour)
            r.params = params.get_test_params()
            r.values = lift_effort_result.values
            if params.flex_test:
                r.values.extend(flex_effort_result.values)
                r.values.extend(adjust_result.values)

            # Adjustment required
            if params.flex_test and not adjust_result.result:
                r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
                r.text_summary = adjust_result.summary

            # Check motors halted
            if self._motors_halted:
                r.text_summary = 'Fail, motors halted. Check estop and power board.'
                r.html_result = '<H4>Motors Halted</H4>\n<p>Unable to analyze CB. Check estop and power board.</p>\n' + r.html_result
                r.result = TestResultRequest.RESULT_FAIL
            
            # Check timeout of controller
            if params.timeout_hit:
                r.text_summary = 'Fail, controller timeout hit. Timeout = %.ds. Test terminated early.' % (int(params.named_params["Timeout"]))
                r.html_result = '<H4>Timeout Hit</H4>\n<p>Unable to analyzer CB. Controller timeout hit.</p>\n' + r.html_result
                r.result = TestResultRequest.RESULT_FAIL


            self.send_results(r)
        except Exception, e:
            import traceback
            self.test_failed_service_call(traceback.format_exc());

            
if __name__ == '__main__':
    rospy.init_node('cb_analyzer')
    app = CounterbalanceAnalyzer()
    try:
        my_rate = rospy.Rate(5)
        while not app.has_data() and not rospy.is_shutdown():
            my_rate.sleep()

        if not rospy.is_shutdown():
            app.process_results()

        rospy.spin()
    except KeyboardInterrupt, e:
        pass
    except Exception, e:
        print('Caught Exception in CB application')
        import traceback
        traceback.print_exc()
        result_service = rospy.ServiceProxy('test_result', TestResult)        
        
        r = TestResultRequest()
        r.html_result = traceback.format_exc()
        r.text_summary = 'Caught exception, automated test failure.'
        r.plots = []
        r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
        result_service.call(r)
