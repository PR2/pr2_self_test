#!/usr/bin/env python
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

##\author Matthew Piccoli, Kevin Watts
##\brief Qualification of gripper tips
##
## This test checks that the gripper tip sensors on the PR2 gripper. It checks 
## that each tip has the correct number of tips, and that each one is reading.
## After that, the node commands the gripper to repeatedly open and close and measures
## the total force of the interior gripper sensors. It compares this total force to a
## cubic polynomial. After checking the relationship between total pressure and gripper 
## force, and checking the differences between the two gripper tips, the node reports
## success or failure.

from __future__ import with_statement

PKG = 'qualification'
import roslib
roslib.load_manifest(PKG)

import numpy
import matplotlib
import matplotlib.pyplot as plot
from StringIO import StringIO
from time import sleep
import threading
import os

import rospy
from pr2_self_test_msgs.srv import TestResult, TestResultRequest
from pr2_self_test_msgs.msg import Plot
from std_msgs.msg import Float64
from pr2_msgs.msg import PressureState

lvl_dict = {0: 'OK', 1: 'Questionable', 2: 'Failure' }

##\brief Performs testing PR2 gripper tip sensors
class FingertipQualification:
    def __init__(self):
        self._mutex = threading.Lock()
        self.set_cmd = 0.0
        
        self.pub = rospy.Publisher('r_gripper_effort_controller/command', Float64)

        self.l_finger_tip = None
        self.r_finger_tip = None

        rospy.init_node('fingertip_qualification')

        # Squeezing params
        self.initial = rospy.get_param('~grasp_force_initial', -25.0)
        self.increment = rospy.get_param('~grasp_force_increment', -25.0) 
        self.num_increments = rospy.get_param('~grasp_increments', 4) 

        # Equation to fit against
        self.x0 = rospy.get_param('~x^0', 0)
        self.x1 = rospy.get_param('~x^1', 0)
        self.x2 = rospy.get_param('~x^2', 0)
        self.x3 = rospy.get_param('~x^3', 0)

        # Gripper parameters
        self.fingertip_refresh = rospy.get_param('~fingertip_refresh_hz', 25)
        self.num_sensors = rospy.get_param('~num_sensors', 22)
        self.num_sensors_outside = rospy.get_param('~num_sensors_outside', 7)
        self.force = self.initial

        # Tolerances
        self._tol_max_quest = rospy.get_param('~tol_max_question', 10)
        self._tol_max_fail = rospy.get_param('~tol_max_fail', 20)
        self._tol_avg_quest = rospy.get_param('~tol_avg_question', 10)
        self._tol_avg_fail = rospy.get_param('~tol_avg_fail', 20)

        self._diff_max_quest = rospy.get_param('~diff_max_question', 5)
        self._diff_max_fail = rospy.get_param('~diff_max_fail', 10)
        self._diff_avg_quest = rospy.get_param('~diff_avg_question', 5)
        self._diff_avg_fail = rospy.get_param('~diff_avg_fail', 10)
        self._diff_avg_abs_quest = rospy.get_param('~diff_avg_abs_question', 5)
        self._diff_avg_abs_fail = rospy.get_param('~diff_avg_abs_fail', 10)
        
        # Don't look at tip values, just check the connections
        self.check_connect_only = rospy.get_param('~check_connect_only', False)
        self.expect_no_connect  = rospy.get_param('~not_connected', False)

        self.data_sent = False
        self.result_service = rospy.ServiceProxy('/test_result', TestResult)

        self._connection_data = ''

        # Data
        self._forces = []
        self._expected = []
        self._tip0 = []
        self._tip1 = []

        self._pressure_topic = 'pressure/r_gripper_motor'
        rospy.Subscriber(self._pressure_topic, PressureState, self.pressure_callback)

    ##\brief Callback for gripper pressure topic
    def pressure_callback(self, data):
        with self._mutex:
            self.l_finger_tip = data.l_finger_tip
            self.r_finger_tip = data.r_finger_tip

    ##\brief Record errors in analysis
    def test_failed_service_call(self, except_str = ''):
        rospy.logerr(except_str)
        r = TestResultRequest()
        r.html_result = except_str
        r.text_summary = 'Caught exception, automated test failure.'
        r.plots = []
        r.result = TestResultRequest.RESULT_FAIL
        self.send_results(r)

    ##\brief Send test results to qualification system
    def send_results(self, test_result):
        if self.data_sent:
            return 

        rospy.wait_for_service('test_result', 15)
        self.result_service.call(test_result)
        self.data_sent = True

    ##\brief Command open, wait for 1 sec
    def open_gripper(self):
        self.set_cmd = 50
        self.pub.publish(self.set_cmd)
        sleep(1)
        
    ##\brief Command close at desired force, wait for 4 sec
    def close_gripper(self):
        self.set_cmd = self.force
        self.pub.publish(self.set_cmd)
        sleep(4)
    
    ##\brief Make sure we have correct number of gripper sensors on each tip, and all are OK
    def check_connected(self):
        sleep(1/self.fingertip_refresh*2)
        with self._mutex:
            if self.l_finger_tip is None or self.r_finger_tip is None:
                r = TestResultRequest()
                r.text_summary = 'No gripper tip data.'
                
                r.html_result = '<p>No gripper tip data. Check connections. L tip OK: %s, R tip OK: %s.</p>\n' % (str(self.l_finger_tip is None), str(self.r_finger_tip is None))
                r.html_result +=  '<hr size="2">\n' + self._write_equation()
                r.html_result += '<hr size="2">\n' + self._write_params()
                r.html_result += '<hr size="2">\n' + self._write_tols()
                r.result = TestResultRequest.RESULT_FAIL
                self.send_results(r)

            if len(self.l_finger_tip) != self.num_sensors or len(self.r_finger_tip) != self.num_sensors:
                r = TestResultRequest()
                r.text_summary = 'Incorrect number of sensors. Expected: %d.' % self.num_sensors
                
                r.html_result = '<p>Incorrect number of sensors. Expected: %d. L tip: %d, tip 1: %d.</p>\n' % (self.num_sensors, len(self.l_finger_tip), len(self.r_finger_tip))
                r.html_result +=  '<hr size="2">\n' + self._write_equation()
                r.html_result += '<hr size="2">\n' + self._write_params()
                r.html_result += '<hr size="2">\n' + self._write_tols()
                r.result = TestResultRequest.RESULT_FAIL
                self.send_results(r)

            ok = True
            tips_bad = True
            connect_table = '<table border="1" cellpadding="2" cellspacing="0">\n'
            connect_table += '<tr><td><b>Sensor</b></td><td><b>L tip</b></td><td><b>R tip</b></td></tr>\n'
            for i in range(0, self.num_sensors):
                tip0 = 'OK'
                tip1 = 'OK'
                if self.l_finger_tip[i] == 0 or self.l_finger_tip[i] == -1:
                    ok = False
                    tip0 = 'No data'  
                else:
                    tips_bad = False

                if self.r_finger_tip[i] == 0 or self.r_finger_tip[i] == -1:
                    ok = False
                    tip1 = 'No data'
                else:
                    tips_bad = False

                connect_table += '<tr><td>%d</td><td>%s</td><td>%s</td></tr>\n' % (i, tip0, tip1)
            connect_table += '</table>\n'

        connect_str = 'OK'
        if not ok:
            connect_str = 'Not connected'
        self._connected_data = '<p align=center><b>Tip connections: %s</b></p><br>\n' % connect_str
        self._connected_data += connect_table
        
        if self.expect_no_connect:
           r = TestResultRequest()
           
           r.html_result = self._connected_data 
           r.html_result += '<hr size="2">\n' + self._write_params()
           if tips_bad:
               r.text_summary = 'Gripper tips not connected - OK'
               r.result = TestResultRequest.RESULT_PASS
           else:
               r.text_summary = 'Gripper tips connected, expected no connection'
               r.result = TestResultRequest.RESULT_FAIL
           
           self.send_results(r)
           return tips_bad
        
        if self.check_connect_only:
            r = TestResultRequest()

            r.html_result = self._connected_data 
            r.html_result += '<hr size="2">\n' + self._write_params()
            if ok:
                r.text_summary = 'Gripper tips connected'
                r.result = TestResultRequest.RESULT_PASS
            else:
                r.text_summary = 'Gripper tips not connected'
                r.result = TestResultRequest.RESULT_FAIL
            self.send_results(r)
            return ok

        if not ok:
            r = TestResultRequest()
            r.text_summary = 'Not connected.'
            r.html_result = self._connected_data 
            r.html_result +=  '<hr size="2">\n' + self._write_equation()
            r.html_result += '<hr size="2">\n' + self._write_params()
            r.html_result += '<hr size="2">\n' + self._write_tols()
            r.result = TestResultRequest.RESULT_FAIL
            self.send_results(r)
                
        return ok

    ##\brief Increment commanded force by increment
    def increment_value(self):
        self.force += self.increment
    
    ##\brief Record "zero" of gripper, when tips are open
    def record_zero_val(self):
        with self._mutex:
            self.starting_sum0 = 0
            self.starting_sum1 = 0
            for i in range(self.num_sensors_outside, self.num_sensors):
                self.starting_sum0 += self.l_finger_tip[i]
                self.starting_sum1 += self.r_finger_tip[i]
      
    ##\brief Record sum of gripper tip pressure at the commanded force
    def record_increase(self):
        with self._mutex:
            current_sum0 = 0
            current_sum1 = 0
            for i in range(self.num_sensors_outside, self.num_sensors):
                current_sum0 += self.l_finger_tip[i]
                
            for i in range(self.num_sensors_outside, self.num_sensors):
                current_sum1 += self.r_finger_tip[i]
 
        expected_value = self.force*self.force*self.force*self.x3 + self.force*self.force*self.x2 + self.force*self.x1 + self.x0

        self._forces.append(self.force)
        self._expected.append(expected_value)
        self._tip0.append(current_sum0 - self.starting_sum0)
        self._tip1.append(current_sum1 - self.starting_sum1)

    ##\brief Write fit equation as HTML table
    def _write_equation(self):
        html = '<p align=center><b>Gripper Equation</b></p><br>\n'
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Term</b></td><td><b>Value</b></td></tr>\n'
        html += '<tr><td>Force^3</td><td>%f</td></tr>\n' % self.x3
        html += '<tr><td>Force^2</td><td>%f</td></tr>\n' % self.x2
        html += '<tr><td>Force</td><td>%f</td></tr>\n' % self.x1
        html += '<tr><td>Constant</td><td>%s</td></tr>\n' % self.x0
        html += '</table>\n'

        return html

    ##\brief Write test params as HTML table
    def _write_params(self):
        html = '<p align=center><b>Test Parameters</b></p><br>\n'
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Parameter</b></td><td><b>Value</b></td></tr>\n'
        html += '<tr><td>Pressure Topic</td><td>%s</td></tr>\n' % self._pressure_topic
        html += '<tr><td>Num Sensors</td><td>%d</td></tr>\n' % self.num_sensors
        html += '<tr><td>Num Sensors Outside</td><td>%d</td></tr>\n' % self.num_sensors_outside
        html += '<tr><td>Num. Increments</td><td>%d</td></tr>\n' % self.num_increments
        html += '<tr><td>Max Force</td><td>%f</td></tr>\n' % (self.force - self.increment)
        html += '<tr><td>Force Increment</td><td>%f</td></tr>\n' % self.increment
        html += '</table>\n'

        return html

    ##\brief Write tolerances as HTML table
    def _write_tols(self):
        html = '<p align=center><b>Error Tolerances</b></p>\n'
        html += '<p>All tolerances given as a percent of the maximum observed value.</p>\n'
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Parameter</b></td><td><b>Questionable</b></td><td><b>Failure</b></td></tr>\n'
        html += '<tr><td>Max Error</td><td>%.1f</td><td>%.1f</td></tr>\n' % (self._tol_max_quest, self._tol_max_fail)
        html += '<tr><td>Average Error</td><td>%.1f</td><td>%.1f</td></tr>\n' % (self._tol_avg_quest, self._tol_avg_fail)
        html += '<tr><td>Max Diff.</td><td>%.1f</td><td>%.1f</td></tr>\n' % (self._diff_max_quest, self._diff_max_fail)
        html += '<tr><td>Avg. Diff.</td><td>%.1f</td><td>%.1f</td></tr>\n' % (self._diff_avg_quest, self._diff_avg_fail)
        html += '<tr><td>Avg. Abs. Diff.</td><td>%.1f</td><td>%.1f</td></tr>\n' % (self._diff_avg_abs_quest, self._diff_avg_abs_fail)
        
        html += '</table>\n'

        return html

    ##\brief Write pressure at each datapoint for both tips
    def _write_data(self):
        html = '<p align=center><b>Fingertip Pressure Data</b></p>\n'
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Force</b></td><td><b>L tip</b></td><td><b>R tip</b></td><td><b>Expected</b></td></tr>\n'
        for i in range(0, len(self._forces)):
            html += '<tr><td>%.2f</td><td>%.2f</td><td>%.2f</td><td>%.2f</td></tr>\n' % (self._forces[i], self._tip0[i], self._tip1[i], self._expected[i])
        html += '</table>\n'

        # Write CSV log to allow oocalc graph
        if False:
            import csv
            csv_path = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'results/temp/%s_fingers.csv' % rospy.get_param('/qual_item/serial', 'finger'))
            log_csv = csv.writer(open(csv_path, 'wb'))
            log_csv.writerow(['Force', 'L tip', 'R tip', 'Expected'])
            for i in range(0, len(self._forces)):
                log_csv.writerow([self._forces[i], self._tip0[i], self._tip1[i], self._expected[i]])
            
            html += '<p>Wrote CSV of results to: %s.</p>\n' % csv_path

        return html

    ##\brief At every point, check differences bwt tips
    def _check_diff(self):
        diff = numpy.array(self._tip0) - numpy.array(self._tip1)
        avg_vals = 0.5 * (numpy.array(self._tip0) + numpy.array(self._tip1))
        
        max_val = max(avg_vals)
        max_diff = max(abs(diff)) / max_val * 100
        avg_diff = abs(numpy.average(diff) / max_val * 100)
        avg_abs_diff = numpy.average(abs(diff)) / max_val * 100

        max_diff_lvl = 0
        avg_diff_lvl = 0
        avg_abs_diff_lvl = 0

        if max_diff > self._diff_max_quest:
            max_diff_lvl = 1
        if max_diff > self._diff_max_fail:
            max_diff_lvl = 2

        if avg_diff > self._diff_avg_quest:
            avg_diff_lvl = 1
        if avg_diff > self._diff_avg_fail:
            avg_diff_lvl = 2

        if avg_abs_diff > self._diff_avg_abs_quest:
            avg_abs_diff_lvl = 1
        if avg_abs_diff > self._diff_avg_abs_fail:
            avg_abs_diff_lvl = 2
        
        stat_lvl = max(max_diff_lvl, avg_diff_lvl, avg_abs_diff_lvl)
        
        html = '<p align=center><b>Fingertip Differences</b></p>\n'
        html += '<p>Differences given in percent of maximum observed value. Status: %s</p>\n' % lvl_dict[stat_lvl]
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Parameter</b></td><td><b>Value</b></td><td><b>Status</b></td><td><b>Questionable Pt.</b></td><td><b>Failure Pt.</b></td></tr>\n'
        html += '<tr><td>Max Diff.</td><td>%.1f</td><td>%s</td><td>%.1f</td><td>%.1f</td></tr>\n' % (max_diff, lvl_dict[max_diff_lvl], self._diff_max_quest, self._diff_max_fail)
        html += '<tr><td>Avg. Diff.</td><td>%.1f</td><td>%s</td><td>%.1f</td><td>%.1f</td></tr>\n' % (avg_diff, lvl_dict[avg_diff_lvl], self._diff_avg_quest, self._diff_avg_fail)
        html += '<tr><td>Avg. Abs. Diff.</td><td>%.1f</td><td>%s</td><td>%.1f</td><td>%.1f</td></tr>\n' % (avg_abs_diff, lvl_dict[avg_abs_diff_lvl], self._diff_avg_abs_quest, self._diff_avg_abs_fail)

        html += '</table>\n'
        
        return html, stat_lvl

    ##\brief At every point, check abs dev from expected
    def _check_tol(self):
        true_vals = numpy.array(self._expected)
        tip0_vals = numpy.array(self._tip0)
        tip1_vals = numpy.array(self._tip1)
        
        max_val = 0.5 * max(tip0_vals + tip1_vals)

        # Maximum error 
        max_err0 = max(abs(tip0_vals - true_vals)) / max_val * 100
        max_err1 = max(abs(tip1_vals - true_vals)) / max_val * 100
        
        err0_lvl = 0
        err1_lvl = 0

        max_val_warn = self._tol_max_quest
        max_val_err  = self._tol_max_fail

        if max_err0 > self._tol_max_quest:
            err0_lvl = 1
        if max_err0 > self._tol_max_fail:
            err0_lvl = 2

        if max_err1 > self._tol_max_quest:
            err1_lvl = 1
        if max_err1 > self._tol_max_fail:
            err1_lvl = 2

        # Average error
        avg_err0 = abs(numpy.average(tip0_vals - true_vals) / max_val * 100)
        avg_err1 = abs(numpy.average(tip1_vals - true_vals) / max_val * 100)

        avg_err0_lvl = 0
        avg_err1_lvl = 0
        
        if avg_err0 > self._tol_avg_quest:
            avg_err0_lvl = 1
        if avg_err0 > self._tol_avg_fail:
            avg_err0_lvl = 2

        if avg_err1 > self._tol_avg_quest:
            avg_err1_lvl = 1
        if avg_err1 > self._tol_avg_fail:
            avg_err1_lvl = 2
        
        stat_lvl = max(err0_lvl, err1_lvl, avg_err0_lvl, avg_err1_lvl)

        html = '<p align=center><b>Fingertip Error</b></p>\n'
        html += '<p>Error from expected value given in percent of maximum observed value. Status: %s</p>\n' % lvl_dict[stat_lvl]
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Parameter</b></td><td><b>Value</b></td><td><b>Status</b></td><td><b>Questionable Pt.</b></td><td><b>Failure Pt.</b></td></tr>\n'
        html += '<tr><td>L tip Maximum Error</td><td>%.1f</td><td>%s</td><td>%.1f</td><td>%.1f</td></tr>\n' % (max_err0, lvl_dict[err0_lvl], self._tol_max_quest, self._tol_max_fail)
        html += '<tr><td>R tip Maximum Error</td><td>%.1f</td><td>%s</td><td>%.1f</td><td>%.1f</td></tr>\n' % (max_err1, lvl_dict[err1_lvl], self._tol_max_quest, self._tol_max_fail)
        html += '<tr><td>L tip Average Error</td><td>%.1f</td><td>%s</td><td>%.1f</td><td>%.1f</td></tr>\n' % (avg_err0, lvl_dict[avg_err0_lvl], self._tol_avg_quest, self._tol_avg_fail)
        html += '<tr><td>R tip Average Error</td><td>%.1f</td><td>%s</td><td>%.1f</td><td>%.1f</td></tr>\n' % (avg_err1, lvl_dict[avg_err1_lvl], self._tol_avg_quest, self._tol_avg_fail)
        html += '</table>\n'

        return html, stat_lvl

    ##\brief Check pass/fail, write HTML result of all data, parameters
    def process_results(self):
        tol_html, tol_stat = self._check_tol()
        diff_html, diff_stat = self._check_diff()

        html = '<img src="IMG_PATH/finger_tip.png", width=640, height=480 />\n'
        html += '<hr size="2">\n' + tol_html
        html += '<hr size="2">\n' + diff_html
        html += '<hr size="2">\n' + self._write_equation()
        html += '<hr size="2">\n' + self._write_params()
        html += '<hr size="2">\n' + self._write_tols()
        html += '<hr size="2">\n' + self._write_data()
        html += '<hr size="2">\n' + self._connected_data
                
        result_val = TestResultRequest.RESULT_PASS
        if max(tol_stat, diff_stat) == 1:
            result_val = TestResultRequest.RESULT_HUMAN_REQUIRED
        elif max(tol_stat, diff_stat) > 1:
            result_val = TestResultRequest.RESULT_FAIL

        r = TestResultRequest()
        r.text_summary = 'Fingertip test. Error tolerance: %s. Tip differences: %s' % (lvl_dict[tol_stat], lvl_dict[diff_stat])
        r.html_result = html
        r.plots = []
        r.result = result_val

        fig = plot.figure(1)
        plot.xlabel('Effort')
        plot.ylabel('Total pressure')
        plot.plot(numpy.array(self._forces), numpy.array(self._expected), label='Expected')
        plot.plot(numpy.array(self._forces), numpy.array(self._tip0), label='L tip')
        plot.plot(numpy.array(self._forces), numpy.array(self._tip1), label='R tip')
        fig.text(.3, .95, 'Fingertip Pressure v. Effort')
        plot.legend(shadow=True)
        
        # Store figure in Plot message
        stream = StringIO()
        plot.savefig(stream, format="png")
        image = stream.getvalue()
        p = Plot()
        r.plots.append(p)
        p.title = 'finger_tip'
        p.image = image
        p.image_format = 'png'

        self.send_results(r)
        

if __name__ == '__main__':
    qual = FingertipQualification()
    try:
        qual.open_gripper()
        if not qual.check_connected():
            # We've failed and sent it in, now wait to be terminated
            rospy.spin()
            import sys
            sys.exit()

        # Only check connection, we've sent results already
        if qual.check_connect_only:
            rospy.spin()
            import sys
            sys.exit()

        for j in range(0, qual.num_increments):
            if rospy.is_shutdown():
                break

            qual.record_zero_val()
            qual.close_gripper()
            qual.record_increase()
            qual.open_gripper()
            qual.increment_value()

        qual.process_results()
        rospy.spin()
    except KeyboardInterrupt:
        pass
    except Exception, e:
        import traceback
        print 'Caught exception in fingertip_qualification.\n%s' % traceback.format_exc()
        rospy.logerr('Fingertip qualification exception.\n%s' % traceback.format_exc())
        qual.test_failed_service_call(traceback.format_exc())

    print 'Quitting fingertip qualification'
