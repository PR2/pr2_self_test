#!/usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
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
##\brief Fingertip qualification of gripper

import roslib
roslib.load_manifest('qualification')

import numpy
import matplotlib
import matplotlib.pyplot as plot
from StringIO import StringIO
from time import sleep
import traceback

import rospy
from qualification.srv import TestResult, TestResultRequest
from qualification.msg import Plot
from std_msgs.msg import Float64
from pr2_msgs.msg import PressureState

import threading

class FingertipQualification:
    def __init__(self):
        self._mutex = threading.Lock()
        self.set_cmd = 0.0
        

        self.pub = rospy.Publisher('r_gripper_effort_controller/command', Float64)

        self._pressure_topic = 'pressure/r_gripper_motor'
        rospy.Subscriber(self._pressure_topic, PressureState, self.pressure_callback)
        rospy.init_node('fingertip_qualification')
        self.initial = rospy.get_param('~grasp_force_initial', -25.0)
        self.increment = rospy.get_param('~grasp_force_increment', -25.0) 
        self.num_increments = rospy.get_param('~grasp_increments', 4) 
        self.x0 = rospy.get_param('~x^0', 701.63)
        self.x1 = rospy.get_param('~x^1', -896.61)
        self.x2 = rospy.get_param('~x^2', 0)
        self.x3 = rospy.get_param('~x^3', 0)
        self.tol_percent = rospy.get_param('~tolerance_percent', 10.0)
        self.fingertip_refresh = rospy.get_param('~fingertip_refresh_hz', 25)
        self.num_sensors = rospy.get_param('~num_sensors', 22)
        self.num_sensors_outside = rospy.get_param('~num_sensors_outside', 7)
        self.force = self.initial

        self.data_sent = False
        self.result_service = rospy.ServiceProxy('/test_result', TestResult)

        self._connection_data = ''

        # Data
        self._forces = []
        self._expected = []
        self._tip0 = []
        self._tip1 = []

    def pressure_callback(self,data):
        self._mutex.acquire()
        self.data0 = data.data0
        self.data1 = data.data1
        self._mutex.release()

    def test_failed_service_call(self, except_str = ''):
        rospy.logerr(except_str)
        r = TestResultRequest()
        r.html_result = except_str
        r.text_summary = 'Caught exception, automated test failure.'
        r.plots = []
        r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
        self.send_results(r)

    def send_results(self, test_result):
        if not self.data_sent:
            rospy.wait_for_service('test_result', 30)
            self.result_service.call(test_result)
            self.data_sent = True

    def open_gripper(self):
        self.set_cmd = 50
        self.pub.publish(self.set_cmd)
        sleep(1)
        
    def close_gripper(self):
        self.set_cmd = self.force
        self.pub.publish(self.set_cmd)
        sleep(4)
    
    def check_connected(self):
        sleep(1/self.fingertip_refresh*2)
        self._mutex.acquire()

        ok = True

        # TODO::check if data exists!!!!!
        if self.data0 is None or self.data1 is None:
            r = TestResultRequest()
            r.text_summary = 'No gripper tip data.'
            
            r.html_result = '<p>No gripper tip data. Check connections. Tip 0: %s, tip 1: %s.</p>\n' % (str(self.data0), str(self.data1))
            r.html_result +=  '<hr size="2">\n' + self._write_equation()
            r.html_result += '<hr size="2">\n' + self._write_params()
            r.result = TestResultRequest.RESULT_FAIL
            self.send_results(r)

        if len(self.data0) != self.num_sensors or len(self.data1) != self.num_sensors:
            r = TestResultRequest()
            r.text_summary = 'Incorrect number of sensors. Expected: %d' % self.num_sensors
            
            r.html_result = '<p>Incorrect number of sensors. Expected: %d. Tip 0: %d, tip 1: %d.</p>\n' % (self.num_sensors, len(self.data0), len(self.data1))
            r.html_result +=  '<hr size="2">\n' + self._write_equation()
            r.html_result += '<hr size="2">\n' + self._write_params()
            r.result = TestResultRequest.RESULT_FAIL
            self.send_results(r)

        connect_table = '<table border="1" cellpadding="2" cellspacing="0">\n'
        connect_table += '<tr><td><b>Sensor</b></td><td><b>Tip 0</b></td><td><b>Tip 1</b></td></tr>\n'
        for i in range(0, self.num_sensors):
            tip0 = 'OK'
            tip1 = 'OK'
            if self.data0[i] == 0:
                ok = False
                tip0 = 'No data'               

            if self.data1[i] == 0:
                ok = False
                tip1 = 'No data'

            connect_table += '<tr><td>%d</td><td>%s</td><td>%s</td></tr>\n' % (i, tip0, tip1)
        connect_table += '</table>\n'

        self._mutex.release()
        
        connect_str = 'OK'
        if not ok:
            connect_str = 'Error'
        self._connected_data = '<p>Tip connections: %s.</p>\n' % connect_str
        self._connected_data += connect_table
        
        if not ok:
            r = TestResultRequest()
            r.text_summary = 'Not connected'
            r.html_result = self._connected_data 
            r.html_result +=  '<hr size="2">\n' + self._write_equation()
            r.html_result += '<hr size="2">\n' + self._write_params()
            r.result = TestResultRequest.RESULT_HUMAN_REQUIRED # FAIL
            self.send_results(r)
                
        return ok

    def increment_value(self):
        self.force += self.increment
    
    def record_zero_val(self):
        self._mutex.acquire()
        self.starting_sum0 = 0
        self.starting_sum1 = 0
        for i in range(self.num_sensors_outside, self.num_sensors):
            self.starting_sum0 += self.data0[i]
            self.starting_sum1 += self.data1[i]
        self._mutex.release()
      
    def record_increase(self):
        self._mutex.acquire()
        current_sum0 = 0
        for i in range(self.num_sensors_outside, self.num_sensors):
            current_sum0 += self.data0[i]
        current_sum1 = 0
        for i in range(self.num_sensors_outside, self.num_sensors):
            current_sum1 += self.data1[i]
        #for current in self.data1:
        #    current_sum1 += current
        self._mutex.release()
 
        expected_value = self.force*self.force*self.force*self.x3 + self.force*self.force*self.x2 + self.force*self.x1 + self.x0

        self._forces.append(self.force)
        self._expected.append(expected_value)
        self._tip0.append(current_sum0 - self.starting_sum0)
        self._tip1.append(current_sum1 - self.starting_sum1)

    def _write_equation(self):
        html = '<p>Equation for total gripper force</p>\n'
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Term</b></td><td><b>Value</b></td></tr>\n'
        html += '<tr><td>Force^3</td><td>%f</td></tr>\n' % self.x3
        html += '<tr><td>Force^2</td><td>%f</td></tr>\n' % self.x2
        html += '<tr><td>Force</td><td>%f</td></tr>\n' % self.x1
        html += '<tr><td>Constant</td><td>%s</td></tr>\n' % self.x0
        html += '</table>\n'

        return html

    def _write_params(self):
        html = '<p>Test parameters:</p>\n'
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Parameter</b></td><td><b>Value</b></td></tr>\n'
        html += '<tr><td>Pressure Topic</td><td>%s</td></tr>\n' % self._pressure_topic
        html += '<tr><td>Num Sensors</td><td>%d</td></tr>\n' % self.num_sensors
        html += '<tr><td>Num Sensors Outside</td><td>%d</td></tr>\n' % self.num_sensors_outside
        html += '<tr><td>Num. Increments</td><td>%d</td></tr>\n' % self.num_increments
        html += '<tr><td>Max Force</td><td>%f</td></tr>\n' % self.force
        html += '<tr><td>Tolerance</td><td>%f</td></tr>\n' % self.tol_percent
        html += '<tr><td>Force Increment</td><td>%f</td></tr>\n' % self.increment
        html += '</table>\n'

        return html

    def _write_data(self):
        html = '<p>Fingertip Pressure Data</p>\n'
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Force</b></td><td><b>Tip 0</b></td><td><b>Tip 0</b></td><td><b>Expected</b></td></tr>\n'
        for i in range(0, len(self._forces)):
            html += '<tr><td>%.2f</td><td>%.2f</td><td>%.2f</td><td>%.2f</td></tr>\n' % (self._forces[i], self._tip0[i], self._tip1[i], self._expected[i])
        html += '</table>\n'

        return html

    def process_results(self):
        html = '<p>Finger tip data.</p>\n'
        html += '<img src="IMG_PATH/finger_tip.png", width=640, height=480 />\n'
        html += '<hr size="2">\n' + self._write_equation()
        html += '<hr size="2">\n' + self._write_params()
        html += '<hr size="2">\n' + self._write_data()
        html += '<hr size="2">\n' + self._connected_data
                
        r = TestResultRequest()
        r.text_summary = 'Finger tip test'
        r.html_result = html
        r.plots = []
        r.result = TestResultRequest.RESULT_HUMAN_REQUIRED # PASS or FAIL

        fig = plot.figure(1)
        plot.ylabel('Effort')
        plot.xlabel('Total pressure')
        plot.plot(numpy.array(self._forces), numpy.array(self._expected), label='Expected')
        plot.plot(numpy.array(self._forces), numpy.array(self._tip0), label='Tip 0')
        plot.plot(numpy.array(self._forces), numpy.array(self._tip1), label='Tip 1')
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
            sleep(1)

        for j in range(0,qual.num_increments):
            if rospy.is_shutdown():
                break

            qual.record_zero_val()
            qual.close_gripper()
            qual.record_increase()
            qual.open_gripper()
            qual.increment_value()
        qual.process_results()
    except Exception, e:
        print 'Caught exception in fingertip_qualification.\n%s' % traceback.format_exc()
        rospy.logerr('Fingertip qualification exception.\n%s' % traceback.format_exc())
        qual.test_failed_service_call(traceback.format_exc())

    print 'Quitting fingertip qualification'
