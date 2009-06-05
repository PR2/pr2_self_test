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

# Author: Kevin Watts

import roslib
roslib.load_manifest('qualification')

import rospy
import sys, os, string
from time import sleep

import traceback

import numpy
import math
import matplotlib
import matplotlib.pyplot as plot
from StringIO import StringIO

import roslaunch

from qualification.msg import Plot
from qualification.srv import *

from joint_qualification_controllers.srv import * 

class CounterBalanceAnalysis:
    def __init__(self):
        rospy.init_node('cb_analysis')
        rospy.logerr('Initing node')
        self.data_topic = rospy.Service('hold_set_data', HoldSetData, self.data_callback)
        self.result_service = rospy.ServiceProxy('test_result', TestResult)
        
        self._sent_results = False

        self._hold_data_srvs = []

        self.r = TestResultRequest()
        self.r.html_result = '<p>Test Failed.</p>'
        self.r.text_summary = 'Failure.'
        self.r.plots = []
        self.r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
        
        rospy.spin()                    
    
      
    def test_failed_service_call(self, except_str = ''):
        rospy.logerr(except_str)
        self.r.html_result = except_str
        self.r.text_summary = 'Caught exception, automated test failure.'
        self.r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
        if not self._sent_results:
            self.result_service.call(self.r)
        self._sent_results = True


    def data_callback(self, srv):
        self.process_results(srv)
        return HoldSetDataResponse()

    def process_results(self, srv):
        try:
            html = '<p>Counterbalance Analysis</p>\n'

            # Add counter plot of points
            html += self.contour_plot(srv)
            
            html += '<p>Test Data</p>\n'
            html += '<table border="1" cellpadding="2" cellspacing="0">\n'
            html += '<tr><td><b>Joint</b></td><td><b>Dither Amplitude</b></td></tr>\n'
            for i in range(0, len(srv.joint_names)):
                html += '<tr><td>%s</td><td>%s</td></tr>\n' % (srv.joint_names[i], srv.dither_amps[i])
            html += '</table>\n<hr size="2">\n'
        
            # Summarize all positions
            html += '<p>Summary of all hold positions.</p>\n'
            html += '<table  border="1" cellpadding="2" cellspacing="0">\n'
            html += '<tr>'
            for joint in srv.joint_names:
                html += '<td><b>%s</b></td><td><b>Goal</b></td><td><b>Position</b></td><td><b>Vel. SD</b></td><td><b>Effort</b></td><td><b>Effort SD</b></td>' % joint
                html += '</tr>\n'
            for hold in srv.hold_data:
                html += self.hold_summary(hold, srv.joint_names)
            html += '</table><hr size="2" > \n'

            self.r.html_result = html
            self.r.text_summary = 'Analysis performed, got CB data for: %s' % (string.join(srv.joint_names, ', '))
            self.r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
            if not self._sent_results:
                self.result_service.call(self.r)
                self._sent_results = True
        except Exception, e:
            self.test_failed_service_call(traceback.format_exc());


    def hold_summary(self, hold, joints):
        html = '<tr>'

        for i in range(0, len(hold.joint_data)):
            jnt_data = hold.joint_data[i]

            desire = jnt_data.desired
            pos_avg = float(numpy.average(numpy.array(jnt_data.position)))
            vel_sd = float(numpy.std(numpy.array(jnt_data.velocity)))
            effort_avg = float(numpy.average(numpy.array(jnt_data.effort)))
            effort_sd = float(numpy.std(numpy.array(jnt_data.effort)))

            html += '<td>%s</td>' % joints[i]
            html += '<td>%.2f</td><td>%.2f</td><td>%.2f</td><td>%.2f</td><td>%.2f</td>' % (desire, pos_avg, vel_sd, effort_avg, effort_sd)
        html += '</tr>\n'

        return html
    

    def contour_plot(self, srv):
        # Find shoulder lift, elbow flex joint indices
        idx_flex = srv.joint_names.index('r_elbow_flex_joint')
        idx_lift = srv.joint_names.index('r_shoulder_lift_joint')

        flex_pos = []
        flex_tau = []
        lift_pos = []
        lift_tau = []

        # Two passes, one with lift = 1, other with flex = 0
        flex_pass = []
        lift_pass = []
        lift_pass_eff = []
        flex_pass_eff = []


        for hold in srv.hold_data:
            lift_data = hold.joint_data[idx_lift]

            lift_pos_avg = numpy.average(numpy.array(lift_data.position))
            lift_eff_avg = numpy.average(numpy.array(lift_data.effort))

            lift_pos.append(lift_pos_avg)
            lift_tau.append(lift_eff_avg)

            flex_data = hold.joint_data[idx_flex]

            flex_pos_avg = numpy.average(numpy.array(flex_data.position))
            flex_eff_avg = numpy.average(numpy.array(flex_data.effort))

            flex_pos.append(flex_pos_avg)
            flex_tau.append(flex_eff_avg)

            if (abs(lift_data.desired) - 1.00) < 0.01:
                flex_pass.append(lift_pos_avg)
                flex_pass_eff.append(lift_eff_avg)
            
            if abs(flex_data.desired) < 0.01:
                lift_pass.append(lift_pos_avg)
                lift_pass_eff.append(lift_eff_avg)


        fig = plot.figure(1)

        #C_flex = plot.contour(numpy.array(lift_pos), numpy.array(flex_pos), numpy.array(flex_tau))
        plot.plot(numpy.array(flex_pass), numpy.array(flex_pass_eff))
        plot.title('Elbow Flex Effort, Lift = 1.00')
        plot.axes()
        plot.xlabel('Flex Position')
        plot.ylabel('Torque')

        stream = StringIO()
        plot.savefig(stream, format = 'png')
        image = stream.getvalue()
        p = qualification.msg.Plot()
        self.r.plots.append(p)
        p.title = 'flex_pass'
        p.image = image
        p.image_format = 'png'
        
        plot.close()

        fig = plot.figure(2)
        plot.plot(numpy.array(lift_pass), numpy.array(lift_pass_eff))
        #C_lift = plot.contour(numpy.array(lift_pos), numpy.array(flex_pos), numpy.array(lift_tau))
        plot.title('Shoulder Lift Effort, Flex = 0.0')
        plot.axes()
        plot.xlabel('Lift Position')
        plot.ylabel('Effort')

        stream = StringIO()
        plot.savefig(stream, format = 'png')
        image = stream.getvalue()
        p = qualification.msg.Plot()
        self.r.plots.append(p)
        p.title = 'lift_pass'
        p.image = image
        p.image_format = 'png'
 
        plot.close()
        
        html = '<p><b>Lift Pass Torque Data.</b></p>\n'
        html += '<img src="IMG_PATH/lift_pass.png" width="640" height="480" /><br><br>\n'

        html += '<p><b>Flex Pass Torque Data.</b></p>\n'
        html += '<img src="IMG_PATH/flex_pass.png" width="640" height="480" /><br><br>\n'


        return html
        


if __name__ == '__main__':
    try:
        print 'Making app'
        app = CounterBalanceAnalysis()
        rospy.spin()
    except Exception, e:
        print 'Caught Exception in CB application'
        traceback.print_exc()
        result_service = rospy.ServiceProxy('test_result', TestResult)        
        
        r = TestResultRequest()
        r.html_result = traceback.format_exc()
        r.text_summary = 'Caught exception, automated test failure.'
        r.plots = []
        r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
        result_service.call(r)
