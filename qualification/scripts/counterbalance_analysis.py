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
##\brief Analysis results from counterbalance test controller

import roslib
roslib.load_manifest('qualification')

import rospy

import numpy
import math
import matplotlib
import matplotlib.pyplot as plot
from StringIO import StringIO

from qualification.msg import Plot, TestValue, TestParam
from qualification.srv import *

from std_msgs.msg import Bool

from joint_qualification_controllers.msg import CounterbalanceTestData

ok_dict = { False: 'FAIL', True: 'OK' }

class JointPositionAnalysisData:
    def __init__(self, msg):
        self.time     = numpy.array(msg.time)
        self.position = numpy.array(msg.position)
        self.velocity = numpy.array(msg.velocity)
        self.effort   = numpy.array(msg.effort)

        self.position_avg = numpy.average(self.position)
        self.position_sd  = numpy.std(self.position)
        self.effort_avg   = numpy.average(self.effort)
        self.effort_sd    = numpy.std(self.effort)

class CBPositionAnalysisData:
    def __init__(self, msg):
        self.flex_position = msg.flex_position
        self.lift_hold = JointPositionAnalysisData(msg.lift_hold)
        self.flex_hold = JointPositionAnalysisData(msg.flex_hold)

class CBRunAnalysisData:
    def __init__(self, msg):
        self.lift_position = msg.lift_position
        self.flex_data = []
        for fd in msg.flex_data:
            self.flex_data.append(CBPositionAnalysisData(fd))

class CounterbalanceAnalysisData:
    def __init__(self, srv):
        self.lift_data = []
        for ld in srv.lift_data:
            self.lift_data.append(CBRunAnalysisData(ld))

class CounterbalanceAnalysisParams:
    def __init__(self, srv):
        self.lift_dither = srv.lift_amplitude
        self.flex_dither = srv.flex_amplitude
        
        self.lift_joint = srv.lift_joint
        self.flex_joint = srv.flex_joint
        
        self.timeout_hit = srv.timeout_hit
        self.flex_test = srv.flex_test

        self.lift_mse     = srv.arg_value[9]
        self.lift_avg_abs = srv.arg_value[10]
        self.lift_avg_eff = srv.arg_value[11]
        self.flex_mse     = srv.arg_value[12]
        self.flex_avg_abs = srv.arg_value[13]
        self.flex_avg_eff = srv.arg_value[14]

        self.lift_p       = srv.arg_value[15]
        self.lift_i       = srv.arg_value[16]
        self.lift_d       = srv.arg_value[17]
        self.lift_i_clamp = srv.arg_value[18]

        self.flex_p       = srv.arg_value[19]
        self.flex_i       = srv.arg_value[20]
        self.flex_d       = srv.arg_value[21]
        self.flex_i_clamp = srv.arg_value[22]
        
        self.num_flexes   = len(srv.lift_data[0].flex_data)
        self.num_lifts    = len(srv.lift_data[0].flex_data)
        ##\todo Need to count len flexes, lifts

    def get_test_params(self):
        params = []
        params.append(TestParam(key='Lift Dither', value=str(self.lift_dither)))
        params.append(TestParam(key='Flex Dither', value=str(self.flex_dither)))
        params.append(TestParam(key='Lift Joint', value=self.lift_joint))
        params.append(TestParam(key='Flex Joint', value=self.flex_joint))
        params.append(TestParam(key='Timeout Hit', value=ok_dict[not self.timeout_hit]))
        params.append(TestParam(key='Flex Tested', value=str(self.flex_test)))

        params.append(TestParam(key='Lift MSE', value=str(self.lift_mse)))
        params.append(TestParam(key='Lift Avg Abs', value=str(self.lift_avg_abs)))
        params.append(TestParam(key='Lift Avg Effort', value=str(self.lift_avg_eff)))
        params.append(TestParam(key='Flex MSE', value=str(self.flex_mse)))
        params.append(TestParam(key='Flex Avg Abs', value=str(self.flex_avg_abs)))
        params.append(TestParam(key='Flex Avg Effort', value=str(self.flex_avg_eff)))

        params.append(TestParam(key='Lift P Gain', value=str(self.lift_p)))
        params.append(TestParam(key='Lift I Gain', value=str(self.lift_i)))
        params.append(TestParam(key='Lift D Gain', value=str(self.lift_d)))
        params.append(TestParam(key='Lift I Clamp', value=str(self.lift_i_clamp)))

        params.append(TestParam(key='Flex P Gain', value=str(self.flex_p)))
        params.append(TestParam(key='Flex I Gain', value=str(self.flex_i)))
        params.append(TestParam(key='Flex D Gain', value=str(self.flex_d)))
        params.append(TestParam(key='Flex I Clamp', value=str(self.flex_i_clamp)))

        return params

class CounterbalanceAnalysisResult:
    def __init__(self):
        self.html = ''
        self.summary = ''
        self.result = False
        self.values = []
        

def get_efforts(data, lift_calc):
    avg_effort_list = []
    for ld in data.lift_data:
        for fd in ld.flex_data:
            if lift_calc:
                avg_effort_list.append(fd.lift_hold.effort_avg)
            else:
                avg_effort_list.append(fd.flex_hold.effort_avg)

    return avg_effort_list

def get_mean_sq_effort(avg_effort_array):
    sq_array = avg_effort_array * avg_effort_array
    return numpy.average(sq_array)

def get_mean_abs_effort(avg_effort_array):
    abs_array = abs(avg_effort_array)
    return numpy.average(abs_array)

def get_mean_effort(avg_effort_array):
    return numpy.average(avg_effort_array)

##\brief Returns a list of lift positions, efforts for a given flex position (vary by lift)
def get_const_flex_effort(data, flex_index = 0, lift_calc = True):
    if flex_index > len(data.lift_data[0].flex_data):
        raise "Flex index out of range"
    
    effort_list = []
    lift_list = []
    for ld in data.lift_data:
        fd = ld.flex_data[flex_index]
        lift_list.append(ld.lift_position)
        if lift_calc:
            effort_list.append(fd.lift_hold.effort_avg)
        else:
            effort_list.append(fd.flex_hold.effort_avg)

    return lift_list, effort_list
    
def get_const_lift_effort(data, lift_index = 0, lift_calc = True):
    if lift_index > len(data.lift_data):
        raise "Lift index out of range"
    
    effort_list = []
    flex_list = []
    ld = data.lift_data[lift_index]
    for fd in ld.flex_data:
        flex_list.append(fd.flex_position)
        if lift_calc:
            effort_list.append(fd.lift_hold.effort_avg)
        else:
            effort_list.append(fd.flex_hold.effort_avg)

    return flex_list, effort_list

def plot_efforts_by_lift_position(params, data, flex_index = 0, lift_calc = True):
    lift_position, effort = get_const_flex_effort(data, flex_index, lift_calc)
    
    flex_position = data.lift_data[0].flex_data[flex_index].flex_position

    plot.plot(numpy.array(lift_position), numpy.array(effort))
    if lift_calc:
        plot.title('Shoulder Lift Effort at Flex Position %.2f' % (flex_position))
    else:
        plot.title('Shoulder Flex Effort at Flex Position %.2f' % (flex_position))
    plot.axes()
    plot.xlabel('Lift Position')
    plot.ylabel('Effort')
    plot.axhline(y = 0, color = 'r', label='_nolegend_')

    stream = StringIO()
    plot.savefig(stream, format = 'png')
    image = stream.getvalue()
    p = Plot()
    if lift_calc:
        p.title = 'lift_effort_const_flex_%d' % flex_index
    else:
        p.title = 'flex_effort_const_flex_%d' % flex_index
    p.image = image
    p.image_format = 'png'
    
    plot.close()

    return p

def get_flex_positions(data):
    flex_list = []
    for fd in data.lift_data[0].flex_data:
        flex_list.append(fd.flex_position)

    return flex_list

def plot_effort_contour(params, data, lift_calc = True):
    effort_list = []
    for i in range(0, params.num_flexes):
        lifts, efforts = get_const_lift_effort(data, i, lift_calc)
        effort_list.append(efforts)
    flexes = get_flex_positions(data)
    flex_grid, lift_grid = numpy.meshgrid(numpy.array(flexes), numpy.array(lifts))
    effort_grid = numpy.array(effort_list)

    CS = plot.contour(flex_grid, lift_grid, effort_grid)
    plot.clabel(CS, inline=0, fontsize=10)
    
    plot.xlabel('Flex')
    plot.ylabel('Lift')
        
    stream = StringIO()
    plot.savefig(stream, format = 'png')
    image = stream.getvalue()
    p = Plot()
    if lift_calc:
        p.title = 'lift_effort_contour'
    else:
        p.title = 'flex_effort_contour'
    p.image = image
    p.image_format = 'png'
    
    plot.close()

    return p

def analyze_lift_efforts(params, data):
    result = CounterbalanceAnalysisResult()
    
    avg_efforts = numpy.array(get_efforts(data, True))
    mse = get_mean_sq_effort(avg_efforts)
    avg_abs = get_mean_abs_effort(avg_efforts)
    avg_eff = get_mean_effort(avg_efforts)

    mse_ok = mse < params.lift_mse
    avg_abs_ok = avg_abs < params.lift_avg_abs
    avg_eff_ok = abs(avg_eff) < abs(params.lift_avg_eff)

    if mse_ok and avg_abs_ok:
        result.summary = 'Lift efforts OK'
    else:
        result.summary = 'Lift MSE/Avg. Absolute effort too high'

    html = ['<p>%s</p>' % result.summary]
    
    html.append('<table border="1" cellpadding="2" cellspacing="0">')
    html.append('<tr><td><b>Parameter</b></td><td><b>Value</b></td><td><b>Maximum</b></td><td><b>Status</b></td></tr>')
    html.append('<tr><td><b>Mean Sq. Effort</b></td><td>%.2f</td><td>%.2f</td><td>%s</td></tr>' % (mse, params.lift_mse, ok_dict[mse_ok]))
    html.append('<tr><td><b>Average Abs. Effort</b></td><td>%.2f</td><td>%.2f</td><td>%s</td></tr>' % (avg_abs, params.lift_avg_abs, ok_dict[avg_abs_ok]))
    html.append('<tr><td><b>Average Effort</b></td><td>%.2f</td><td>%.2f</td><td>%s</td></tr>' % (avg_eff, params.lift_avg_eff, ok_dict[avg_eff_ok]))
    html.append('</table>')

    result.html = '\n'.join(html)

    result.result = mse_ok and avg_abs_ok
    
    result.values = []
    result.values.append(TestValue('Lift MSE', str(mse), '', str(params.lift_mse)))
    result.values.append(TestValue('Lift Avg. Abs. Effort', str(avg_abs), '', str(params.lift_avg_abs)))
    result.values.append(TestValue('Lift Avg.  Effort', str(avg_eff), '', str(params.lift_avg_eff)))

    return result
    
def analyze_flex_efforts(params, data):
    result = CounterbalanceAnalysisResult()
    
    avg_efforts = numpy.array(get_efforts(data, False))
    mse = get_mean_sq_effort(avg_efforts)
    avg_abs = get_mean_abs_effort(avg_efforts)
    avg_eff = get_mean_effort(avg_efforts)

    mse_ok = mse < params.flex_mse
    avg_abs_ok = avg_abs < params.flex_avg_abs
    avg_eff_ok = abs(avg_eff) < abs(params.flex_avg_eff)

    if mse_ok and avg_abs_ok:
        result.summary = 'Flex efforts OK'
    else:
        result.summary = 'Flex MSE/Avg. Absolute effort too high'

    html = ['<p>%s</p>' % result.summary]
    
    html.append('<table border="1" cellpadding="2" cellspacing="0">')
    html.append('<tr><td><b>Parameter</b></td><td><b>Value</b></td><td><b>Maximum</b></td><td><b>Status</b></td></tr>')
    html.append('<tr><td><b>Mean Sq. Effort</b></td><td>%.2f</td><td>%.2f</td><td>%s</td></tr>' % (mse, params.flex_mse, ok_dict[mse_ok]))
    html.append('<tr><td><b>Average Abs. Effort</b></td><td>%.2f</td><td>%.2f</td><td>%s</td></tr>' % (avg_abs, params.flex_avg_abs, ok_dict[avg_abs_ok]))
    html.append('<tr><td><b>Average Effort</b></td><td>%.2f</td><td>%.2f</td><td>%s</td></tr>' % (avg_eff, params.flex_avg_eff, ok_dict[avg_eff_ok]))
    html.append('</table>')

    result.html = '\n'.join(html)

    result.result = mse_ok and avg_abs_ok
    
    result.values = []
    result.values.append(TestValue('Flex MSE', str(mse), '', str(params.flex_mse)))
    result.values.append(TestValue('Flex Avg. Abs. Effort', str(avg_abs), '', str(params.flex_avg_abs)))
    result.values.append(TestValue('Flex Avg.  Effort', str(avg_eff), '', str(params.flex_avg_eff)))

    return result

class CounterbalanceAnalysis:
    def __init__(self):
        self._sent_results = False

        self._hold_data_srvs = []

        self._motors_halted = True

        rospy.init_node('cb_analysis')
        self.motors_topic = rospy.Subscriber('pr2_etherCAT/motors_halted', Bool, self.motors_cb)
        self.data_topic = rospy.Subscriber('cb_test_data', CounterbalanceTestData, self.data_callback)
        self._result_service = rospy.ServiceProxy('test_result', TestResult)

    def send_results(self, r):
        if not self._sent_results:
            try:
                rospy.wait_for_service('test_result', 10)
            except:
                rospy.logerr('Wait for service \'test_result\' timed out! Unable to send results.')
                return False
                
            self._result_service.call(r)
            self._sent_results = True

            print r.text_summary
            print r.html_result
            print r.result
            return True
            
    def test_failed_service_call(self, except_str = ''):
        rospy.logerr(except_str)
        r = TestResultRequest()
        r.html_result = except_str
        r.text_summary = 'Caught exception, automated test failure.'
        r.result = TestResultRequest.RESULT_FAIL
        self.send_results(r)

    def motors_cb(self, msg):
        self._motors_halted = msg.data

    def data_callback(self, msg):
        self.process_results(msg)
        
    def process_results(self, msg):
        try:
            data = CounterbalanceAnalysisData(msg)
            params = CounterbalanceAnalysisParams(msg)

            lift_effort_result = analyze_lift_efforts(params, data)
            lift_effort_plot = plot_efforts_by_lift_position(params, data)

            if params.flex_test:
                flex_effort_result = analyze_flex_efforts(params, data)
                lift_effort_contour = plot_effort_contour(params, data)

            html = ['<p>Counterbalance Analysis</p>']
            if params.flex_test:
                html.append('<H4>Lift Effort Contour Plot</H4>')
                html.append('<img src=\"IMG_PATH/%s.png\" width=\"640\" height=\"480\" />' % (lift_effort_contour.title))
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
           
            ##\todo If flex test, check flex data


            r = TestResultRequest()
            r.html_result = '\n'.join(html)
            r.text_summary = ' '.join([lift_effort_result.summary])
            if params.flex_test:
                r.text_summary = ' '.join([r.text_summary, flex_effort_result.summary])

            r.result = TestResultRequest.RESULT_HUMAN_REQUIRED

            if params.flex_test and lift_effort_result.result and flex_effort_result.result:
                r.result = TestResultRequest.RESULT_PASS
            elif lift_effort_result.result:
                r.result = TestResultRequest.RESULT_PASS


            r.plots = [ lift_effort_plot ]
            if params.flex_test:
                r.plots.append(lift_effort_contour)
            r.params = params.get_test_params()
            r.values = lift_effort_result.values

            if self._motors_halted:
                r.text_summary = 'Fail, motors halted. Check estop and power board.'
                r.html_result = '<H4>Motors Halted</H4>\n<p>Unable to analyze CB. Check estop and power board.</p>\n' + r.html_result
                r.result = TestResultRequest.RESULT_FAIL

            self.send_results(r)
        except Exception, e:
            import traceback
            self.test_failed_service_call(traceback.format_exc());

            
if __name__ == '__main__':
    app = CounterbalanceAnalysis()
    try:
        rospy.spin()
    except KeyboardInterrupt, e:
        pass
    except Exception, e:
        print 'Caught Exception in CB application'
        import traceback
        traceback.print_exc()
        result_service = rospy.ServiceProxy('test_result', TestResult)        
        
        r = TestResultRequest()
        r.html_result = traceback.format_exc()
        r.text_summary = 'Caught exception, automated test failure.'
        r.plots = []
        r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
        result_service.call(r)
