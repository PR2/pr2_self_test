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
##\brief Analyzes counterbalance data

PKG = 'qualification'
import roslib; roslib.load_manifest(PKG)

import numpy
import math
import matplotlib
import matplotlib.pyplot as plot
from StringIO import StringIO

from qualification.msg import Plot, TestValue, TestParam

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
    ##\param msg CounterbalanceTestData : Message from controller
    def __init__(self, msg):
        self.lift_data = []
        for ld in msg.lift_data:
            self.lift_data.append(CBRunAnalysisData(ld))

##\brief Stores parameters from CB analysis test
class CounterbalanceAnalysisParams:
    def __init__(self, msg):
        self.lift_dither  = msg.lift_amplitude
        self.flex_dither  = msg.flex_amplitude
        
        self.lift_joint   = msg.lift_joint
        self.flex_joint   = msg.flex_joint
        
        self.timeout_hit  = msg.timeout_hit
        self.flex_test    = msg.flex_test

        self.lift_mse     = msg.arg_value[9]
        self.lift_avg_abs = msg.arg_value[10]
        self.lift_avg_eff = msg.arg_value[11]
        self.flex_mse     = msg.arg_value[12]
        self.flex_avg_abs = msg.arg_value[13]
        self.flex_avg_eff = msg.arg_value[14]

        self.lift_p       = msg.arg_value[15]
        self.lift_i       = msg.arg_value[16]
        self.lift_d       = msg.arg_value[17]
        self.lift_i_clamp = msg.arg_value[18]

        self.flex_p       = msg.arg_value[19]
        self.flex_i       = msg.arg_value[20]
        self.flex_d       = msg.arg_value[21]
        self.flex_i_clamp = msg.arg_value[22]

        if len(msg.arg_value) > 24:
            self.screw_tol = msg.arg_value[23]
            self.bar_tol   = msg.arg_value[24]
        else:
            # For backwards compatibility
            self.screw_tol = 2.0
            self.bar_tol   = 1.0

        self.num_flexes = len(msg.lift_data[0].flex_data)
        self.num_lifts  = len(msg.lift_data)
        
        self.min_lift = msg.lift_data[0].lift_position
        self.max_lift = msg.lift_data[-1].lift_position

        self.min_flex = msg.lift_data[0].flex_data[0].flex_position
        self.max_flex = msg.lift_data[0].flex_data[-1].flex_position

        self.named_params = {}
        for i in range(0, 9):
            self.named_params[msg.arg_name[i]] = msg.arg_value[i]

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


        params.append(TestParam(key='Lift P Gain', value=str(self.lift_p)))
        params.append(TestParam(key='Lift I Gain', value=str(self.lift_i)))
        params.append(TestParam(key='Lift D Gain', value=str(self.lift_d)))
        params.append(TestParam(key='Lift I Clamp', value=str(self.lift_i_clamp)))

        params.append(TestParam(key='Num Lifts', value=str(self.num_lifts)))
        params.append(TestParam(key='Min Lift', value="%.2f" % self.min_lift))
        params.append(TestParam(key='Max Lift', value="%.2f" % self.max_lift))

        if self.flex_test:
            params.append(TestParam(key='Flex MSE', value=str(self.flex_mse)))
            params.append(TestParam(key='Flex Avg Abs', value=str(self.flex_avg_abs)))
            params.append(TestParam(key='Flex Avg Effort', value=str(self.flex_avg_eff)))
            params.append(TestParam(key='Flex P Gain', value=str(self.flex_p)))
            params.append(TestParam(key='Flex I Gain', value=str(self.flex_i)))
            params.append(TestParam(key='Flex D Gain', value=str(self.flex_d)))
            params.append(TestParam(key='Flex I Clamp', value=str(self.flex_i_clamp)))
            params.append(TestParam(key='Num Flexes', value=str(self.num_flexes)))
            params.append(TestParam(key='Min Flex', value="%.2f" % self.min_flex))
            params.append(TestParam(key='Max Flex', value="%.2f" % self.max_flex))

        for key, val in self.named_params.iteritems():
            params.append(TestParam(key=key, value=str(val)))
            

        return params

class CounterbalanceAnalysisResult:
    def __init__(self):
        self.html = ''
        self.summary = ''
        self.result = False
        self.values = []

##\brief Get average efforts for CB test as a list
##
##\param lift_calc bool : Lift or flex efforts
def get_efforts(data, lift_calc):
    avg_effort_list = []
    for ld in data.lift_data:
        for fd in ld.flex_data:
            if lift_calc:
                avg_effort_list.append(fd.lift_hold.effort_avg)
            else:
                avg_effort_list.append(fd.flex_hold.effort_avg)

    return avg_effort_list

def _get_mean_sq_effort(avg_effort_array):
    sq_array = avg_effort_array * avg_effort_array
    return numpy.average(sq_array)

def _get_mean_abs_effort(avg_effort_array):
    abs_array = abs(avg_effort_array)
    return numpy.average(abs_array)

def _get_mean_effort(avg_effort_array):
    return numpy.average(avg_effort_array)

##\brief Returns a list of lift positions, efforts for a given flex position (vary by lift)
def _get_const_flex_effort(data, flex_index = 0, lift_calc = True):
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
    
def _get_const_lift_effort(data, lift_index = 0, lift_calc = True):
    ld = data.lift_data[lift_index]
        
    effort_list = []
    flex_list = []

    for fd in ld.flex_data:
        flex_list.append(fd.flex_position)
        if lift_calc:
            effort_list.append(fd.lift_hold.effort_avg)
        else:
            effort_list.append(fd.flex_hold.effort_avg)

    return flex_list, effort_list



def _get_flex_positions(data):
    flex_list = []
    for fd in data.lift_data[0].flex_data:
        flex_list.append(fd.flex_position)

    return flex_list

def _get_lift_positions(data):
    lifts = []
    for ld in data.lift_data:
        lifts.append(ld.lift_position)
    return lifts


##\brief Gives effort contour plot of efforts by lift, flex position
##
##\param params CounterbalanceAnalysisParams : Input params
##\param data CounterbalanceAnalysisData : Test Data
##\return qualification.msg.Plot : Plot message with contour
def plot_effort_contour(params, data, lift_calc = True):
    effort_list = []
    for i in range(0, params.num_lifts):
        flexes, efforts = _get_const_lift_effort(data, i, lift_calc)
        effort_list.append(efforts)
    flexes = _get_flex_positions(data)
    lifts = _get_lift_positions(data)

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

##\brief Plots CB efforts against shoulder lift position
##
##\param flex_index int : Index of flex data to plot against
##\param lift_calc bool : Lift efforts or flex efforts
def plot_efforts_by_lift_position(params, data, flex_index = -1, lift_calc = True):
    lift_position, effort = _get_const_flex_effort(data, flex_index, lift_calc)
    
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

##\brief Checks shoulder lift efforts against params
##
##\return CounterbalanceAnalysisResult
def analyze_lift_efforts(params, data):
    result = CounterbalanceAnalysisResult()
    
    avg_efforts = numpy.array(get_efforts(data, True))
    mse = _get_mean_sq_effort(avg_efforts)
    avg_abs = _get_mean_abs_effort(avg_efforts)
    avg_eff = _get_mean_effort(avg_efforts)

    mse_ok = mse < params.lift_mse
    avg_abs_ok = avg_abs < params.lift_avg_abs
    avg_eff_ok = abs(avg_eff) < abs(params.lift_avg_eff)

    if mse_ok and avg_abs_ok:
        result.summary = 'Lift efforts OK'
    elif avg_eff < 0:
        result.summary = 'Counterbalance is too weak. Requires adjustment'
    else:
        result.summary = 'Counterbalance is too strong. Requires adjustment'

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
    
##\brief Checks shoulder flex efforts against params
##
##\return CounterbalanceAnalysisResult
def analyze_flex_efforts(params, data):
    result = CounterbalanceAnalysisResult()
    
    avg_efforts = numpy.array(get_efforts(data, False))
    mse = _get_mean_sq_effort(avg_efforts)
    avg_abs = _get_mean_abs_effort(avg_efforts)
    avg_eff = _get_mean_effort(avg_efforts)

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

##\brief Return CB adjustments to minimize total torque
##
## Uses CB adjustment/torque value from the model file. 
## Model file is generated by 'test_pr2_self_test/counterbalance_training.py'
##\param params CounterbalanceAnalysisParams
##\param data CounterbalanceAnalysisData
##\param str : Filename of model file
def check_cb_adjustment(params, data, model_file):
    result = CounterbalanceAnalysisResult()

    try:
        # Use this to tune to last known "good" position
        # A = numpy.load(model_file).transpose()
        
        # This uses minimum of total torque
        A = numpy.load(model_file)[:-1].transpose() 
        B = numpy.array(get_efforts(data, True) + get_efforts(data, False))
        X = numpy.linalg.lstsq(A,B)
    except:
        result.result = False
        result.summary = 'Unable to calculate CB adjustment. Data may be corrupt or CB may be too far off'
        result.html = '<p>Unable to calculate CB adjustment.\nException:\n%s</p>\n' % traceback.format_exc()
        return result

    # Recommended adjustments
    secondary = -X[0][0]
    cb_bar = -X[0][1] # Same as arm gimbal shaft

    
    secondary_dir = 'CW' if secondary > 0 else 'CCW' # CW increases CB force
    cb_bar_dir = 'CW' if cb_bar > 0 else 'CCW' # CCW increases CB force
    
    adjust_msg = 'Please turn secondary by %.2f turns %s and the arm gimbal shaft by %.2f turns %s' % (secondary, secondary_dir, cb_bar, cb_bar_dir)
    adjust_msg = '<table border="1" cellpadding="2" cellspacing="0">'
    adjust_msg += '<tr><td><b>Adjustment</b></td><td><b>Turns</b></td><td><b>Direction</b></td><td><b>Allowable Tolerance</b></td></tr>\n'
    adjust_msg += '<tr><td>Secondary Spring</td><td>%.1f</td><td>%s</td><td>%.1f</td></tr>\n' % (abs(secondary), secondary_dir, params.screw_tol)
    adjust_msg += '<tr><td>Arm Gimbal Shaft</td><td>%.1f</td><td>%s</td><td>%.1f</td></tr>\n' % (abs(cb_bar), cb_bar_dir, params.bar_tol)
    adjust_msg += '</table>\n'

    
    if abs(secondary) > params.screw_tol or abs(cb_bar) > params.bar_tol:
        result.result = False
        result.summary = 'CB adjustment recommended. Follow instructions below to tune CB. '
        result.html = '<p>CB adjustment recommended. Adjusting the counterbalance will increase performance of the arm. (Note: CW = "Clockwise")</p>\n<p>%s</p>\n' % adjust_msg
    else:
        result.result = True
        result.summary = ''
        result.html = '<p>No CB adjustment recommended. You may choose to adjust the CB using the following instructions, but this is within tolerance.</p>\n<p>%s</p>' % adjust_msg


    result.values = [TestValue('Secondary Spring Adjustment', str(secondary), '', str(params.screw_tol)),
                     TestValue('CB Bar Adjustment', str(cb_bar), '', str(params.bar_tol))]

    return result

    
