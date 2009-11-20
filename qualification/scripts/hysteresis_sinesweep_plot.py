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

from joint_qualification_controllers.srv import TestData, TestDataResponse

import traceback

class AnalysisApp:
  def __init__(self):
    self.data_sent = False
    rospy.init_node("test_plotter")
    self.data_topic = rospy.Service("/test_data", TestData, self.on_data)
    self.result_service = rospy.ServiceProxy('/test_result', TestResult)
    rospy.spin()
    
  def on_data(self, req):
    rospy.loginfo('Got data named %s' % (req.test_name))
    self.data = req
    if self.data.test_name == "hysteresis":
      self.hysteresis_plot()
    elif self.data.test_name == "sinesweep":
      self.sine_sweep_plot()
    else:
      rospy.logerr('Recieved test message with name %s, unable to analyze' % s)
      self.test_failed_service_call('Unable to analyze result.')
    return TestDataResponse(1)
      
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
      rospy.wait_for_service('test_result')
      self.result_service.call(test_result)
      self.data_sent = True

  def hysteresis_plot(self):
    try:
      r = TestResultRequest()
      r.html_result = ''
      r.text_summary = 'No data.'
      r.plots = []
      r.result = TestResultRequest.RESULT_FAIL

      image_title = self.data.joint_name + "_hysteresis"

      # Check the num points, timeout
      if len(self.data.position) < 250:
        param_html = self.controller_params()
        r.result = TestResultRequest.RESULT_FAIL
        r.text_summary = "Not enough data points, bad encoder or bad gains."
        
        error_msg = "<p>Not enough data points, hysteresis controller may have malfunctioned. Check diagnostics.</p>"
        error_msg += "<p>The encoder may be missing ticks, check encoder.<br>\n"
        error_msg += "Make sure mechanism is free to move and not trapped or constrained<br>\n"
        error_msg += "Make sure motors are not in safety lockout (check diagnostics)<br>\n"
        error_msg += "Check controller gains if problem persists on components of same type.</p>\n"
        error_msg += "<p>Test status: <b>FAIL</b>.</p>"
        error_msg += '<p align=center><b>Test Parameters</b></p>' + param_html + '<hr size="2">\n'
        
        r.html_result = error_msg
        self.send_results(r)
        return

      if self.data.arg_value[5] == 0:
        param_html = self.controller_params()
        r.result = TestResultRequest.RESULT_FAIL
        r.text_summary = 'Hysteresis controller timed out. Check diagnostics.'
        r.html_result = '<p>Hysteresis controller timed out. Check diagnostics. Controller gains may be bad, or motors may be in safety lockout. Did the device pass the visualizer? Is it calibrated?</p><p>Test status: <b>FAIL</b>.</p>'
        r.html_result += '<p align=center><b>Test Parameters</b></p>' + param_html + '<hr size="2">\n'
        self.send_results(r)
        return

      # Compute the encoder travel
      min_encoder = min(numpy.array(self.data.position))
      max_encoder = max(numpy.array(self.data.position))

      if abs(max_encoder - min_encoder) < 0.005:
        param_html = self.controller_params()
        r.result = TestResultRequest.RESULT_FAIL
        r.text_summary = 'Mechanism didn\'t move. Check diagnostics.'
        r.html_result = "<p>No travel of mechanism, hysteresis did not complete. Check controller gains and encoder.</p><p>Test status: <b>FAIL</b>.</p>"
        r.html_result += '<p align=center><b>Test Parameters</b></p>' + param_html + '<hr size="2">\n'
        self.send_results(r)
        return
      
      # Find the index to do the average over
      # Data set starts close to one end of hysteresis
      # Find endpoint of travel closest to midpoint of data
      index_max = numpy.argmax(numpy.array(self.data.position))
      index_min = numpy.argmin(numpy.array(self.data.position))
      end = numpy.array(self.data.position).size
      if (abs(end/2 - index_max) < abs(end/2 - index_min)):
        index = index_max
      else:
        index = index_min
        
      # Find indices to compute averages / SD over
      # Take middle 90% of array by index
      array1_min = int(0.05 * index)
      array1_max = int(0.95 * index)
      array2_min = index + int(0.05 * (end - index))
      array2_max = index + int(0.95 * (end - index))

      effort1_array   = numpy.array(self.data.effort)  [array1_min: array1_max]
      position1_array = numpy.array(self.data.position)[array1_min: array1_max]
      velocity1_array = numpy.array(self.data.velocity)[array1_min: array1_max]

      effort2_array   = numpy.array(self.data.effort)  [array2_min: array2_max]
      position2_array = numpy.array(self.data.position)[array2_min: array2_max]
      velocity2_array = numpy.array(self.data.velocity)[array2_min: array2_max]
      
      # Make sure we have at least some points in both directions
      if effort1_array.size < 20 or effort2_array.size < 20:
        param_html = self.controller_params()
        r.result = TestResultRequest.RESULT_FAIL
        r.text_summary = 'Incomplete data set. Check diagnostics.'
        r.html_result = "<p>Not enough data in one or more directions.</p><p>Test Status: <b>FAIL</b>.</p><p>Test status: <b>FAIL</b>.</p>"
        r.html_result += '<p align=center><b>Test Parameters</b></p>' + param_html + '<hr size="2">\n'
        self.send_results(r)
        return

      effort1 = numpy.average(effort1_array)
      effort2 = numpy.average(effort2_array)
      
      if effort1 < effort2:
        pos_eff = effort2_array
        neg_eff = effort1_array
        pos_position = position2_array
        neg_position = position1_array    
        pos_vel = velocity2_array
        neg_vel = velocity1_array
      else:
        pos_eff = effort1_array
        neg_eff = effort2_array
        pos_position = position1_array
        neg_position = position2_array
        pos_vel = velocity1_array
        neg_vel = velocity2_array

      min_avg = numpy.average(neg_eff)
      min_sd  = numpy.std(neg_eff)
      max_avg = numpy.average(pos_eff)
      max_sd  = numpy.std(pos_eff)

      # Unpack expected values
      min_effort_param = self.data.arg_value[0]
      max_effort_param = self.data.arg_value[1]
      min_range_param  = self.data.arg_value[2]
      max_range_param  = self.data.arg_value[3]
      tol_percent      = self.data.arg_value[7]
      sd_max_percent   = self.data.arg_value[8]

      # Process HTML result
      range_html, range_sum, range_result = self.hysteresis_range( min_encoder, max_encoder, min_range_param, max_range_param)

      slope = self.data.arg_value[9]
      if slope == 0:
        effort_html, effort_sum, effort_result = self.hysteresis_effort( min_effort_param, max_effort_param, neg_eff, pos_eff, tol_percent, sd_max_percent)
      else:
        effort_html, effort_sum, effort_result = self.hysteresis_regression(pos_position, pos_eff, neg_position, neg_eff, slope, tol_percent, sd_max_percent)

      
      vel_html = self.hysteresis_velocity(self.data.arg_value[4], pos_position, pos_vel, neg_position, neg_vel)

      param_html = self.controller_params()

      html = '<img src=\"IMG_PATH/%s1.png\", width = 640, height = 480/>' % image_title
      html += '<p align=center><b>Range Data</b></p>' + range_html + '<hr size="2">\n'
      html += '<p align=center><b>Effort Data</b></p>' + effort_html + '<hr size="2">\n'

      html += '<p align=center><b>Velocity Data</b></p>' 
      html += '<img src=\"IMG_PATH/%s2.png\", width = 640, height = 480/>' % image_title
      html += vel_html + '<hr size="2">\n'
      #html += '<p align=center><b>Regression Analysis</b></p>' + regress_html + '<hr size="2">\n'
      html += '<p align=center><b>Test Parameters</b></p>' + param_html + '<hr size="2">\n'

      html += '<img src=\"IMG_PATH/%s3.png\", width = 640, height = 480/>' % image_title

      r.html_result = html

      if range_result and effort_result:
        r.text_summary = 'Hysteresis result: OK'
        r.result = TestResultRequest.RESULT_PASS
      elif not range_result:
        r.text_summary = 'Hysteresis range failed.'
        r.result = TestResultRequest.RESULT_FAIL
      else:
        r.text_summary = 'Hysteresis result: FAIL. ' + range_sum + ' ' + effort_sum
        r.result = TestResultRequest.RESULT_HUMAN_REQUIRED

      # Can move to separate plotting function
      # Plot the analyzed data
      fig = plot.figure(1)
      axes1 = fig.add_subplot(211)
      axes2 = fig.add_subplot(212)
      axes2.set_xlabel('Position')
      axes1.set_ylabel('Effort (+ dir)')
      axes2.set_ylabel('Effort (- dir)')
      axes1.plot(pos_position, pos_eff, 'b--', label='_nolegend_')
      axes2.plot(neg_position, neg_eff, 'b--', label='_nolegend_')
      if max_avg != 0 or min_avg != 0:
        # Add error bars for SD
        axes1.axhline(y = max_avg, color = 'r', label='Average')
        axes1.axhline(y = max_avg + max_sd, color = 'y', label='Error bars')
        axes1.axhline(y = max_avg - max_sd, color = 'y', label='_nolegend_')

        axes2.axhline(y = min_avg, color = 'r', label='Average')
        axes2.axhline(y = min_avg + min_sd, color = 'y', label='Error')
        axes2.axhline(y = min_avg - min_sd, color = 'y', label='_nolegend_')

      # Add expected efforts to both plots
      axes2.axhline(y = self.data.arg_value[0], color = 'g', label='_nolegend_')
      axes1.axhline(y = self.data.arg_value[1], color = 'g', label='Expected')
       
      fig.text(.3, .95, self.data.joint_name + ' Hysteresis Analysis')
      axes1.legend(shadow=True)
      

      stream = StringIO()
      plot.savefig(stream, format = "png")
      image = stream.getvalue()
      
      p = Plot()
      r.plots.append(p)
      p.title = image_title + "1"
      p.image = image
      p.image_format = "png"

      plot.close()

      fig = plot.figure(2)
      plot.ylabel('Velocity')
      plot.xlabel('Position')
      plot.plot(numpy.array(self.data.position), numpy.array(self.data.velocity), 'b--', label='Data')
      plot.axhline(y = self.data.arg_value[4], color = 'g', label = 'Command')
      plot.axhline(y = -1 * self.data.arg_value[4], color = 'g', label = '_nolegend_')

      fig.text(.3, .95, self.data.joint_name + ' Hysteresis Velocity')
      plot.legend(shadow=True)

      stream = StringIO()
      plot.savefig(stream, format = "png")
      image = stream.getvalue()
      
      p = Plot()
      r.plots.append(p)
      p.title = image_title + "2"
      p.image = image
      p.image_format = "png"

      # Create the figure 3: Effort vs. Position.
      fig=plot.figure(3)
      
      # Plot the effort hysteresis
      plot.plot(numpy.array(self.data.position), numpy.array(self.data.effort), 'b--')
      # Show the expected average effort lines in blue
      plot.axhline(y = self.data.arg_value[1], color = 'g', label='Expected')
      plot.axhline(y = 0, color = 'k')
      plot.axhline(y = self.data.arg_value[0], color = 'g', label='_nolegend_')
      # Show actual average effort lines in green
      if max_avg != 0 or min_avg != 0:
        plot.axhline(y = max_avg, color='r', label='Average')
        plot.axhline(y = min_avg, color='r', label='_nolegend_')

      
      # Joint name on title
      fig.text(.3, .95, self.data.joint_name + ' Hysteresis Data')
      plot.legend()

      stream = StringIO()
      plot.savefig(stream, format = "png")
      image = stream.getvalue()
      
      p = Plot()
      r.plots.append(p)
      p.title = image_title + "3"
      p.image = image
      p.image_format = "png"

      self.send_results(r)
    except Exception, e:
      rospy.logerr('hysteresis_plot caught exception, returning test failure.')
      rospy.logerr(traceback.format_exc())
      self.test_failed_service_call(traceback.format_exc())

  # Processes range data for hysteresis test
  def hysteresis_range(self, min, max, min_expect, max_expect):
    if (min_expect == 0 and max_expect == 0):
      return '<p>Continuous joint, no range data.</p>\n', 'Range: Continuous.', True

    fail = '<div class="error">FAIL</div>'
    ok = '<div class="pass">OK</div>'

    max_ok = True
    min_ok = True
    min_status = ok
    max_status = ok
    # Check to range against expected values
    if (min > min_expect):
      min_ok = False
      min_status = fail

    if (max < max_expect):
      max_ok = False
      max_status = fail

    result = min_ok and max_ok

    summary = 'Range: FAIL.'
    if result:
      summary = 'Range: OK.'

    if result:
      html = '<p>Range of motion: OK.</p>\n'
    elif max_ok and not min_ok:
      html = "<p>Mechanism did not reach expected minimum range.</p>\n"
    elif min_ok and not max_ok:
      html = "<p>Mechanism did not reach expected maximum range.</p>\n"
    else:
      html = "<p>Mechanism did not reach expected minimum or maximum range.</p>\n"

    table = '<table border="1" cellpadding="2" cellspacing="0">\n'
    table += '<tr><td></td><td><b>Observed</b></td><td><b>Expected</b></td><td><b>Status</b></td></tr>\n' 
    table += '<tr><td>Maximum</td><td>%.3f</td><td>%.3f</td><td>%s</td></tr>\n' % (max, max_expect, max_status)
    table += '<tr><td>Minimum</td><td>%.3f</td><td>%.3f</td><td>%s</td></tr>\n' % (min, min_expect, min_status)
    table += '</table>\n'

    html += table

    return html, summary, result

  # Processes range data for hysteresis test
  def hysteresis_effort(self, min_exp, max_exp, min_array, max_array, 
                        tol_percent, sd_tol_percent):
    # Check 
    negative_msg = "OK"
    positive_msg = "OK"

    max_avg = numpy.average(max_array)
    min_avg = numpy.average(min_array)
    max_sd = numpy.std(max_array)
    min_sd = numpy.std(min_array)

    tolerance = abs(max_exp - min_exp) * tol_percent

    sd_denominator = abs(max_avg - min_avg)
    sd_tol = sd_denominator * sd_tol_percent
    
    max_ok = abs(max_avg - max_exp) < tolerance
    min_ok = abs(min_avg - min_exp) < tolerance
  
    max_even = max_sd < sd_tol
    min_even = min_sd < sd_tol

    summary = 'Effort: FAIL.'
    result = False
    if max_ok and min_ok and max_even and min_even:
      html = '<p>Efforts OK.</p>\n'
      summary = 'Effort: OK.'
      result = True
    elif max_ok and min_ok:
      html = '<p>Efforts have acceptable average, but are uneven.</p>\n'
    elif max_avg - max_exp > tolerance and min_exp - min_avg > tolerance:
      html = '<p>Effort is too high.</p>\n'
    elif max_avg - max_exp < tolerance and min_exp - min_avg < tolerance:
      html = '<p>Effort is too low.</p>\n'
    else:
      html = '<p>Efforts are outside acceptable parameters. See graphs.</p>\n'
      
    # Standard deviations in percent of value
    sd_min_percent = abs(min_sd / sd_denominator) * 100
    sd_max_percent = abs(max_sd / sd_denominator) * 100

    if max_even:
      positive_sd_msg = '<div class="pass">OK</div>'
    else:
      positive_sd_msg = '<div class="warning">UNEVEN</div>'

    if min_even:
      negative_sd_msg = '<div class="pass">OK</div>'
    else:
      negative_sd_msg = '<div class="warning">UNEVEN</div>'
      
    if max_ok:
      positive_msg = '<div class="pass">OK</div>'
    else:
      positive_msg = '<div class="error">FAIL</div>'

    if min_ok:
      negative_msg = '<div class="pass">OK</div>'
    else:
      negative_msg = '<div class="error">FAIL</div>'

    html += '<table border="1" cellpadding="2" cellspacing="0">\n'
    html += '<tr><td><b>Name</b></td><td><b>Average</b></td><td><b>Expected</b></td><td><b>Status</b></td><td><b>SD %</b></td><td><b>Status</b></td></tr>\n'
    html += '<tr><td>Positive</td><td>%.2f</td><td>%.2f</td><td>%s</td><td>%.2f</td><td>%s</td></tr>\n' % (max_avg, max_exp, positive_msg, sd_max_percent, positive_sd_msg)  
    html += '<tr><td>Negative</td><td>%.2f</td><td>%.2f</td><td>%s</td><td>%.2f</td><td>%s</td></tr>\n' % (min_avg, min_exp, negative_msg, sd_min_percent, negative_sd_msg)  
    html += '</table><br>\n'    
    html += '<p>Effort tolerance: %.2f. SD tolerance: %.2f percent</p>\n' % (tolerance, sd_tol_percent)

    return html, summary, result

  def hysteresis_velocity(self, search, pos_position, pos_vel, neg_position, neg_vel):
    html = '<p>Search velocity: %.2f.</p><br>\n' % abs(search)

    pos_avg = numpy.average(pos_vel)
    pos_sd = numpy.std(pos_vel)
    pos_rms = math.sqrt(numpy.average( (pos_vel - search) * (pos_vel - search) ))

    neg_avg = numpy.average(neg_vel)
    neg_sd = numpy.std(neg_vel)
    neg_rms = math.sqrt(numpy.average( (neg_vel + search) * (neg_vel + search) ))


    html += '<table border="1" cellpadding="2" cellspacing="0">\n'
    html += '<tr><td></td><td><b>Average</b></td><td><b>Std. Dev.</b></td><td><b>RMS</b></td></tr>\n'
    html += '<tr><td>Positive</td><td>%.2f</td><td>%.3f</td><td>%.3f</td></tr>\n' % (pos_avg, pos_sd, pos_rms)
    html += '<tr><td>Negative</td><td>%.2f</td><td>%.3f</td><td>%.3f</td></tr>\n' % (neg_avg, neg_sd, neg_rms)
    html += '</table>\n'
    
    return html

  def hysteresis_regression(self, pos_position, pos_effort, neg_position, neg_effort, slope_exp, tolerance, sd_max):
    A_pos = numpy.vstack([pos_position, numpy.ones(len(pos_position))]).T
    A_neg = numpy.vstack([neg_position, numpy.ones(len(neg_position))]).T
    
    # If we are given a slope, check it against the values
    coeff_max = numpy.polyfit(pos_position, pos_effort, 1)
    coeff_min = numpy.polyfit(neg_position, neg_effort, 1)

    # y = ax + b
    a_max, b_max = numpy.linalg.lstsq(A_pos, pos_effort)[0]
    a_min, b_min = numpy.linalg.lstsq(A_neg, neg_effort)[0]

    # Check min/max slopes close together
    slope_avg = 0.5 * (a_max + a_min)
    slope_diff = abs((a_max - a_min) / slope_avg)
    slope_sd_tol = abs(slope_exp * sd_max)

    # Check that slope value is close to expected
    slope_err = abs(slope_exp - slope_avg)
    
    diff_ok = slope_diff < slope_sd_tol
    avg_ok = slope_err < abs(tolerance * slope_avg)
    slope_ok = diff_ok and avg_ok

    ok_dict = {True: 'OK', False: 'FAIL'}

    if slope_ok:
      summary = "Slope: OK"
    else:
      summary = "Slope: FAIL"
    
    # Check if slope is significant....
    regress_html = '<p>Simple linear regression on the effort and position plot data. Expected slope: %0.2f (%s). Difference in slopes: %0.2f (%s).</p><br>\n' % (slope_exp, avg_ok, slope_diff, diff_ok)
    regress_html += '<table border="1" cellpadding="2" cellspacing="0">\n'
    regress_html += '<tr><td><b>Name</b></td><td><b>Slope</b></td><td><b>Intercept</b></td>\n' 
    regress_html += '<tr><td>%s</td><td>%.3f</td><td>%.3f</td>\n' % ('Positive', a_max, b_max) 
    regress_html += '<tr><td>%s</td><td>%.3f</td><td>%.3f</td>\n' % ('Negative', a_min, b_min) 
    regress_html += '</table>\n'

    return regress_html, summary, slope_ok

    # Add table of all test params
  def controller_params(self):
    html = '<p>Test parameters from controller.</p><br>\n'
    html += '<table border="1" cellpadding="2" cellspacing="0">\n'
    html += '<tr><td><b>Name</b></td><td><b>Value</b></td></tr>\n'
    for i in range(0, len(self.data.arg_name)):
      html += '<tr><td>%s</td><td>%.2f</td></tr>\n' % (self.data.arg_name[i], self.data.arg_value[i])
    html += '</table>\n'

    return html


  def sine_sweep_plot(self):
    try:
      r = TestResultRequest()
      r.html_result = ''
      r.text_summary = 'No data.'
      r.plots = []
      r.result = TestResultRequest.RESULT_FAIL

      image_title = self.data.joint_name + "_sine_sweep"

      # Compute the encoder travel
      min_encoder = min(numpy.array(self.data.position))
      max_encoder = max(numpy.array(self.data.position))
      
      if abs(max_encoder - min_encoder) < 0.001:
        r.html_result = "<p>No travel of mechanism, sinesweep did not complete. Check controller gains and encoder.</p><p>Test status: <b>FAIL</b>.</p>"
        r.text_summary = "No travel of mechanism. Bad encoder, bad gains, or motors on lockout."
        self.result_service.call(r)
      
      # Plot the values and line of best fit
      fig = plot.figure(1)
      axes1 = fig.add_subplot(211)
      axes1.clear()
      axes2 = fig.add_subplot(212)
      axes2.clear()
      # Find the next power of two above our number of data points
      next_pow_two=int(2**math.ceil(math.log(numpy.array(self.data.position).size,2)))
      # plot in decibels
      axes1.psd(numpy.array(self.data.effort), NFFT=next_pow_two, Fs=1000, Fc=0, color='r')
      axes1.psd(numpy.array(self.data.position), NFFT=next_pow_two, Fs=1000, Fc=0)
      #axes1.set_xlim(0, 100)
      axes1.set_title('Position PSD')
    
      # plot in power (pxx - power, f - freqs)
      pxx, f = axes2.psd(numpy.array(self.data.velocity), NFFT=next_pow_two, Fs=1000, Fc=0)
      axes2.clear()
      for i in f:
        if f[i]>4:
          cutoff=i # Set cutoff to avoid finding first mode at extremely low freqs
          break
        axes2.plot(f,pxx)

      # find the peak
      index = numpy.argmax(pxx[cutoff:pxx.size])
      index = index + cutoff
      max_value = max(pxx[cutoff:pxx.size])
      axes2.plot([f[index]],[pxx[index]],'r.', markersize = 10);
      self.first_mode = f[index]

      # Find second mode
      # Max after first mode
      #second_array = pxx[index + cutoff: pxx.size]
      #if len(second_array > 10):
      #  index2 = numpy.argmax(second_array)
      #  max_value = max(pxx[index - cutoff: pxx.size])
      #  index2 = index2 + index + cutoff
      #  axes2.plot([f[index2]], [pxx[index2]], 'r.', markersize = 10);
      #  self.second_mode = f[index2]
      #else:
      #  self.second_mode = 0
      
      r.html_result, r.text_summary, tr = self.sine_sweep_analysis(image_title)
      axes2.axvline(x=self.data.arg_value[0], color='r') # Line at first mode
      #axes2.axvline(x=self.data.arg_value[1], color='r') # Line at second mode
      axes2.set_xlim(0, 100)
      axes2.set_ylim(0, min(max_value + 10, max_value * 1.25))
      axes2.set_xlabel('Frequency')
      axes2.set_ylabel('Power')
      axes2.set_title('Velocity PSD')
      
      # Title
      fig.text(.35, .95, self.data.joint_name + ' SineSweep Test')

      if tr==True:
        r.result = TestResultRequest.RESULT_PASS
      else:
        r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
    
      stream = StringIO()
      plot.savefig(stream, format="png")
      image = stream.getvalue()
      
      p = Plot()
      r.plots.append(p)
      p.title = image_title
      p.image = image
      p.image_format = "png"
      self.result_service.call(r)
    except Exception, e:
      rospy.logerr('sine_sweep_plot caught exception, returning test failure.')
      rospy.logerr(traceback.format_exc())
      self.test_failed_service_call(traceback.format_exc())

  def sine_sweep_analysis(self, image_title): 
    # Check data array for mininum number of points
    num_pts = numpy.array(self.data.position).size

    if num_pts < 101: 
      tr = False
      error_msg = "<p>Not enough data points in position array. Minimum 101, found %d. Check encoder and amplitude of controller.</p>" % num_pts
      return (error_msg, "Not enough data points.", tr)
  
    # Parameters
    first_mode_param = self.data.arg_value[0]
    #second_mode_param = self.data.arg_value[1]
    mode_error_param = self.data.arg_value[2]

    html = ''
    summary = ''
    if abs(self.first_mode - first_mode_param)/first_mode_param > mode_error_param:
      html += "<H6>The first mode is incorrect: <b>FAIL</b>.</H6>"
      summary += "First mode: FAIL."
      tr=False
    else:
      html += "<H6>First mode: <b>OK</b>.</H6>"
      summary += "First mode: OK."
      tr=True
    html += "<p>First mode measured: %f, expected: %f.<br>" % (self.first_mode, first_mode_param)
    #html += "<br>Second mode measured: %f, expected %f.</p>" % (self.second_mode, second_mode_param)

    html += '<br><img src=\"IMG_PATH/%s.png\", width = 640, height = 480/><br>' % image_title

    html += '<hr size="2">\n'

    html += self.controller_params()

    return (html, summary, tr)

     
if __name__ == "__main__":
  try:
    app = AnalysisApp()
    rospy.spin()
  except Exception, e:
    traceback.print_exc()
    
  rospy.loginfo('Quitting Hysteresis Sinesweep Plot, shutting down node.')
