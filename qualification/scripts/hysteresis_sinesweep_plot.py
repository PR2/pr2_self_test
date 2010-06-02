#!/usr/bin/env python
#
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

import rospy

from pr2_self_test_msgs.srv import TestResult, TestResultRequest
from qualification.analysis.hysteresis_analysis import *

from joint_qualification_controllers.msg import HysteresisData
from std_msgs.msg import Bool

import traceback

class AnalysisApp:
  def __init__(self):
    self.data_sent = False
    self._motors_halted = True
    rospy.init_node("test_plotter")
    self.data_topic = rospy.Subscriber("/test_data", HysteresisData, self._on_data)
    self.motors_topic = rospy.Subscriber("pr2_etherCAT/motors_halted", Bool, self.on_motor_cb)
    self.result_service = rospy.ServiceProxy('/test_result', TestResult)
    self.data = None
    
  def _on_data(self, msg):
    self.data = msg

  def has_data(self):
    return self.data is not None
    
  def on_motor_cb(self, msg):
    self._motors_halted = msg.data

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

      params = HysteresisParameters()
      params.joint_name = self.data.joint_name
      params.pos_effort = self.data.arg_value[1]
      params.neg_effort = self.data.arg_value[0]
      params.range_max  = self.data.arg_value[3]
      params.range_min  = self.data.arg_value[2]
      params.tolerance  = self.data.arg_value[7]
      params.sd_max     = self.data.arg_value[8]
      params.slope      = self.data.arg_value[9]
      params.timeout    = self.data.arg_value[5]
      params.velocity   = self.data.arg_value[4]
      params.max_effort = self.data.arg_value[6]

      params.p_gain     = self.data.arg_value[10]
      params.i_gain     = self.data.arg_value[11]
      params.d_gain     = self.data.arg_value[12]
      params.i_clamp    = self.data.arg_value[13]

      r.params = params.get_test_params()

      if self._motors_halted:
        r.result = TestResultRequest.RESULT_FAIL
        r.text_summary = "Motors halted. Check runstop. Hysteresis results are invalid."
        r.html_result = "<p>Motors halted during test. Check estop and verify that power is on.</p>\n"
        self.send_results(r)

      # Check the num points, timeout
      if len(self.data.position_up) < 20 or len(self.data.position_down) < 20:
        r.result = TestResultRequest.RESULT_FAIL
        r.text_summary = "Not enough data points, bad encoder or bad gains."
        
        error_msg = "<p>Not enough data points, hysteresis controller may have malfunctioned. Check diagnostics.</p>"
        error_msg += "<p>Up data points: %d, down points: %d</p>\n" % (len(self.data.position_up), len(self.data.position_down))
        error_msg += "<p>The encoder may be missing ticks, check encoder.<br>\n"
        error_msg += "Make sure mechanism is free to move and not trapped or constrained<br>\n"
        error_msg += "Make sure motors are not in safety lockout (check diagnostics)<br>\n"
        error_msg += "Check controller gains if problem persists on components of same type.</p>\n"
        error_msg += "<p>Test status: <b>FAIL</b>.</p>"
        error_msg += '<p align=center><b>Test Parameters</b></p>\n'
        
        r.html_result = error_msg
        self.send_results(r)
        return

      if self.data.arg_value[5] == 0:
        r.result = TestResultRequest.RESULT_FAIL
        r.text_summary = 'Hysteresis controller timed out. Check diagnostics.'
        r.html_result = '<p>Hysteresis controller timed out. Check diagnostics. Controller gains may be bad, or motors may be in safety lockout. Did the device pass the visualizer? Is it calibrated?</p><p>Test status: <b>FAIL</b>.</p>'
        r.html_result += '<p align=center><b>Test Parameters</b></p>\n'
        self.send_results(r)
        return

      # Compute the encoder travel
      min_encoder = min(min(numpy.array(self.data.position_up)), min(numpy.array(self.data.position_down)))
      max_encoder = max(max(numpy.array(self.data.position_up)), max(numpy.array(self.data.position_down)))

      if abs(max_encoder - min_encoder) < 0.005:
        r.result = TestResultRequest.RESULT_FAIL
        r.text_summary = 'Mechanism didn\'t move. Check diagnostics.'
        r.html_result = "<p>No travel of mechanism, hysteresis did not complete. Check controller gains and encoder.</p><p>Test status: <b>FAIL</b>.</p>"
        r.html_result += '<p align=center><b>Test Parameters</b></p>\n'
        self.send_results(r)
        return
      
      pos_data = HysteresisDirectionData(self.data.position_up, self.data.effort_up, self.data.velocity_up)
      neg_data = HysteresisDirectionData(self.data.position_down, self.data.effort_down, self.data.velocity_down)
      hyst_data = HysteresisTestData(pos_data, neg_data)
      
      # Make sure we have at least some points in both directions
      if pos_data.position.size < 20 or neg_data.position.size < 20:
        r.result = TestResultRequest.RESULT_FAIL
        r.text_summary = 'Incomplete data set. Check diagnostics.'
        r.html_result = "<p>Not enough data in one or more directions.</p><p>Test Status: <b>FAIL</b>.</p><p>Test status: <b>FAIL</b>.</p>"
        r.html_result += '<p align=center><b>Test Parameters</b></p>\n'
        self.send_results(r)
        return

      # Process HTML result
      range_result = range_analysis(params, hyst_data)
 
      if params.slope == 0:
        effort_result = effort_analysis(params, hyst_data)
      else:
        effort_result = regression_analysis(params, hyst_data)
 
      vel_result = velocity_analysis(params, hyst_data)
      vel_html = vel_result.html


      effort_plot = plot_effort(params, hyst_data)
      vel_plot = plot_velocity(params, hyst_data)
      r.plots = [effort_plot, vel_plot]

      html = '<img src=\"IMG_PATH/%s.png\", width = 640, height = 480/>' % effort_plot.title
      html += '<p><b>Legend:</b> Red - Average Effort. Yellow - Error Bars. Green - Expected Effort.</p>\n'
      html += '<p align=center><b>Range Data</b></p>' + range_result.html + '<hr size="2">\n'
      html += '<p align=center><b>Effort Data</b></p>' + effort_result.html + '<hr size="2">\n'

      html += '<p align=center><b>Velocity Data</b></p>' 
      html += '<img src=\"IMG_PATH/%s.png\", width = 640, height = 480/>' % vel_plot.title

      html += vel_html + '<hr size="2">\n'
      html += '<p align=center><b>Test Parameters</b></p>\n'


      r.html_result = html

      r.values.extend(effort_result.values)
      r.values.extend(range_result.values)
      r.values.extend(vel_result.values)
     
      if range_result.result and effort_result.result:
        r.text_summary = 'Hysteresis result: OK'
        r.result = TestResultRequest.RESULT_PASS
      elif not range_result.result:
        r.text_summary = 'Hysteresis range failed. Joint did not complete range of motion.'
        r.result = TestResultRequest.RESULT_FAIL
      else:
        r.text_summary = 'Hysteresis result: Questionable. ' + range_result.summary + ' ' + effort_result.summary
        r.result = TestResultRequest.RESULT_HUMAN_REQUIRED

      self.send_results(r)
    except Exception, e:
      rospy.logerr('hysteresis_plot caught exception, returning test failure.')
      rospy.logerr(traceback.format_exc())
      self.test_failed_service_call(traceback.format_exc())

      
if __name__ == "__main__":
  try:
    app = AnalysisApp()
    my_rate = rospy.Rate(5)
    while not rospy.is_shutdown() and not app.has_data():
      my_rate.sleep()
      
    if not rospy.is_shutdown():
      app.hysteresis_plot()
    rospy.spin()
  except KeyboardInterrupt:
    pass
  except Exception, e:
    traceback.print_exc()
    
