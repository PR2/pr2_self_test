#! /usr/bin/env python
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
##\brief Checks to make sure robot is calibrated, visualizer passes and all diagnostics are OK

PKG = 'qualification'
import roslib
roslib.load_manifest(PKG)

import rospy
from time import sleep


from diagnostic_msgs.msg import DiagnosticArray
from qualification.msg import Plot
from qualification.srv import *

from joint_qualification_controllers.msg import RobotData
from std_msgs.msg import Bool

import traceback

##\brief Sorts diagnostic messages by level, name
def level_cmp(a, b):
    if a._level == b._level:
        return cmp(a._name, b._name)

    return cmp(b._level, a._level)

##\brief Wrist diagnostics summary for easy to read messages
def _write_diag_summary(error_names, num_error, num_warn, num_stale):
    camera = False
    for name in error_names:
        if name.find('wge100') > 0:
            camera = True

    hokuyo = False
    for name in error_names:
        if name.find('hokuyo') > 0:
            hokuyo = True
            
    ethercat = error_names.count('EtherCAT Master') > 0
        
    if ethercat:
        return 'EtherCAT error. Check runstop. Motors or MCB\'s may have problem. '
    if camera:
        return 'Error in wge100 camera. Check camera connection. '
    if hokuyo:
        return 'Hokuyo error. Check connections. '

    return 'Diagnostics FAIL: %s errors, %s warnings, %s stale items. ' % (num_error, num_warn, num_stale)

class DiagnosticItem:
    def __init__(self, name, level, message):
        self._name = name
        self._level = level
        self._message = message
        self._update_time = rospy.get_time()

    def discard(self):
        if self._name.startswith('Controller'):
            return True

        if self._name == 'Realtime Control Loop':
            return True

        return False

    def check_stale(self):
        if rospy.get_time() - self._update_time > 3.0:
            self._level = 3

    def update_item(self, level, message):
        self._update_time = rospy.get_time()
        self._level = level
        self._message = message

##\brief Checks that all joints, actuators and diagnostics are OK
class RobotCheckout:
    def __init__(self):
        rospy.init_node('robot_checkout')
        
        self._motors_halted = True
        self.motors_topic = rospy.Subscriber("pr2_etherCAT/motors_halted", Bool, self.on_motor_cb)

        self._calibrated = False
        self._joints_ok = False
        
        self._has_robot_data = False
        self._has_visual_check = False

        #self._mutex = threading.Lock()
        self._messages = []
        self._name_to_diagnostic = {}
        
        self._joint_sum = 'No joint data. '
        self._act_sum = 'No actuator data. '
        self._joint_html = '<p>No joint data.</p><hr size="2">\n'
        self._act_html = '<p>No actuator data.</p><hr size="2">\n'

        self._visual_sum = 'No visual check. '
        self._visual_html = '<p>No response from visual verification!</p>\n'

        self._joints_ok = False
        self._acts_ok = False
        self._is_ok = False
        self._visual_ok = False

        self._timeout = True
        self._check_time = 0

        self.has_sent_result = False

        self._expected_actuators = rospy.get_param('~motors', None)
        if self._expected_actuators is None:
            rospy.logwarn('Not given list of expected actuators! Deprecation warning')

        self.robot_data = rospy.Subscriber('robot_checkout', RobotData, self.on_robot_data)
        self.result_srv = rospy.ServiceProxy('test_result', TestResult)

        self.diagnostics = rospy.Subscriber('diagnostics', DiagnosticArray, self.on_diagnostic_msg)
        self.visual_srv = rospy.Service('visual_check', ScriptDone, self.on_visual_check)
        
    def on_motor_cb(self, msg):
        self._motors_halted = msg.data
        
    def send_failure_call(self, caller = 'No caller', except_str = ''):
        if self.has_sent_result:
            rospy.logerr('Wanted to send failure call after result sent. Caller: %s. Exception:\n%s' % (caller, except_str))
            return

        r = TestResultRequest()
        r.html_result = '<p><b>Exception received during %s.</b></p>\n<p><b>Exception:</b> %s</p>\n' % (caller, except_str)
        r.text_summary = 'Exception during %s.' % caller
        r.plots = []
        r.result = r.RESULT_FAIL
        try:
            rospy.wait_for_service('test_result', 10)
            self.result_srv.call(r)
            self.has_sent_result = True
        except Exception, e:
            rospy.logerr('Caught exception sending failure call! %s' % traceback.format_exc())
            traceback.print_exc()
            

    def wait_for_data(self):
        try:
            rospy.logdebug('Robot checkout is waiting for diagnostics')
            # Wait at least 10 seconds for data
            for i in range(0, 20):
                if not rospy.is_shutdown():
                    sleep(0.5)
                    
            rospy.logdebug('Waiting for robot checkout controller')
            # Now start checking for robot data, done if we have it
            while not rospy.is_shutdown():
                # If the visualizer fails, abort early
                if self._has_visual_check and not self._visual_ok:
                    self.checkout_robot()
                    return
                if self._has_robot_data and self._has_visual_check:
                    self.checkout_robot()
                    return
                sleep(0.5)
 
        except KeyboardInterrupt:
            pass
        except Exception, e:
            self.send_failure_call('wait_for_data', traceback.format_exc())

    def on_diagnostic_msg(self, message):
        try:
            for stat in message.status:
                if stat.name not in self._name_to_diagnostic:
                    self._name_to_diagnostic[stat.name] = DiagnosticItem(stat.name, stat.level, stat.message)
                    
                else:
                    self._name_to_diagnostic[stat.name].update_item(stat.level, stat.message)
        except Exception, e:
            rospy.logerr('Caught exception processing diagnostic msg.\nEx: %s' % traceback.format_exc())
            self.send_failure_call('on_diagnostic_msg', traceback.format_exc())

    def on_visual_check(self, srv):
        rospy.logdebug('Got visual check')
        self._has_visual_check = True
        
        if srv.result == ScriptDoneRequest.RESULT_OK:
            self._visual_ok = True
            self._visual_sum = 'Visual: OK. '
            self._visual_html = '<p>Visual Verification Succeeded.</p>\n'
        else:
            self._visual_ok = False
            self._visual_html = '<p>Visual Verification Failed. '
            if srv.result == ScriptDoneRequest.RESULT_FAIL:
                self._visual_sum = 'Visual: FAIL. '
                self._visual_html += 'Operator recorded failure. Message: %s</p>\n' % srv.failure_msg
            else:
                self._visual_sum = 'Visual: ERROR. '
                self._visual_html += 'Visual verifier reported error!</p>\n'
            self._visual_html += '<p>Failure data:<br>%s</p>\n' % srv.failure_msg

        return ScriptDoneResponse()
    
            

    def on_robot_data(self, msg):
        rospy.logdebug('Got robot data service')
        self._has_robot_data = True
        
        self._timeout = msg.timeout
     
        self._check_time = msg.test_time
        self.joint_data(msg.joint_data)
        self.act_data(msg.actuator_data)
        
 
    def process_diagnostics(self):
        # Sort diagnostics by level
        my_diags = dict.values(self._name_to_diagnostic)
        
        diagnostics = []
        for diag in my_diags:
            if diag.discard():
                continue

            diag.check_stale()
            diagnostics.append(diag)

        diagnostics.sort(level_cmp)

        # Counts number by status
        stat_count = { 3: 0, 0: 0, 1: 0, 2: 0}

        level_dict = { 3: 'Stale', 0: 'OK', 1: 'Warn', 2: 'Error' }

        table = '<table border=\"1\" cellpadding=\"2\" cellspacing=\"0\">\n'
        table += '<tr><td><b>Name</b></td><td><b>Level</b></td><td><b>Message</b></td></tr>\n'

        error_names = []

        for diag in diagnostics:
            level = level_dict[diag._level]
            table += '<tr><td>%s</td><td>%s</td><td>%s</td></tr>\n' % (diag._name, level, diag._message)
            stat_count[diag._level] = stat_count[diag._level] + 1

            if level > 0:
                error_names.append(diag._name)

        table += '</table>\n'
            
        if stat_count[2] == 0 and stat_count[1] == 0 and stat_count[3] == 0 and len(diagnostics) > 0:
            summary = 'Diagnostics: OK. '
            self._is_ok = True
        else:
            if len(diagnostics) == 0:
                summary = 'No diagnostics received! '
                self._is_ok = False
            else:
                summary = _write_diag_summary(error_names, stat_count[2], stat_count[1], stat_count[3])
                self._is_ok = False
        
        html = '<p><b>Diagnostic Data</b></p><p>%s</p><br>\n' % summary 
        html += table

        return summary, html

    def checkout_robot(self):
        if self.has_sent_result:
            return

        try:
            html = '<p><b>Robot Checkout Test</b></p><br>\n'
                        
            diag_sum, diag_html = self.process_diagnostics()
            summary = ''
            
            if not self._has_robot_data:
                summary += 'No robot data received! '
                html += '<p><b>No robot data received!</b> CheckoutController might have had an error.</p>\n'
            
            if self._timeout and self._has_robot_data:
                summary += 'Test timed out. '
                html += '<p><b>Timeout in robot checkout controller! Check Time: %2f</b></p>\n' % self._check_time
            else:
                html += '<p>Time to complete check: %.3fs.</p>\n' % self._check_time

            summary += 'Data: ' + diag_sum + self._visual_sum + self._joint_sum + self._act_sum 

            if self._motors_halted:
                html += "<p>Motors halted, robot is not working.</p>\n"
                summary = "Motors halted. Check runstop. " + summary

            html += diag_html + '<hr size="2">\n'
            html += self._visual_html + '<hr size="2">\n'
            html += self._joint_html + '<hr size="2">\n'
            html += self._act_html + '<hr size="2">\n'

            
            r = TestResultRequest()
            r.html_result = html
            r.text_summary = summary
            
            if self._is_ok and self._visual_ok and self._joints_ok and \
                    self._acts_ok and not self._timeout and not self._motors_halted:
                r.result = r.RESULT_PASS
            else:
                r.result = r.RESULT_FAIL
                
            rospy.wait_for_service('test_result', 5)

            try:
                self.result_srv.call(r)
                self.has_sent_result = True
            except Exception, e:
                rospy.logerr('Caught exception sending OK service. %s' % traceback.format_exc())
        except Exception, e:
            self.send_failure_call('checkout_robot', traceback.format_exc())

    def act_data(self, act_datas):
        try:
            self._acts_ok = True

            found_acts = []

            html = ['<p><b>Actuator Data</b></p><br>\n']
            if self._expected_actuators is None:
                html.append('<p>WARNING: No list of expected actuators was given. In the future, this will cause a failure.</p>\n')
            html.append('<table border=\"1\" cellpadding=\"2\" cellspacing=\"0\">\n')
            html.append('<tr><td><b>Index</b></td><td><b>Name</b></td><td><b>ID</b></td><td><b>Expected</b></td></tr>\n')
            for act_data in act_datas:
                index = act_data.index
                name  = act_data.name
                id    = act_data.id

                expect = 'N/A'
                found_acts.append(act_data.name)
                # Compare found against expected
                if self._expected_actuators is not None:
                    expect = 'True'
                    if not act_data.name in self._expected_actuators:
                        expect = 'False'
                        self._acts_ok = False
                        

                html.append('<tr><td>%d</td><td>%s</td><td>%d</td><td>%s</td></tr>\n' % (index, name, id, expect))

            # Compare expected against found
            if self._expected_actuators is not None:
                for name in self._expected_actuators:
                    if not name in found_acts:
                        html.append('<tr><td>N/A</td><td>%s</td><td>Not Found</td><td>True</td></tr>\n' % (name))
                        self._acts_ok = False

            html.append('</table>\n')
            
            if self._acts_ok:
                self._act_sum = 'Acutators: OK. ' 
            else:
                self._act_sum = 'Actuators: FAIL! '

            self._act_html = ''.join(html)

        except Exception, e:
            self.send_failure_call('actuator_data', traceback.format_exc())
            

    def joint_data(self, jnt_datas):
        # Type doesn't work
        try:
            self._joints_ok = True
            self._calibrated = True   
            
            html = '<table border=\"1\" cellpadding=\"2\" cellspacing=\"0\">\n'
            html += '<tr><td><b>Index</b></td><td><b>Name</b></td><td><b>Type</b></td><td><b>Is Cal?</b></td><td><b>Has Safety?</b></td></tr>\n'
            for jnt_data in jnt_datas:
                id = jnt_data.index
                name = jnt_data.name
                type = jnt_data.type

                if jnt_data.is_cal == 1:
                    cal = '<div class=\"pass\">OK</div>'
                elif name.endswith('wheel_joint'):
                    cal = '<div class=\"pass\">Wheel</div>'
                elif name == 'base_joint':
                    cal = '<div class=\"pass\">Base</div>'
                elif name.find('finger') > 0:
                    cal = '<div class=\"pass\">Gripper</div>'
                elif name.find('accelerometer') > 0:
                    cal = '<div class=\"pass\">Gripper</div>'
                elif name.find('gripper_float_joint') > 0:
                    cal = '<div class=\"pass\">Gripper</div>'
                elif name.find('gripper_palm_joint') > 0:
                    cal = '<div class=\"pass\">Gripper</div>'
                elif name.find('gripper_tool_joint') > 0:
                    cal = '<div class=\"pass\">Gripper</div>'
                else:
                    cal = '<div class=\"warn\"><b>NO</b></div>'
                    self._joints_ok = False
                    self._calibrated = False
                    
                if jnt_data.has_safety:
                    safe = 'OK'
                elif type == 'Continuous': 
                    # Cont. joints don't have safety min/max
                    safe = '<div class=\"pass\">Continuous</div>'
                elif type == 'Fixed':
                    safe = '<div class=\"pass\">Fixed</div>'
                elif type =='Planar' and name=='base_joint':
                    safe = '<div class=\"pass\">Base Joint</div>'
                elif jnt_data.name.find('finger') > 0:
                    safe == '<div class=\"pass\">Finger Joint</div>'
                else:
                    safe = '<div class=\"warn\">NO</div>'
                    self._joints_ok = False
                    
                html += '<tr><td>%d</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>\n' % (id, name, type, cal, safe)

            html += '</table>\n'
                        
            if self._joints_ok:
                self._joint_sum = 'Joint states: OK. '
            else:
                if self._calibrated:
                    self._joint_sum = 'Joint states: FAIL. '
                else:
                    self._joint_sum = 'Joints states: FAIL, not all calibrated. '
            self._joint_html = '<p><b>Joint Data</b></p><p>%s</p><br>\n' % self._joint_sum
            self._joint_html += html
        except Exception, e:
            self.send_failure_call('joint_data', traceback.format_exc())

             
if __name__ == '__main__':
    try:
        checkout = RobotCheckout()
        sleep(1)
        checkout.wait_for_data()
        rospy.spin()
    except KeyboardInterrupt:
        pass
    except Exception, e:
        print 'Caught exception in robot checkout.\n%s' % traceback.format_exc()
        rospy.logerr('Robot checkout exception.\n%s' % traceback.format_exc())

