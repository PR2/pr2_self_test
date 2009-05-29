#! /usr/bin/python
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

# This script brings up an effort controller on your joint of choice
# and use a joystick to command it
#
# Author: Eric berger

import random
CONTROLLER_NAME = "quick_effort_controller_%08d" % random.randint(0,10**8-1)

import sys

import roslib
roslib.load_manifest('teleop_joint_effort')
import rospy
from std_msgs.msg import *
from joy.msg import Joy
from mechanism_control import mechanism
from robot_srvs.srv import SpawnController, KillController


def xml_for(joint):
    return "\
<controller name=\"%s\" type=\"JointEffortControllerNode\">\
<joint name=\"%s\" />\
</controller>" % (CONTROLLER_NAME, joint)

def main():
    rospy.init_node('teleop_joint_button', anonymous=True)
    rospy.wait_for_service('spawn_controller')
    spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
    kill_controller = rospy.ServiceProxy('kill_controller', KillController)

    params = {}
    params['joint'] = rospy.get_param("joint")
    params['max_effort'] = rospy.get_param("max_effort")
    params['joy_button_positive'] = rospy.get_param("joy_button_positive")
    params['joy_button_negative'] = rospy.get_param("joy_button_negative")

    print "Spawning effort controller %s"%params['joint']
    resp = spawn_controller(xml_for(params['joint']))
    if len(resp.ok) < 1 or not resp.ok[0]:
        print "Failed to spawn effort controller %s"%params['joint']
        sys.exit(1)

    pub = rospy.Publisher("/%s/set_command" % CONTROLLER_NAME, Float64)
    params['pub'] = pub

    rospy.Subscriber("joy", Joy, joy_callback, params)
    rospy.spin()
    kill_controller(CONTROLLER_NAME)

def joy_callback(data, params):
    command = data.buttons[params['joy_button_positive']] - data.buttons[params['joy_button_negative']]
    if(command > 1):
      command = 1;
    if(command < -1):
      command = -1;
    if(command != 0):
      effort = command * params['max_effort']
      params['pub'].publish(Float64(effort));

if __name__ == '__main__':
    main()
