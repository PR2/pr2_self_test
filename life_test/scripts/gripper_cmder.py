#! /usr/bin/python
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

##\brief Commands a right gripper to open and close repeatedly

import roslib; roslib.load_manifest('life_test')
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

import random
import sys

from time import sleep

class LastMessage():
    def __init__(self, topic, msg_type):
        self.msg = None
        rospy.Subscriber(topic, msg_type, self.callback)

    def last(self):
        return self.msg

    def callback(self, msg):
        self.msg = msg

has_warned = False

def main():
    controller_name = rospy.myargv()[1]
    
    rospy.init_node('gripper_cmder')
    control_topic = '%s/command' % controller_name

    eff = 100

    grip_name = 'r_gripper_joint' if controller_name.startswith('r') else 'l_gripper_joint'

    turn_count = 0
    last_state = LastMessage('joint_states', JointState)
    try:
        pub = rospy.Publisher(control_topic, Float64)
        while not last_state.msg and not rospy.is_shutdown(): sleep(0.1)
        pub.publish(Float64(eff))
        while not rospy.is_shutdown():
            sleep(0.01)
            jnt_states = last_state.last()
            grip_idx = -1
            for index, name in enumerate(jnt_states.name):
                if name == grip_name:
                    grip_idx = index
                    break
            if grip_idx < 0:
                global has_warned
                if not has_warned:
                    rospy.logwarn("The joint %s was not found in the joints states" % grip_name)
                    has_warned = True

            if abs(jnt_states.velocity[grip_idx]) < 0.0005:
                turn_count += 1
            else:
                turn_count = 0

            if turn_count > 25:
                eff = -1 * eff
                pub.publish(Float64(eff))
                turn_count = 0

    except  KeyboardInterrupt, e:
        pass
    except:
        import traceback
        rospy.logerr(traceback.format_exc())
        traceback.print_exc()


if __name__ == '__main__':
    main()
