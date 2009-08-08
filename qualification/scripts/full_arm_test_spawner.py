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
import rospy, sys, time
import subprocess
from optparse import OptionParser

from std_msgs.msg import Float64
from mechanism_msgs.srv import SpawnController, KillController, SwitchController

spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
kill_controller = rospy.ServiceProxy('kill_controller', KillController)
switch_controller = rospy.ServiceProxy('switch_controller', SwitchController)

class SendMessageOnSubscribe(rospy.SubscribeListener):
    def __init__(self, msg):
        self.msg = msg
        print "Waiting for subscriber..."

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        peer_publish(self.msg)
        time.sleep(0.1)

spawned = None
prev_handler = None

def shutdown(sig, stackframe):
    global spawned
    if spawned is not None:
        for i in range(3):
            try:
                rospy.logout("Trying to kill %s" % spawned)
                kill_controller(spawned)
                rospy.logout("Succeeded in killing %s" % spawned)
                break
            except rospy.ServiceException:
                raise
                rospy.logerr("ServiceException while killing %s" % spawned)
    # We're shutdown.  Now invoke rospy's handler for full cleanup.
    if prev_handler is not None:
        prev_handler(signal.SIGINT,None)

def set_controller(controller, command):
    pub = rospy.Publisher(controller + '/set_command', Float64,
                              SendMessageOnSubscribe(Float64(command)))

def hold_arm(side, pan_angle):
    set_controller("%s_gripper_joint_hold" % side, float(0.0))
    set_controller("%s_wrist_roll_joint_hold" % side, float(0.0))
    set_controller("%s_wrist_flex_joint_hold" % side, float(1.0))
    set_controller("%s_forearm_roll_joint_hold" % side, float(0.0))
    set_controller("%s_elbow_flex_joint_hold" % side, float(-0.5))
    set_controller("%s_upper_arm_roll_joint_hold" % side, float(0.0))
    set_controller("%s_shoulder_lift_joint_hold" % side, float(0.5))
    set_controller("%s_shoulder_pan_joint_hold" % side, float(pan_angle))

def main():
    if len(sys.argv) < 4:
        print "Can't test arm, need <controller> <joint> <joint_name>"
        sys.exit(1)

    rospy.init_node('arm_test_spawner')

    # Pull side (l or r) from param server
    side = rospy.get_param("full_arm_test/side")

    # Override rospy's signal handling.  We'll invoke rospy's handler after
    # we're done shutting down.
    global prev_handler
    prev_handler = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, shutdown)

    try:
        controller = rospy.myargv()[1]
        joint = rospy.myargv()[2].replace('SIDE', side)
        joint_param_name = rospy.myargv()[3]

        holding = []

        rospy.wait_for_service('spawn_controller')

        rospy.loginfo('Raising torso')
        set_controller("torso_lift_joint_hold", float(0.30))

        rospy.loginfo('Holding arms')
        # Hold both arms in place
        hold_arm('r', -1.2)
        hold_arm('l', 1.2)
        time.sleep(1.5)

        rospy.loginfo('Killing joint controller %s_hold' % joint)
        # Kill controller for given joint
        kill_controller(joint + '_hold')

        rospy.loginfo('Spawning test controller')

        global spawned
        
        resp = spawn_controller(controller)
        if resp.ok != 0:
            spawned = controller
            rospy.loginfo("Spawned controller: %s" % controller)
        else:
            time.sleep(1) # give error message a chance to get out
            rospy.logerr("Failed to spawn %s" % controller)
            
        resp = switch_controller([spawned], [], 2)
        if resp.ok != 0:
            rospy.loginfo("Started controllers: %s" % spawned)
        else:
            rospy.logerr("Failed to start controller: %s" % spawned)
            
        rospy.spin()


if __name__ == '__main__':
    main()
