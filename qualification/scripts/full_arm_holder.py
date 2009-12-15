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

import roslib
roslib.load_manifest('qualification')
import rospy, sys, time
import subprocess
from optparse import OptionParser
from std_msgs.msg import Bool, Float64

class SendMessageOnSubscribe(rospy.SubscribeListener):
    def __init__(self, msg):
        self.msg = msg
        print "Waiting for subscriber..."

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        peer_publish(self.msg)
        time.sleep(0.1)


def set_controller(controller, command):
    pub = rospy.Publisher(controller + '/command', Float64,
                              SendMessageOnSubscribe(Float64(command)))

def hold_arm(side, pan_angle):
    #set_controller("%s_gripper_position_controller" % side, float(0.0))
    set_controller("%s_wrist_roll_position_controller" % side, float(0.0))
    set_controller("%s_wrist_flex_position_controller" % side, float(1.0))
    set_controller("%s_forearm_roll_position_controller" % side, float(0.0))
    set_controller("%s_elbow_flex_position_controller" % side, float(-0.5))
    set_controller("%s_upper_arm_roll_position_controller" % side, float(0.0))
    set_controller("%s_shoulder_lift_position_controller" % side, float(0.0))
    set_controller("%s_shoulder_pan_position_controller" % side, float(pan_angle))


def main():
    parser = OptionParser()
    parser.add_option('--wait-for', action="store", dest="wait_for_topic", default=None)

    options, args = parser.parse_args(rospy.myargv())

    rospy.init_node('arm_test_holder')
    pub_held = rospy.Publisher('arms_held', Bool, latch=True)


    try:
        if options.wait_for_topic:
            global result
            result = None
            def wait_for_topic_cb(msg):
                global result
                result = msg

            rospy.Subscriber(options.wait_for_topic, Bool, wait_for_topic_cb)
            started_waiting = rospy.get_time()
            
            # We might not have receieved any time messages yet
            warned_about_not_hearing_anything = False
            while not result and not rospy.is_shutdown():
                time.sleep(0.01)
                if not warned_about_not_hearing_anything:
                    if rospy.get_time() - started_waiting > 10.0:
                        warned_about_not_hearing_anything = True
                        rospy.logwarn("Full arm holder hasn't heard anything from its \"wait for\" topic (%s)" % \
                                         options.wait_for_topic)
            if result:
                while not result.data and not rospy.is_shutdown():
                    time.sleep(0.01)

        if not rospy.is_shutdown():
            rospy.loginfo('Raising torso')
            set_controller("torso_lift_position_controller", float(0.20))
            time.sleep(3)
            
        if not rospy.is_shutdown():
            rospy.loginfo('Holding arms')
            # Hold both arms in place
            hold_arm('r', -1.2)
            hold_arm('l', 1.2)
            time.sleep(1.5)
        
        if not rospy.is_shutdown():
            pub_held.publish(True)

        rospy.spin()
    except KeyboardInterrupt:
        pass
    except:
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
