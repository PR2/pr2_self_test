#! /usr/bin/env python
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

# Author: Stuart Glaser

import roslib; roslib.load_manifest('teleop_spacenav')
import rospy, sys, math
from geometry_msgs.msg import Vector3

def print_usage(code = 0):
    print sys.argv[0], '<velocity topic> <rotational velocity topic>'
    sys.exit(code)


def sign(x):
    if x < 0:
        return -1
    return 1


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print_usage(1)

    topic = sys.argv[1]
    rot_topic = '-'
    if len(sys.argv) > 2:
        rot_topic = sys.argv[2]

    publisher = None
    rot_publisher = None
    if topic != '-':
        publisher = rospy.Publisher(topic + '/command', Vector3)
    if rot_topic != '-':
        rot_publisher = rospy.Publisher(rot_topic + '/set_command', Vector3)

    i = 0
    def spacenav_updated(msg):
        global i
        def t(x):
            #return x * 0.005
            return sign(x) * abs(x * 0.005) ** 1.5
        msg.x = t(msg.x)
        msg.y = t(msg.y)
        msg.z = t(msg.z)
        publisher.publish(msg)

    rot_i = 0
    def spacenav_rot_updated(msg):
        global rot_i
        def t(x):
            return sign(x) * 0.000002 * abs(x) ** 2
        msg.x = t(msg.x)
        msg.y = t(msg.y)
        msg.z = t(msg.z)
        rot_publisher.publish(msg)

    if publisher:
        print "Publishing translations"
        rospy.Subscriber("/spacenav/offset", Vector3, spacenav_updated)
    if rot_publisher:
        print "Publishing rotations"
        rospy.Subscriber("/spacenav/rot_offset", Vector3, spacenav_rot_updated)
    rospy.init_node('spacenav_teleop')
    rospy.spin()

'''
TODO:
reset to current position occasionally
'''
