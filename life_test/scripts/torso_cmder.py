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
##\brief Commands torso to go up and down repeatedly

PKG = "life_test"
import roslib; roslib.load_manifest(PKG)

import sys
import rospy

from time import sleep
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from optparse import OptionParser

class LastMessage():
    def __init__(self, topic, msg_type):
        self.msg = None
        rospy.Subscriber(topic, msg_type, self.callback)

    def last(self):
        return self.msg

    def callback(self, msg):
        self.msg = msg
 
def main():
    parser = OptionParser()
    parser.add_option("-l", "--low", dest="low_only", 
                      help = "Keep in lower end only", action="store_true", 
                      default = False)
    parser.add_option("-s", "--slow", dest="slow",
                      help = "Move only every 5 minutes",
                      action="store_true", default=False)
                     
    
    (options, args) = parser.parse_args()

    rospy.init_node('torso_life_test')

    pub = rospy.Publisher("torso_lift_velocity_controller/command", Float64, latch = True)
    last_state = LastMessage('joint_states', JointState)
    turn_count = 0
    vel = 10.0
    pub.publish(Float64(vel))
    print 'Published', vel

    try:
        while not last_state.msg and not rospy.is_shutdown(): sleep(0.1)
        while not rospy.is_shutdown():
            sleep(0.01)
            jnt_states = last_state.last()
            torso_idx = -1
            for index, name in enumerate(jnt_states.name):
                if name == 'torso_lift_joint':
                    torso_idx = index
                    break
            if torso_idx < 0:
                rospy.logwarn("The joint %s was not found in the joints states" % 'torso_lift_joint')

            if abs(jnt_states.velocity[torso_idx]) < 0.002:
                turn_count += 1
            else:
                turn_count = 0

            if turn_count > 25:
                vel = -1 * vel
                turn_count = 0
                pub.publish(Float64(vel))

            if options.slow:
                sleep(5)

    except  KeyboardInterrupt, e:
        pass
    except:
        import traceback
        rospy.logerr(traceback.format_exc())
        traceback.print_exc()


if __name__ == '__main__':
    main()
