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

##\brief Commands random positions on r_wrist flex, efforts on roll
##\author Kevin Watts

PKG = 'life_test'

import roslib; roslib.load_manifest(PKG)
import rospy

from std_msgs.msg import Float64

import random
from time import sleep

def main():
    rospy.init_node('wrist_cmder')
    pub_grip = rospy.Publisher("r_gripper_effort_controller/command", Float64)
    pub_flex = rospy.Publisher("r_wrist_flex_position_controller/command", Float64)
    pub_roll = rospy.Publisher("r_wrist_roll_effort_controller/command", Float64)
    
    effort_roll = float(rospy.get_param('roll_effort'))
    effort_grip = 0
        
    freq = float(rospy.get_param('cycle_rate'))

    try:
        my_rate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            position_flex = random.uniform(-1.8, -0.2)

            if random.randint(0, 1) == 1:
                effort_roll = effort_roll * -1
                
            pub_grip.publish(Float64(effort_grip))
            pub_flex.publish(Float64(position_flex))
            pub_roll.publish(Float64(effort_roll))

            my_rate.sleep()
    except KeyboardInterrupt:
        pass
    except:
        rospy.logerr('Wrist commander caught exception.\n%s' % traceback.format_exc())
    
if __name__ == '__main__':
    main()
