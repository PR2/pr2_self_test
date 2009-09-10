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

# Author: Kevin Watts, original by Stuart Glaser

import roslib
roslib.load_manifest('life_test')
import time
import random
import rospy
from std_msgs.msg import Float64

ROTATION_JOINT = 'fl_caster_rotation_joint'
SPEED = 10
STEER_VEL = 5

class CasterCmd:
    def __init__(self):
        self.steer = STEER_VEL
        self.drive = SPEED

    def update(self):
        if self.steer > 0:
            self.steer = -1 * self.steer
            self.drive = 0
        elif self.steer < 0:
            self.steer = 0
            if random.randint(0, 1) == 1:
                self.drive = SPEED
            else:
                self.drive = -SPEED
        elif self.steer == 0:
            self.steer = STEER_VEL
            self.drive = 0
            
        
def main():
    rospy.init_node('caster_cmder')
    cmder = CasterCmd()
    pub_steer = rospy.Publisher("caster_fl/steer", Float64)
    pub_drive = rospy.Publisher("caster_fl/drive", Float64)
    pub_steer.publish(Float64(0.0))
    pub_drive.publish(Float64(0.0))

    rate = float(rospy.get_param('cycle_rate', 1.0))

    while not rospy.is_shutdown():
        # Steers the caster to be straight
        pub_steer.publish(Float64(cmder.steer))
        pub_drive.publish(Float64(cmder.drive))
        
        if random.randint(0, 3) != 3:
            cmder.update()
        time.sleep(1 / rate / 2)

if __name__ == '__main__':
    main()
