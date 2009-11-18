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

PKG = 'life_test'
import roslib
roslib.load_manifest(PKG)
import time
import random
import rospy
from std_msgs.msg import Float64

ROTATION_JOINT = 'fl_caster_rotation_joint'
TOPIC_PREFIX = 'caster_fl'
SPEED = 100
STEER_VEL = 100

class CasterCmd:
    def __init__(self):
        self.steer = STEER_VEL
        self.drive = SPEED

        self._count = 0

        self._left = True # Makes it alternate between starting left, right
 
    ##\brief 1/5 duty cycle on turn, regular sequence
    def update(self):
        if self._count == 0:
            if self._left:
                self.steer = -1 * STEER_VEL
            else:
                self.steer = STEER_VEL
            self.drive = 0
        elif self._count == 1:
            if self._left:
                self.steer = STEER_VEL
            else:
                self.steer = -1 * STEER_VEL
            self._left = not self._left
            self.drive = 0
        else:
            self.steer = 0
            if self._count % 2 == 0:
                self.drive = SPEED
            else:
                self.drive = -1 * SPEED

        self._count += 1
        if self._count >= 10:
            self._count = 0
            
                    
def main():
    rospy.init_node('caster_cmder')
    cmder = CasterCmd()
    pub_steer = rospy.Publisher("%s/steer" % TOPIC_PREFIX, Float64)
    pub_drive = rospy.Publisher("%s/drive" % TOPIC_PREFIX, Float64)
    pub_steer.publish(Float64(0.0))
    pub_drive.publish(Float64(0.0))

    my_interval = 1 / float(rospy.get_param('cycle_rate', 1.0))
    while not rospy.is_shutdown():
        pub_steer.publish(Float64(cmder.steer))
        pub_drive.publish(Float64(cmder.drive))
        
        cmder.update()
        time.sleep(random.uniform(0.9 * my_interval, 1.1 * my_interval))
        

if __name__ == '__main__':
    main()
