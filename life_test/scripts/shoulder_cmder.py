#!/usr/bin/python                                                               
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
#                                                                               

##\author Kevin Watts                                                           
##\brief Commands upperarm back and forth and rolls forearm

import roslib
roslib.load_manifest('life_test')

from time import sleep

import rospy
from std_msgs.msg import Float64
import random

def main():
    rospy.init_node('ua_cmder')
    
    pan_pos = rospy.Publisher('r_shoulder_pan_position_controller/command', Float64)
    lift_pos = rospy.Publisher('r_shoulder_lift_position_controller/command', Float64)
    roll_pos = rospy.Publisher('r_upper_arm_roll_position_controller/command', Float64)

    my_rate = rospy.Rate(float(rospy.get_param('cycle_rate', 1.0)))
    try:
        while not rospy.is_shutdown():
            pan_pos.publish(Float64(random.uniform(-2.0, 0.4)))
            lift_pos.publish(Float64(random.uniform(-0.40, 1.25)))
            roll_pos.publish(Float64(random.uniform(-3.70, 0.5)))

            my_rate.sleep()

    except KeyboardInterrupt, e:
        pass
    except Exception, e:
        import traceback
        rospy.logerr(traceback.format_exc())


if __name__ == '__main__':
    main()
