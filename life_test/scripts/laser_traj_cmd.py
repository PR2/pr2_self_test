#!/usr/bin/env python
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
##\brief Modified from pr2_mechanism_controllers/send_laser_traj_cmd_ms2 by Vijay Pradeep


PKG = "life_test"

import roslib; roslib.load_manifest(PKG) 

import sys
import os
import string

import rospy
from std_msgs import *

from pr2_msgs.msg import LaserTrajCmd
from pr2_msgs.srv import *
from time import sleep

def print_usage(exit_code = 0):
    print '''Usage:
    send_periodic_cmd.py [controller]
'''
    sys.exit(exit_code)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print_usage()

    cmd = LaserTrajCmd()
    controller   =    sys.argv[1]
    cmd.header   =    roslib.msg.Header(None, None, None)
    cmd.profile  = "linear"
    #cmd.pos      = [1.0, .26, -.26, -.7,   -.7,   -.26,   .26,   1.0, 1.0]
    d = .025
    #cmd.time     = [0.0, 0.4,  1.0, 1.1, 1.1+d,  1.2+d, 1.8+d, 2.2+d, 2.2+2*d]
    
    cmd.position = [1.5,  -.7, 1.5]
    cmd.time_from_start = [0.0, 0.5, 1.0]
    cmd.time_from_start = [rospy.Time.from_sec(x) for x in cmd.time_from_start]
    cmd.max_velocity =  5
    cmd.max_acceleration =  5

    print 'Sending Command to %s: ' % controller
    print '  Profile Type: %s' % cmd.profile
    print '  Pos: %s ' % ','.join(['%.3f' % x for x in cmd.position])
    print '  Time: %s' % ','.join(['%.3f' % x.to_sec() for x in cmd.time_from_start])
    print '  MaxRate: %f' % cmd.max_velocity
    print '  MaxAccel: %f' % cmd.max_acceleration

    rospy.wait_for_service(controller + '/set_traj_cmd')                                        
    s = rospy.ServiceProxy(controller + '/set_traj_cmd', SetLaserTrajCmd)
    resp = s.call(SetLaserTrajCmdRequest(cmd))

    print 'Command sent!'
    print '  Resposne: %f' % resp.start_time.to_seconds()
