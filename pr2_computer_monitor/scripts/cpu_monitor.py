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
roslib.load_manifest('pr2_computer_monitor')

import rospy

import traceback
import threading
import sys, os, time
import subprocess

import socket

from diagnostic_msgs.msg import * 

def main():
    hds = []
    
    rospy.init_node('cpu_monitor', anonymous = True)
    
    pub = rospy.Publisher('/diagnostics', DiagnosticMessage)


    while not rospy.is_shutdown():
        msg = DiagnosticMessage()
        msg.status = []
        stat = DiagnosticStatus()
        stat.name = '%s CPU Temp Data' % socket.gethostname()
        stat.strings = []
        stat.values = []

        try:
            p = subprocess.Popen('ipmitool sdr type Temperature', 
                                 stdout = subprocess.PIPE,
                                 stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode

            stderr = stderr.replace('\n', '')
            
            stat.level = retcode
            
            if retcode == 0:
                stat.level = 0
                stat.message = 'OK'
            else:
                stat.level = 2
                stat.message = 'Exception'
                stat.strings.append(DiagnosticString(label = 'Error msg', value = stderr.replace('\n', '')))
                
            rospy.logerr('STDOUT')
            rospy.logerr(stdout)
            
            # Parse STDOUT for temp, device ID
            lines = stdout.split('\n')
            for ln in lines:
                lst = ln.split('|')
                stat.strings.append(DiagnosticString(label = lst[0], value = lst[len(lst) - 1]))
            

        except Exception, e:
            traceback.print_exc()
            
            stat.level = 2
            stat.message = 'Exception'
            stat.strings.append(DiagnosticString(label = 'Exception', value = str(e)))

        msg.status.append(stat)
        
        rospy.logerr('Publishing')
        pub.publish(msg)
        time.sleep(1.0)



if __name__ == '__main__':
    main()
            
