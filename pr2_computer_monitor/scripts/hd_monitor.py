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

low_hd_level = 5
critical_hd_level = 1

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }

def check_hd_temp(hostname, hds):
    stat = DiagnosticStatus()
    stat.level = 0
    stat.name = "%s HD Temps" % hostname
    stat.strings = []
    stat.values = []

    for index, hd in enumerate(hds):
        stat.strings.append(DiagnosticString(label = 'Disk %d HW addr' % index, value = hd))
        try:
            p = subprocess.Popen('hddtemp %s' % hd, 
                                 stdout = subprocess.PIPE,
                                 stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode
            
            stdout = stdout.replace('\n', '')
            stderr = stderr.replace('\n', '')
            
            temp_level = retcode
            
            if retcode == 0:
                temp_level = 0
            else:
                temp_level = 2
                    
            # Parse STDOUT for temp, device ID
            lst = stdout.split(':')
            if len(lst) > 2:
                dev_id = lst[1]
                tmp = lst[2].strip()[:2] # Temp shows up as ' 40dC'
                
                if unicode(tmp).isnumeric():
                    temp = float(tmp)
                    if temp > 45:
                        temp_level = 1
                    if temp > 50:
                        temp_level = 2
                else:
                    temp = float(0.0)
                    temp_level = 2
                    
            else:
                dev_id = 'ERROR'
                temp = 0.0
                temp_level = 2
            

            stat.strings.append(DiagnosticString(label = 'Disk %d Device ID' % index, value = dev_id))
            stat.values.append(DiagnosticValue(label = 'Disk %d Temp' % index, value = temp))
            stat.strings.append(DiagnosticString(label = 'Disk %d Temp Status' % index, value = stat_dict[temp_level]))

            stat.level = max(stat.level, temp_level)
                
        except:
            traceback.print_exc()
            stat.values.append(DiagnosticValue(label = 'Disk %d Temp' % index, value = 0))
            stat.strings.append(DiagnosticString(label = 'Disk %d Temp Status' % index, value = stat_dict[2]))
            stat.strings.append(DiagnosticString(label = 'Disk %d Device ID' % index, value = 'No ID'))
            stat.level = 2
            stat.message = stat_dict[2]

    stat.message = stat_dict[stat.level]
    return stat

# Should get the HOME environment variable and stuff
# Could put this into separate status for each disk
def check_disk_usage(hostname, home_dir):
    stat = DiagnosticStatus()
    stat.level = 0
    stat.name = "%s HD Disk Usage" % hostname
    stat.strings = []
    stat.values = []

    try:
        p = subprocess.Popen(["df", "-P", "--block-size=1G", home_dir], 
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if (retcode == 0):
            stat.strings.append(DiagnosticString(label = 'Disk Space Reading', value = 'OK'))
            row_count = 0
            for row in stdout.split('\n'):
                if len(row.split()) < 2:
                    continue
                if not unicode(row.split()[1]).isnumeric() or float(row.split()[1]) < 10: # Ignore small drives
                    continue
                
                row_count += 1
                g_available = float(row.split()[-3])
                name = row.split()[0]
                size = float(row.split()[1])
                mount_pt = row.split()[-1]
                
                if (g_available > low_hd_level):
                    level = 0
                elif (g_available > critical_hd_level):
                    level = 1
                else:
                    level = 2
        
                stat.strings.append(DiagnosticString(label = 'Disk %d Name' % row_count, value = name))
                stat.values.append(DiagnosticString(label = 'Disk %d Available' % row_count, value = g_available))
                stat.values.append(DiagnosticString(label = 'Disk %d Size' % row_count, value = size))
                stat.strings.append(DiagnosticString(label = 'Disk %d Status' % row_count, value = stat_dict[level]))
                stat.strings.append(DiagnosticString(label = 'Disk %d Mount Point' % row_count, value = mount_pt))
                
                stat.level = max(level, stat.level)
                
        else:
            stat.strings.append(DiagnosticString(label = 'Disk Space Reading', value = 'Failed'))
            stat.level = 2
            

    except:
        traceback.print_exc()
        stat.strings.append(DiagnosticString(label = 'Disk Space Reading', value = 'Exception'))
        stat.level = 2

    stat.message = stat_dict[stat.level]
    return stat

        
# Need to check HD input/output too using iostat

def main():
    hds = []

    hostname = socket.gethostname()
    
    rospy.init_node('hd_monitor', anonymous = True)
    
    pub = rospy.Publisher('/diagnostics', DiagnosticMessage)

    home_dir = rospy.myargv()[1]
    hds = rospy.myargv()[2:]

    while not rospy.is_shutdown():
        msg = DiagnosticMessage()
        msg.status = []
        
        # Temperature
        msg.status.append(check_hd_temp(hostname, hds))
                           
        # Check disk usage
        msg.status.append(check_disk_usage(hostname, home_dir))

        #rospy.logerr('Publishing')
        pub.publish(msg)
        time.sleep(1.0)



if __name__ == '__main__':
    main()
            
