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

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }

def check_cpu_temp(hostname):
    stat = DiagnosticStatus()
    stat.name = '%s CPU Temp Data' % hostname
    stat.strings = []
    stat.values = []

    try:
        p = subprocess.Popen('ipmitool sdr type Temperature', 
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode
                
        stat.level = retcode
        
        if retcode == 0:
            stat.level = 0
            stat.message = 'OK'
        else:
            stat.level = 2
            stat.message = 'Bad return code'
            stat.strings.append(DiagnosticString(label = 'Error code: %d' % retcode, value = stderr.replace('\n', '')))
                
                        
        # Parse STDOUT for temp, device ID
        rospy.logerr(stdout)
        lines = stdout.split('\n')
        if len(lines) < 2:
            stat.strings.append(DiagnosticString(label = 'ipmitool status', value = 'No output'))
            stat.message = 'No response'
            stat.level = 1
        for ln in lines:
            
            lst = ln.split('|')
            if lst[-1].endswith('degrees C'):
                rospy.logerr(lst[-1])
                tmp = lst[-1].rstrip('degrees C').lstrip()
                if unicode(tmp).isnumeric():
                    rospy.logerr('temp %s' % tmp)
                    stat.values.append(DiagnosticValue(label = lst[0], value = float(tmp)))
            else:
                stat.strings.append(DiagnosticString(label = lst[0], value = lst[-1]))
            

    except Exception, e:
        traceback.print_exc()
        
        stat.level = 2
        stat.message = 'Exception'
        stat.strings.append(DiagnosticString(label = 'Exception', value = str(e)))
        
    return stat




def check_nfs_stat(hostname):
    stat = DiagnosticStatus()
    stat.name = '%s NFS I/O' % hostname
    stat.level = 0
    stat.message = 'OK'
    stat.strings = []
    stat.values = []

    try:
        p = subprocess.Popen('iostat -n',
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode
        
        for index, row in enumerate(stdout.split('\n')):
            if index < 3:
                continue

            lst = row.split()
            if len(lst) < 7:
                continue

            file_sys = lst[0]
            read_blk = float(lst[1])
            write_blk = float(lst[2])
            read_blk_dir = float(lst[3])
            write_blk_dir = float(lst[4])
            r_blk_srv = float(lst[5])
            w_blk_srv = float(lst[6])

            stat.values.append(DiagnosticValue(
                    label = '%s Read Blks/s' % file_sys, value=read_blk))
            stat.values.append(DiagnosticValue(
                    label = '%s Write Blks/s' % file_sys, value=write_blk))
            stat.values.append(DiagnosticValue(
                    label = '%s Read Blk dir/s' % file_sys, value=read_blk_dir))
            stat.values.append(DiagnosticValue(
                    label = '%s Write Blks dir/s' % file_sys, value=write_blk_dir))
            stat.values.append(DiagnosticValue(
                    label = '%s Read Blks srv/s' % file_sys, value=r_blk_srv))
            stat.values.append(DiagnosticValue(
                    label = '%s Write Blks srv/s' % file_sys, value=w_blk_srv))
        
    except Exception, e:
        traceback.print_exc()
        stat.level = 2
        stat.message = 'Exception'
        stat.strings.append(DiagnosticString(label = 'Exception', value = str(e)))
        
    return stat

def check_uptime():
    level = 0
    vals = []
    string = DiagnosticString(label = 'Load Average Status', value = 'Error')
    
    try:
        p = subprocess.Popen('uptime', stdout = subprocess.PIPE, 
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        upvals = stdout.split()
        load1 = float(upvals[-3].rstrip(','))
        load5 = float(upvals[-2].rstrip(','))
        load15 = float(upvals[-1])
        num_users = float(upvals[-7])

        vals.append(DiagnosticValue(label = '1 min Load Average', value = load1))
        vals.append(DiagnosticValue(label = '5 min Load Average', value = load5))
        vals.append(DiagnosticValue(label = '15 min Load Average', value = load15))
        vals.append(DiagnosticValue(label = 'Number of Users', value = num_users))

        if load1 > 25 or load5 > 18:
            level = 1
        if load1 > 35 or load5 > 25 or load15 > 20:
            level = 2

        string = DiagnosticString(label = 'Load Average Status', value = stat_dict[level])

    except Exception, e:
        rospy.logerr(traceback.print_exc())
        level = 2
        DiagnosticString(label = 'Load Average Status', value = str(e))
        
    return level, vals, string

def check_memory():
    values = []
    str = DiagnosticString(label = 'Memory Status', value = 'Exception')
    level = 2

    try:
        p = subprocess.Popen('free -m',
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode
                
        rows = stdout.split('\n')
        data = rows[1].split()
        total_mem = float(data[1])
        used_mem = float(data[2])
        free_mem = float(data[3])

        values.append(DiagnosticValue(label = 'Total Memory', value = total_mem))
        values.append(DiagnosticValue(label = 'Used Memory', value = used_mem))
        values.append(DiagnosticValue(label = 'Free Memory', value = free_mem))

        level = 0
        if free_mem < 250:
            level = 1
        if free_mem < 100:
            level = 2

        str = DiagnosticString(label = 'Total Memory', value = stat_dict[level])
        
    except Exception, e:
        rospy.logerr(traceback.format_exc())
        
    return level, values, str

# Use mpstat
def check_usage(hostname):
    stat = DiagnosticStatus()
    stat.name = '%s CPU Usage' % hostname
    stat.level = 0
    stat.strings = []
    stat.values = []

    try:
        p = subprocess.Popen('mpstat -P ALL 1 1',
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode
    
        for index, row in enumerate(stdout.split('\n')):
            if index < 3:
                continue
            
            lst = row.split()
            if len(lst) < 10:
                continue

            if lst[0] == 'Average:':
                continue

            cpu_name = lst[2]
            if cpu_name == 'all':
                cpu_name == 'ALL'
            idle = float(lst[-2])
            user = float(lst[3])
            system = float(lst[5])
            
            stat.values.append(DiagnosticValue(label = 'CPU %s User' % cpu_name, value = user))
            stat.values.append(DiagnosticValue(label = 'CPU %s System' % cpu_name, value = system))
            stat.values.append(DiagnosticValue(label = 'CPU %s Idle' % cpu_name, value = idle))

            cpu_lvl = 0
            if user > 75.0:
                cpu_lvl = 1
            if user > 90.0:
                cpu_lvl = 2
            stat.level = max(stat.level, cpu_lvl)

            stat.strings.append(DiagnosticString(label = 'CPU %s Status' % cpu_name, value = stat_dict[cpu_lvl]))
            
        # Check uptime
        uptime_level, up_vals, up_str = check_uptime()
        stat.values.extend(up_vals)
        stat.strings.append(up_str)
        stat.level = max(stat.level, uptime_level)

        # Check memory
        mem_level, mem_vals, mem_str = check_memory()
        stat.values.extend(mem_vals)
        stat.strings.append(mem_str)
        stat.level = max(stat.level, mem_level)
        
        stat.message = stat_dict[stat.level]

    except Exception, e:
        stat.level = 2
        stat.message = 'Exception'
        stat.strings.append(DiagnosticString(label = 'mpstat Exception', value = str(e)))

    return stat


def main():
    hds = []

    hostname = socket.gethostname()
    
    rospy.init_node('cpu_monitor', anonymous = True)
    
    pub = rospy.Publisher('/diagnostics', DiagnosticMessage)

    cpu_temp = len(rospy.myargv()) < 2 and rospy.myargv()[1] != '--no_cpu_temp'

    while not rospy.is_shutdown():
        msg = DiagnosticMessage()
        msg.status = []
         
        # Temperature, don't check if user says not to
        if cpu_temp:
            msg.status.append(check_cpu_temp(hostname))
        
        # Usage, memory, and load average
        msg.status.append(check_usage(hostname))

        # NFS status
        msg.status.append(check_nfs_stat(hostname))
        
        pub.publish(msg)
        time.sleep(1.0)



if __name__ == '__main__':
    main()
            
