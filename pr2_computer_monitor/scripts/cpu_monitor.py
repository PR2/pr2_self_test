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
from threading import Timer
import sys, os, time
from time import sleep
import subprocess
import string

import socket

from diagnostic_msgs.msg import * 

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }

# Output entire IPMI data set
def check_ipmi():
    diag_strs = []
    diag_vals = []
    diag_msgs = []
    diag_level = 0

    try:
        p = subprocess.Popen('sudo ipmitool sdr',
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode
                        
        if retcode != 0:
            diag_level = 2
            diag_msg = [ 'ipmitool Error' ]
            diag_strs = [ DiagnosticString(label = 'IPMI Error', value = stderr) ]
            return diag_strs, diag_vals, diag_msgs, diag_level

        lines = stdout.split('\n')
        if len(lines) < 2:
            diag_strs = [ DiagnosticString(label = 'ipmitool status', value = 'No output') ]
            diag_msgs = [ 'No response' ]
            diag_level = 2
            return diag_strs, diag_vals, diag_msgs, diag_level

        for ln in lines:
            if len(ln) < 2:
                continue

            words = ln.split('|')
            name = words[0].strip()
            ipmi_val = words[1].strip()
            stat_byte = words[1].strip()

            # CPU temps
            if words[0].startswith('CPU') and words[0].strip().endswith('Temp'):
                if words[1].strip().endswith('degrees C'):
                    tmp = ipmi_val.rstrip(' degrees C').lstrip()
                    if unicode(tmp).isnumeric():
                        temperature = float(tmp)
                        diag_vals.append(DiagnosticValue(label = name, value = temperature))

                        cpu_name = name.split()[0]
                        if temperature >= 80 and temperature < 85:
                            diag_level = max(diag_level, 1)
                            if diag_msgs.count('CPU Hot') == 0:
                                diag_msgs.append('CPU Warm')
                        if temperature >= 85:
                            diag_level = max(diag_level, 2)
                            diag_msgs.append('CPU Hot')                                
                            # Don't keep CPU Warm in list if CPU is hot
                            if diag_msgs.count('CPU Warm') > 0:
                                idx = diag_msgs.index('CPU Warm')
                                diag_msgs.pop(idx)

                            
                else:
                    diag_strs.append(DiagnosticString(label = name, value = words[1]))


            # MP, BP, FP temps
            if name == 'MB Temp' or name == 'BP Temp' or name == 'FP Temp':
                if ipmi_val.endswith('degrees C'):
                    tmp = ipmi_val.rstrip(' degrees C').lstrip()
                    if unicode(tmp).isnumeric():
                        temperature = float(tmp)
                        diag_vals.append(DiagnosticValue(label = name, value = temperature))
                        
                        dev_name = name.split()[0]
                        if temperature >= 60 and temperature < 70:
                            diag_level = max(diag_level, 1)
                            diag_msgs.append('%s Warm' % dev_name)
                        if temperature >= 70:
                            diag_level = max(diag_level, 2)
                            diag_msgs.append('%s Hot' % dev_name)

                else:
                    diag_strs.append(DiagnosticString(label = name, value = ipmi_val))
        
            # CPU fan speeds
            if (name.startswith('CPU') and name.endswith('Fan')) or name == 'MB Fan':
                if ipmi_val.endswith('RPM'):
                    rpm = ipmi_val.rstrip(' RPM').lstrip()
                    if unicode(rpm).isnumeric():
                        rpm = float(rpm)
                        diag_vals.append(DiagnosticValue(label = name, value = rpm))
                    else:
                        diag_strs.append(DiagnosticString(label = name, value = ipmi_val))

            # If CPU is hot we get an alarm from ipmitool, report that too
            if name.startswith('CPU') and name.endswith('hot'):
                if ipmi_val == '0x01':
                    diag_strs.append(DiagnosticString(label = name, value = 'OK'))
                else:
                    diag_strs.append(DiagnosticString(label = name, value = 'Hot'))
                    diag_level = max(diag_level, 2)
                    diag_msgs.append('CPU Hot Alarm')

    except Exception, e:
        diag_strs.append(DiagnosticString(label = 'Exception', value = traceback.format_exc()))
        diag_level = 2
        diag_msgs.append('Exception')

    return diag_strs, diag_vals, diag_msgs, diag_level
        

# Check core temps 
# Use 'find /sys -name temp1_input' to find cores
# Read from every core, divide by 1000
def check_core_temps(sys_temp_strings):
    diag_vals = []
    diag_strs = []
    diag_level = 0
    diag_msgs = []
    
    for index, temp_str in enumerate(sys_temp_strings):
        if len(temp_str) < 5:
            continue
        
        cmd = 'cat %s' % temp_str
        p = subprocess.Popen(cmd, stdout = subprocess.PIPE, 
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            diag_level = 2
            diag_msg = [ 'Core Temp Error' ]
            diag_strs = [ DiagnosticString(label = 'Core Temp Error', value = stderr), DiagnosticString(label = 'Output', value = stdout) ]
            return diag_strs, diag_vals, diag_msgs, diag_level
  
        tmp = stdout.strip()
        if unicode(tmp).isnumeric():
            temp = float(tmp) / 1000
            diag_vals.append(DiagnosticValue(label = 'Core %d Temp' % index, value = temp))

            if temp >= 85 and temp < 90:
                diag_level = max(diag_level, 1)
                diag_msgs.append('Warm')
            if temp >= 90:
                diag_level = max(diag_level, 2)
                diag_msgs.append('Hot')

        else:
            diag_strs.append(DiagnosticString(label = 'Core %s Temp' % index, value = tmp))

    return diag_strs, diag_vals, diag_msgs, diag_level

## Checks clock speed from reading from CPU info
def check_clock_speed(enforce_speed):
    strs = []
    vals = []
    msgs = []
    lvl = 0

    try:
        p = subprocess.Popen('cat /proc/cpuinfo | grep MHz', 
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            lvl = 2
            msgs = [ 'Clock speed error' ]
            strs = [ DiagnosticString(label = 'Clock speed error', value = stderr), 
                          DiagnosticString(label = 'Output', value = stdout) ]
            
            return (strs, vals, msgs, lvl)

        for index, ln in enumerate(stdout.split('\n')):
            words = ln.split(':')
            if len(words) < 2:
                continue

            speed = words[1].strip().split('.')[0]
            if unicode(speed).isnumeric():
                mhz = float(speed)
                vals.append(DiagnosticValue(label = 'Core %d Speed' % index, value = mhz))
                
                if mhz < 2240 and mhz > 2150:
                    lvl = max(lvl, 1)
                if mhz <= 2150:
                    lvl = max(lvl, 2)

            else:
                lvl = max(lvl, 2)
                strs.append(DiagnosticString(label = 'Core %d Speed' % index, value = speed))

        if not enforce_speed:
            lvl = 0

        if lvl == 1 and enforce_speed:
            msgs = [ 'Core slowing' ]
        elif lvl == 2 and enforce_speed:
            msgs = [ 'Core throttled' ]

    except Exception, e:
        rospy.logerr(traceback.format_exc())
        lvl = 2
        msgs.append('Exception')
        strs.append(DiagnosticString(label = 'Exception', value = traceback.format_exc()))

    return strs, vals, msgs, lvl
                    

# Add msgs output, too
def check_uptime():
    level = 0
    vals = []
    str = DiagnosticString(label = 'Load Average Status', value = 'Error')
    
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

        str = DiagnosticString(label = 'Load Average Status', value = stat_dict[level])

    except Exception, e:
        rospy.logerr(traceback.format_exc())
        level = 2
        str = DiagnosticString(label = 'Load Average Status', value = str(e))
        
    return level, vals, str

# Add msgs output
def check_memory():
    values = []
    str = DiagnosticString(label = 'Memory Status', value = 'Exception')
    level = 2
    msg = ''


    mem_dict = { 0: 'OK', 1: 'Low', 2: 'Very Low' }

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
        if free_mem < 10:
            level = 1
        if free_mem < 5:
            level = 2

        str = DiagnosticString(label = 'Total Memory', value = mem_dict[level])
    
        msg = mem_dict[level]
    except Exception, e:
        rospy.logerr(traceback.format_exc())
        msg = 'Memory Error'
    
    return level, values, str

# Use mpstat
def check_mpstat():
    strs = []
    vals = []
    mp_level = 0
    
    load_dict = { 0: 'OK', 1: 'High Load', 2: 'Very High Load' }

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

            if lst[0].startswith('Average'):
                continue

            cpu_name = lst[2]
            if cpu_name == 'all':
                cpu_name == 'ALL'
            idle = float(lst[-2])
            user = float(lst[3])
            nice = float(lst[4])
            system = float(lst[5])
            
            vals.append(DiagnosticValue(label = 'CPU %s User' % cpu_name, value = user))
            vals.append(DiagnosticValue(label = 'CPU %s Nice' % cpu_name, value = nice))
            vals.append(DiagnosticValue(label = 'CPU %s System' % cpu_name, value = system))
            vals.append(DiagnosticValue(label = 'CPU %s Idle' % cpu_name, value = idle))

            core_level = 0
            if user + nice > 75.0:
                core_level = 1
            if user + nice> 90.0:
                core_level = 2

            strs.append(DiagnosticString(label = 'CPU %s Status' % cpu_name, value = load_dict[core_level]))
            mp_level = max(mp_level, core_level)
            
    except Exception, e:
        mp_level = 2
        strs.append(DiagnosticString(label = 'mpstat Exception', value = str(e)))

    return mp_level, vals, strs

## Returns names for core temperature files
## Returns list of names, each name can be read like file
def get_core_temp_names():
    temp_vals = []
    try:
        p = subprocess.Popen('find /sys/devices -name temp1_input', 
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            rospy.logerr('Error find core temp locations: %s' % stderr)
            return []
        
        for ln in stdout.split('\n'):
            temp_vals.append(ln.strip())
        
        return temp_vals
    except:
        rospy.logerr('Exception finding temp vals: %s' % traceback.format_exc())
        return []

def update_status_stale(stat, last_update_time):
    time_since_update = rospy.get_time() - last_update_time

    level = stat.level
    stale_status = 'OK'
    if time_since_update > 20:
        stale_status = 'Lagging'
        level = max(level, 1)
    if time_since_update > 35:
        stale_status = 'Stale'
        level = max(level, 2)
        
    stat.strings.pop(0)
    stat.values.pop(0)
    stat.strings.insert(0, DiagnosticString(label = 'Update Status', value = stale_status))
    stat.values.insert(0, DiagnosticValue(label = 'Time Since Update', value = time_since_update))

class CPUMonitor():
    def __init__(self, hostname):
        rospy.init_node('cpu_monitor_%s' % hostname)
        
        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticMessage)

        self._mutex = threading.Lock()

        self._check_ipmi = rospy.get_param('check_ipmi_tool', True)
        self._enforce_speed = rospy.get_param('enforce_clock_speed', True)
        
        # Get temp_input files
        self._temp_vals = get_core_temp_names()

        # CPU stats
        self._temp_stat = DiagnosticStatus()
        self._temp_stat.name = '%s CPU Temperature' % hostname
        self._temp_stat.level = 2
        self._temp_stat.hardware_id = hostname
        self._temp_stat.message = 'No Data'
        self._temp_stat.strings = [ DiagnosticString(label = 'Update Status', value = 'No Data' )]
        self._temp_stat.values = [ DiagnosticValue(label = 'Time Since Last Update', value = 100000 )]

        self._usage_stat = DiagnosticStatus()
        self._usage_stat.name = '%s CPU Usage' % hostname
        self._usage_stat.level = 2
        self._usage_stat.hardware_id = hostname
        self._usage_stat.message = 'No Data'
        self._usage_stat.strings = [ DiagnosticString(label = 'Update Status', value = 'No Data' )]
        self._usage_stat.values = [ DiagnosticValue(label = 'Time Since Last Update', value = 100000 )]

        self._nfs_stat = DiagnosticStatus()
        self._nfs_stat.name = '%s NFS I/O' % hostname
        self._nfs_stat.level = 2
        self._nfs_stat.hardware_id = hostname
        self._nfs_stat.message = 'No Data'
        self._nfs_stat.strings = [ DiagnosticString(label = 'Update Status', value = 'No Data' )]
        self._nfs_stat.values = [ DiagnosticValue(label = 'Time Since Last Update', value = 100000 )]

        self._last_temp_time = 0
        self._last_usage_time = 0
        self._last_nfs_time = 0
        self._last_publish_time = 0

        self._temps_timer = None
        self._usage_timer = None
        self._nfs_timer = None
        self._publish_timer = None
        ##@todo Need wireless stuff, at some point, put NFS in usage status
        
        # Start checking everything
        self.check_temps()
        self.check_nfs_stat()
        self.check_usage()

    ## Must have the lock to cancel everything
    def cancel_timers(self):
        if self._temps_timer:
            self._temps_timer.cancel()

        if self._nfs_timer:
            self._nfs_timer.cancel()

        if self._usage_timer:
            self._usage_timer.cancel()

    def check_nfs_stat(self):
        if rospy.is_shutdown():
            self._mutex.acquire()
            self.cancel_timers()
            self._mutex.release()
            return

        nfs_level = 0
        msg = 'OK'
        strs = [ DiagnosticString(label = 'Update Status', value = 'OK' )]
        vals = [ DiagnosticValue(label = 'Time Since Last Update', value = 0 )]

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
                
                vals.append(DiagnosticValue(
                        label = '%s Read Blks/s' % file_sys, value=read_blk))
                vals.append(DiagnosticValue(
                        label = '%s Write Blks/s' % file_sys, value=write_blk))
                vals.append(DiagnosticValue(
                        label = '%s Read Blk dir/s' % file_sys, value=read_blk_dir))
                vals.append(DiagnosticValue(
                        label = '%s Write Blks dir/s' % file_sys, value=write_blk_dir))
                vals.append(DiagnosticValue(
                        label = '%s Read Blks srv/s' % file_sys, value=r_blk_srv))
                vals.append(DiagnosticValue(
                        label = '%s Write Blks srv/s' % file_sys, value=w_blk_srv))
                
        except Exception, e:
            rospy.logerr(traceback.format_exc())
            nfs_level = 2
            msg = 'Exception'
            strings.append(DiagnosticString(label = 'Exception', value = str(e)))
            
        self._mutex.acquire()
        
        self._nfs_stat.level = nfs_level
        self._nfs_stat.message = msg
        self._nfs_stat.strings = strs
        self._nfs_stat.values = vals
        
        self._last_nfs_time = rospy.get_time()

        if not rospy.is_shutdown():
            self._nfs_timer = threading.Timer(5.0, self.check_nfs_stat)
            self._nfs_timer.start()
        else:
            self.cancel_timers()

        self._mutex.release()

    ## Call every 10sec at minimum
    def check_temps(self):
        if rospy.is_shutdown():
            self._mutex.acquire()
            self.cancel_timers()
            self._mutex.release()
            return

        diag_strs = [ DiagnosticString(label = 'Update Status', value = 'OK' ) ]
        diag_vals = [ DiagnosticValue(label = 'Time Since Last Update', value = 0 ) ]
        diag_msgs = []
        diag_level = 0

        if self._check_ipmi:
            ipmi_strs, ipmi_vals, ipmi_msgs, ipmi_level = check_ipmi()
            diag_strs.extend(ipmi_strs)
            diag_vals.extend(ipmi_vals)
            diag_msgs.extend(ipmi_msgs)
            diag_level = max(diag_level, ipmi_level)

        core_strs, core_vals, core_msgs, core_level = check_core_temps(self._temp_vals)
        diag_strs.extend(core_strs)
        diag_vals.extend(core_vals)
        diag_msgs.extend(core_msgs)
        diag_level = max(diag_level, core_level)

        clock_strs, clock_vals, clock_msgs, clock_level = check_clock_speed(self._enforce_speed)
        diag_strs.extend(clock_strs)
        diag_vals.extend(clock_vals)
        diag_msgs.extend(clock_msgs)
        diag_level = max(diag_level, clock_level)

        diag_log = set(diag_msgs)
        if len(diag_log) > 0:
            message = string.join(diag_log, ', ')
        else:
            message = stat_dict[diag_level]

        self._mutex.acquire()
        self._last_temp_time = rospy.get_time()
        
        self._temp_stat.level = diag_level
        self._temp_stat.message = message
        self._temp_stat.strings = diag_strs
        self._temp_stat.values = diag_vals

        if not rospy.is_shutdown():
            self._temp_timer = threading.Timer(5.0, self.check_temps)
            self._temp_timer.start()
        else:
            self.cancel_timers()

        self._mutex.release()

    def check_usage(self):
        if rospy.is_shutdown():
            self._mutex.acquire()
            self.cancel_timers()
            self._mutex.release()
            return 

        diag_level = 0
        diag_strs = [ DiagnosticString(label = 'Update Status', value = 'OK' ) ]
        diag_vals = [ DiagnosticValue(label = 'Time Since Last Update', value = 0 )]
        
        # Check mpstat
        mp_level, mp_vals, mp_strs = check_mpstat()
        diag_vals.extend(mp_vals)
        diag_strs.extend(mp_strs)
        diag_level = max(diag_level, mp_level)
            
        # Check uptime
        uptime_level, up_vals, up_str = check_uptime()
        diag_vals.extend(up_vals)
        diag_strs.append(up_str)
        diag_level = max(diag_level, uptime_level)
        
        # Check memory
        mem_level, mem_vals, mem_str = check_memory()
        diag_vals.extend(mem_vals)
        diag_strs.append(mem_str)
        diag_level = max(diag_level, mem_level)
            

        # Update status
        self._mutex.acquire()
        self._last_usage_time = rospy.get_time()
        self._usage_stat.level = diag_level
        self._usage_stat.values = diag_vals
        self._usage_stat.strings = diag_strs
        
        self._usage_stat.message = stat_dict[diag_level]

        if not rospy.is_shutdown():
            self._usage_timer = threading.Timer(5.0, self.check_usage)
            self._usage_timer.start()
        else:
            self.cancel_timers()

        self._mutex.release()


    def publish_stats(self):
        self._mutex.acquire()
                
        # Update everything with last update times
        update_status_stale(self._temp_stat, self._last_temp_time)
        update_status_stale(self._usage_stat, self._last_usage_time)
        update_status_stale(self._nfs_stat, self._last_nfs_time)

        msg = DiagnosticMessage()
        msg.status.append(self._temp_stat)
        msg.status.append(self._usage_stat)
        msg.status.append(self._nfs_stat)

        if rospy.get_time() - self._last_publish_time > 0.5:
            self._diag_pub.publish(msg)
            self._last_publish_time = rospy.get_time()

        self._mutex.release()

if __name__ == '__main__':
    hostname = socket.gethostname()

    cpu_node = CPUMonitor(hostname)
    try:
        while not rospy.is_shutdown():
            sleep(1.0)
            cpu_node.publish_stats()

    finally:
        cpu_node.cancel_timers()
        sys.exit(0)
    


    

            

