#!/usr/bin/env python
#
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

##\author Kevin Watts
##\brief Records data from LifeTest into a CSV file

from __future__ import with_statement

PKG = 'life_test'
import roslib
roslib.load_manifest(PKG)

import csv
import os, sys, math

import rospy
from time import strftime, localtime

from socket import gethostname
#from test_param import TestParam, LifeTest



def _write_table_row(lst, bold = False):
    html = '<tr>'
    for val in lst:
        if bold:
            html += '<td><b>%s</b></td>' % val
        else:
            html += '<td>%s</td>' % val
        html += '</tr>\n'
    return html

##\brief Updates CSV record with state changes for a test
##
## 
class TestRecord:
    ##\param test LifeTest : Test type, params
    ##\param serial str : Serial number of DUT
    def __init__(self, test, serial, file_path = None):
        self._start_time = rospy.get_time()
        self._cum_seconds = 0
        self._last_update_time = rospy.get_time()
        self._was_running = False
        self._was_launched = False
        self._num_events = 0
        self._num_halts = 0

        self._serial = serial
        self._test_name = test._name

        self._test = test

        self._log = {}
        self._last_log_time = self._last_update_time


        self._cum_data = {}
        for param in test._params:
            if param.is_rate():
                self._cum_data[param.get_name()] = 0

        csv_name = strftime("%Y%m%d_%H%M%S", localtime(self._start_time)) + '_' + \
            str(self._serial) + '_' + self._test_name + '.csv'
        csv_name = csv_name.replace(' ', '_').replace('/', '__')

        if not file_path:
            file_path = os.path.join(roslib.packages.get_pkg_dir(PKG), 'logs')
        self.log_file = os.path.join(file_path, csv_name)

        
        with open(self.log_file, 'ab') as f:
            log_csv = csv.writer(f)
            log_csv.writerow(self._csv_header())

    def get_elapsed(self):
        elapsed = rospy.get_time() - self._start_time
        return elapsed

    def get_cum_time(self):
        return self._cum_seconds

    def get_active_str(self):
        return self.get_duration_str(self._cum_seconds)

    def get_elapsed_str(self):
         return self.get_duration_str(self.get_elapsed())

    @staticmethod
    def get_duration_str(duration):
        hrs = max(math.floor(duration / 3600), 0)
        min = max(math.floor(duration / 6), 0) / 10 - hrs * 60
        
        return "%dhr, %.1fm" % (hrs, min)

    # TODO
    def get_cycles(self, name = None):
        if not self._cum_data.has_key(name):
            kys = self._cum_data.keys()
            kys.sort()
            return self._cum_data[kys[0]]
        return self._cum_data[name]

    def get_cum_data(self):
        return self._cum_data
            
    ##\brief Updates test record with current state
    ##
    ## Looks at current, previous state to record data and send alerts
    ##\param launched bool : Test launched
    ##\param running bool : Running (status OK)
    ##\param stale bool : Test is stale 
    ##\param note str : Notes from operator
    ##\param monitor_msg str : Message from Test Monitor
    ##\return (int, str) : int [0:OK, 1:Notify, 2:Alert]
    def update(self, launched, running, stale, note, monitor_msg):
        if running and not launched:
            rospy.logerr('Reported impossible state of running and not launched')
            return 0, ''

        if stale and running:
            rospy.logerr('Reported impossible state of running and stale')
            return 0, ''
        
        # Something wrong here, cum seconds not updating
        d_seconds = 0
        if self._was_running and running:
            d_seconds = rospy.get_time() - self._last_update_time

        self._cum_seconds += d_seconds

        alert = 0 # 0 - None, 1 - Notify, 2 - alert
        msg = ''
        state = 'Running'

        if launched and (not running) and stale:
            state = 'Stale'
        elif launched and (not running):
            state = 'Halted'
        elif not launched:
            state = 'Stopped'

        if (not self._was_launched) and launched:
            alert = 1
            msg = "Launched."
        elif self._was_launched and (not launched):
            alert = 1
            msg = "Shut down."

        elif self._was_running and (not running):
            alert = 2
            self._num_halts += 1
            if stale:
                msg = "Stale."
            else:
                msg = "Stopped."
        elif (not self._was_running) and running:
            alert = 1
            msg = "Restarted."

        if alert > 0:
            self._num_events += 1

        # Update cumulative parameters
        for param in self._test._params:
            if param.is_rate():
                self._cum_data[param.get_name()] += d_seconds * param.get_value()

        self._was_running = running
        self._was_launched = launched
        self._last_update_time = rospy.get_time()

        if alert or note != '' or  (running and self._last_log_time - rospy.get_time() > 7200):
            self._write_csv_row(self._last_update_time, state, msg, note, monitor_msg)
            self._log[rospy.get_time()] = msg + ' ' + note
            self._last_log_time = self._last_update_time

        return alert, msg

    def _csv_header(self):
        header = [ 'Time', 'Status', 'Elapsed (s)', 'Cum. Time (s)']
        
        for param in self._test._params:
            header.append(param.get_name())
            if param.is_rate():
                header.append('Cum. %s' % param.get_name())
                self._cum_data[param.get_name()] = 0
        
        header.extend(['Num. Halts', 'Num Events', 'Monitor', 'Message'])
        return header

    def _write_csv_row(self, update_time, state, msg, note, monitor_msg):
        log_msg = msg + ' ' + note
        log_msg = log_msg.replace(',', ';')

        # Keep time machine readable?
        time_str = strftime("%m/%d/%Y %H:%M:%S", localtime(update_time))

        # Need to close this file?
        with open(self.log_file, 'ab') as f:
            log_csv = csv.writer(f)
            
            csv_row = [ time_str, state, self.get_elapsed(), self.get_cum_time() ]

            for param in self._test._params:
                csv_row.append(param.get_value())
                if param.is_rate():
                    csv_row.append(self._cum_data[param.get_name()])

            csv_row.extend( [self._num_halts, self._num_events, monitor_msg, log_msg ])
            log_csv.writerow(csv_row)

      
    def csv_filename(self):
        return self.log_file

    ##\brief Writes HTML table of last state of test
    ##
    ##\return str : HTML table 
    def write_table(self):
        html = '<table border="1" cellpadding="2" cellspacing="0">\n'
        time_str = strftime("%m/%d/%Y %H:%M:%S", localtime(self._start_time))
        html += _write_table_row(['Start Time', time_str])
        html += _write_table_row(['Elapsed Time', self.get_elapsed_str()])
        html += _write_table_row(['Active Time', self.get_active_str()])
        for ky in self.get_cum_data().keys():
            cum_name = "Cum. %s" % ky
            html += _write_table_row([cum_name, self.get_cum_data()[ky]])        

        html += _write_table_row(['Num Halts', self._num_halts])
        html += _write_table_row(['Num Alerts', self._num_events])
        html += '</table>\n'

        return html

    ##\brief Writes HTML table of test events and messages
    ##
    ##\return str : HTML table
    def write_log(self):
        if self._log is None or len(dict.keys(self._log)) == 0:
            return '<p>No test log!</p>\n'

        html = '<p>CSV location: %s on machine %s.</p>\n' % (self.csv_filename(), gethostname())

        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Time</b></td><td><b>Entry</b></td></tr>\n'
        
        kys = dict.keys(self._log)
        kys.sort()
        for ky in kys:
            time_str = strftime("%m/%d/%Y %H:%M:%S", localtime(ky))
            html += _write_table_row([time_str, self._log[ky]])
            
        html += '</table>\n'

        return html
