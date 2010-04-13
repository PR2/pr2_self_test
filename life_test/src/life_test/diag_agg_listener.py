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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
##\brief Listens to diagnostics from wge100 camera and reports OK/FAIL


from __future__ import with_statement

PKG = 'life_test'
import roslib; roslib.load_manifest(PKG)

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import *

import rospy

import threading

from robot_monitor.robot_monitor_panel import State



class DiagAggState(State):
    def __init__(self, ignore_categories = []):
        State.__init__(self)

        self._ignore_categories = ignore_categories
              
    def _is_ignored(self, name):
        for ignore in self._ignore_categories:
            if name == ignore:
                return True
            if '/' + ignore == name:
                return True

        return False


    def get_top_level_state(self):
        level = -1
        min_level = 255
        msgs = []
        
        if len(self.get_items()) == 0:
            return level
    
        for item in self.get_items().itervalues():
            # Only look at "top level" items
            if self.get_parent(item) is not None:
                continue
            
            if self._is_ignored(item.status.name):
                continue

            if item.status.level > level:
                level = item.status.level
                if item.status.level < min_level:
                    min_level = item.status.level

            if item.status.level > 0:
                msgs.append(item.status.name.lstrip('/'))
                
        # Top level is error if we have stale items, unless all stale
        if level > 2 and min_level <= 2:
            level = 2
            
        return level, msgs

class DiagAggListener:
    def __init__(self):
        self._mutex = threading.Lock()

        self._level = 0
        self._msgs = []
        self._update_time = 0

    def create(self, params):
        if params.has_key('ignore_diags'):
            ignore_diags = params['ignore_diags']
        else:
            ignore_diags = [ 'Other' ]

        self._state = DiagAggState(ignore_diags)

        self._diag_agg_sub = rospy.Subscriber('/diagnostics_agg', DiagnosticArray, self._diag_callback)

        return True

    def halt(self):
        pass

    def reset(self):
        # Clears state of messages that were in error or warning
        self._msgs = []

    def _diag_callback(self, msg):
        with self._mutex:
            self._update_time = rospy.get_time()
            (added, removed, all) = self._state.update(msg)

            self._level, msgs = self._state.get_top_level_state()

            for msg in msgs:
                if self._msgs.count(msg) < 1:
                    self._msgs.append(msg)
            
    def check_ok(self):
        with self._mutex:
            msg = ''
            stat = self._level
            if stat == 1:
                msg = 'Diag Warn: %s' % (', '.join(self._msgs))
            if stat > 1:
                msg = 'Diag Error: %s' % (', '.join(self._msgs))

            if rospy.get_time() - self._update_time > 3:
                stat = 3
                msg = 'Diagnostics Stale'
                if self._update_time == 0:
                    msg = 'No Diagnostics Data'
        
        return stat, msg, None
    
