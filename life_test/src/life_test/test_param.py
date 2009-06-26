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

# Author: Kevin Watts


import roslib
roslib.load_manifest('life_test')

import rospy

# Should include parameters as a list here
class LifeTest:
    def __init__(self, short_serial, test_name, short_name, trac, desc, test_type, launch_file, params):
        self._short_serial = short_serial
        self._name = test_name
        self._short = short_name
        self._trac = trac
        self._desc = desc
        self._launch_script = launch_file
        self._test_type = test_type

        self._params = params

    def set_params(self, namespace):
        for param in self._params:
            param.set_namespace(namespace)

    def get_title(self, serial):
        if len(serial) == 12: # Take last few digits of SN to ID part
            return "%s #%s %s" % (self._short, self._trac,
                                 serial[len(serial) - 5: 
                                        len(serial)])
        # Or just return the short name and the trac ticket
        return "%s #%s" % (self._short, self._trac)
            
        
## Stores parameter data for each test
## Parameters are ROS parameters, and are updated in test log
## Examples: cycle rate, joint torque, roll on/off
## Allows changes in test setup or implementation to be logged automatically
class TestParam():
    def __init__(self, name, param_name, desc, val, rate):
        self._value = val
        self._desc = desc

        self._cumulative = rate
        self._param_name = param_name
        self._name = name
        self._namespace = ''

 
    def set_namespace(self, ns):
        self._namespace = ns
        rospy.set_param('/' + self._namespace + '/' + self._param_name, self._value)

    def get_namespace(self):
        return self._namespace

    def get_value(self):
        try:
            val = float(self._value)
            return val
        except:
            return str(self._value)

    def get_name(self):
        return self._name

    def is_rate(self):
        return self._cumulative


