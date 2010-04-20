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

PKG = 'life_test'
import roslib
roslib.load_manifest(PKG)

import rospy

class LifeTest:
    def __init__(self, short_serial, test_name, short_name, 
                 duration, desc, test_type, launch_file, need_power, params):
        self._short_serial = short_serial
        self._name = test_name
        self._short = short_name
        self._duration = duration
        self._desc = desc
        self._launch_script = launch_file
        self._test_type = test_type

        self._params = params

        self.need_power = need_power

    def set_params(self, namespace):
        for param in self._params:
            param.set_namespace(namespace)

    def get_title(self, serial):
        if len(serial) == 12: # Take last few digits of SN to ID part
            return "%s %s" % (self._short, 
                                 serial[len(serial) - 3: 
                                        len(serial)])
        # Or just return the short name and the trac ticket
        return self._short

    def get_duration(self):
        return int(self._duration)

    def get_name(self):
        return self._name

    def get_type(self):
        return self._test_type
    def get_launch_file(self):
        return self._launch_script


    def needs_power(self):
        return self.need_power

    ##\brief Called during unit testing only.
    def validate(self):
        import os, sys

        full_path = os.path.join(roslib.packages.get_pkg_dir(PKG), self._launch_script)

        if not os.path.exists(full_path):
            print >> sys.stderr, "Test %s, path %s doesn't exist" % (self._name, full_path)
            return False
        if not full_path.endswith('.launch'):
            print >> sys.stderr, "Test %s, path %s is not a launch file" % (self._name, full_path)
            return False

        return True
            
        
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


