#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

##\brief Kevin Watts
##\brief Loads configuration files for Test Manager

PKG = 'life_test'
import roslib
roslib.load_manifest(PKG)

from xml.dom import minidom

import os
import rospy

from test_param import *
from test_bay import *

##\brief Loads configuration file about testing rooms
def load_rooms_from_file():
    filepath = os.path.join(roslib.packages.get_pkg_dir(PKG), 'wg_test_rooms.xml')

    rooms = {}

    try:
        doc = minidom.parse(filepath)
        rooms_xml = doc.getElementsByTagName('room')
        for room_xml in rooms_xml:
            hostname = room_xml.attributes['hostname'].value
            room = TestRoom(hostname)
            for bay in room_xml.getElementsByTagName('bay'):
                room.add_bay(TestBay(bay))
                rooms[hostname] = room

        return rooms
    except:
        import traceback
        rospy.logerr('Caught exception loading rooms data. Exception: %s' % traceback.format_exc())

        return {}
  
##\brief Loads configuration file about tests
def load_tests_from_file(test_xml_path = os.path.join(roslib.packages.get_pkg_dir(PKG), 'tests.xml')):
    my_tests = {}

    try:
        doc = minidom.parse(test_xml_path)
    except IOError:
        rospy.logerr('Could not load tests from %s' % test_xml_path)
        return {}

    try:
        tests = doc.getElementsByTagName('test')
        for test in tests:
            serial = test.attributes['serial'].value # Short serial only
            name = test.attributes['name'].value
            desc = test.attributes['desc'].value
            script = test.attributes['script'].value
            type = test.attributes['type'].value
            short = test.attributes['short'].value
            power = test.attributes['power'].value != 'false'
            
            if test.attributes.has_key('duration'):
                duration = int(test.attributes['duration'].value)
            else:
                duration = 0
            
            # Add test parameters
            # Make param from XML element
            # Append to list, add to test
            test_params = []
            params_xml = test.getElementsByTagName('param')
            for param_xml in params_xml:
                p_name = param_xml.attributes['name'].value
                p_param_name = param_xml.attributes['param_name'].value
                p_desc = param_xml.attributes['desc'].value
                p_val = param_xml.attributes['val'].value
                
                p_rate = param_xml.attributes['rate'].value == 'true'


                test_params.append(TestParam(p_name, p_param_name, 
                                             p_desc, p_val, p_rate))

                
            life_test = LifeTest(serial, name, short, duration, 
                                 desc, type, script, power, test_params)

            my_tests.setdefault(serial, []).append(life_test)
                
        return my_tests
    except:
        import traceback
        rospy.logerr('Caught exception parsing test XML. Exception: %s' % traceback.format_exc())
        return {}
