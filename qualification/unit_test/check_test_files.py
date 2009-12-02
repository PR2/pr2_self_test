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

PKG = 'qualification'

import roslib; roslib.load_manifest(PKG)
import rostest, unittest

from qualification.component_qual import load_tests_from_map, load_configs_from_map
from qualification.test import Test

import os, sys, subprocess

##\brief Parses launch, tests.xml and configs.xml files in qualification
class QualificationTestParser(unittest.TestCase):
    def setUp(self):
        self.test_files = {}
        descs = {}
        self.tests_ok = load_tests_from_map(self.test_files, descs)

        self.config_files = {}
        self.configs_ok = load_configs_from_map(self.config_files, descs)

    ##\brief All .launch files must pass roslaunch_parse_tester
    def test_launch_file_parse(self):
        cmd = 'rosrun roslaunch_parse_tester package_parse_test.py %s --env=ROS_TEST_HOST,localhost --black_dir config/wge100_camera -a' % PKG
        p = subprocess.Popen(cmd, stdout = None, stderr = None, shell = True)
        p.communicate()
        retcode = p.returncode
        self.assert_(retcode == 0, "Launch files failed to parse. Run roslaunch_parse_tester to check output")

    ##\brief All test.xml files must load properly
    def test_check_tests_parsed(self):
        self.assert_(self.tests_ok, "Tests failed to load (tests.xml)")
        self.assert_(self.test_files is not None, "Tests list is None, nothing to load")

    ##\brief All test.xml files must load and validate
    def test_load_qual_tests(self):
        tests_dir = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'tests')
        
        for sn, lst in self.test_files.iteritems():
            for my_test_file in lst:
                my_test_full_path = str(os.path.join(tests_dir, my_test_file))

                test_dir = os.path.dirname(my_test_full_path)

                self.assert_(os.path.exists(my_test_full_path), "Test file %s does not exist, unable to validate" % my_test_full_path)
                test_str = open(my_test_full_path).read()
                
                my_test = Test()
                my_test.load(test_str, test_dir)
                self.assert_(my_test.validate(), "Test failed to validate. Directory: %s\nXML: %s" % (my_test_file, test_str))

    ##\brief All config files must load successfully
    def test_check_configs_parsed(self):
        #self.assert_(False)

        self.assert_(self.configs_ok, "Configs failed to load (tests.xml)")
        self.assert_(self.config_files is not None, "Configs list is None, nothing to load")

    ##\brief All config files must validate
    def test_load_qual_configs(self):
        configs_dir = roslib.packages.get_pkg_dir('qualification')
        
        for sn, lst in self.config_files.iteritems():
            for my_config_str in lst:
                my_config = Test()
                my_config.load(my_config_str, configs_dir)
                self.assert_(my_config.validate(), "Test failed to validate. Directory: %s\nXML: %s" % (configs_dir, my_config_str))

    def tearDown(self):
        pass

if __name__ == '__main__':
    rostest.unitrun(PKG, 'check_test_files', QualificationTestParser)

