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

PKG = 'roslaunch_parse_tester'

import roslib; roslib.load_manifest(PKG)
import rostest, unittest

import os

from roslaunch_parse_tester.launch_parse import ROSLaunchParser

my_pkg_dir = roslib.packages.get_pkg_dir(PKG)

class TestROSLaunchParseTester(unittest.TestCase):
    def test_node_check_fail(self):
        basename = 'no_node_in_pkg.launch'
        file = os.path.join(my_pkg_dir, 'test', 'launch', basename)
        self.assert_(os.path.exists(file), "%s doesn't exist!" % basename)

        launch_parser = ROSLaunchParser(file, quiet = True)
        self.assert_(launch_parser.parse_test(), "%s failed to parse" % basename)
        
        self.assert_(not launch_parser.check_nodes(), "%s successfully checked nodes even though node doesn't exist!")

    def test_node_check_pass(self):
        basename = 'node_in_pkg.launch'
        file = os.path.join(my_pkg_dir, 'test', 'launch', basename)
        self.assert_(os.path.exists(file), "%s doesn't exist!" % basename)

        launch_parser = ROSLaunchParser(file, quiet = True)
        self.assert_(launch_parser.parse_test(), "%s failed to parse" % basename)
        
        self.assert_(launch_parser.check_nodes(), "%s failed to parse checked nodes even though node exists!")

    def test_machine_not_given(self):
        basename = 'no_machine_to_assign.launch'
        file = os.path.join(my_pkg_dir, 'test', 'launch', basename)
        self.assert_(os.path.exists(file), "%s doesn't exist!" % basename)

        launch_parser = ROSLaunchParser(file, quiet = True)
        self.assert_(launch_parser.parse_test(), "%s failed to parse" % basename)
        
        self.assert_(not launch_parser.check_machines(), "%s successfully parsed even though machines weren't defined!")

    def test_unnamed_node_config_error(self):
        basename = 'unnamed_node_config_error.launch'
        file = os.path.join(my_pkg_dir, 'test', 'launch', basename)
        self.assert_(os.path.exists(file), "%s doesn't exist!" % basename)

        launch_parser = ROSLaunchParser(file, quiet = True)
        #self.assert_(launch_parser.parse_test(), "%s failed to parse" % basename)
        
        self.assert_(not launch_parser.parse_test(), "%s successfully parsed even though unnamed nodes were present. This should be a config error.")
        #self.assert_(not launch_parser.check_config_errors(), "%s successfully checked even though unnamed nodes were present. This should be a config error.")

    def test_parse_failure(self):
        basename = 'parse_failure.launch'
        file = os.path.join(my_pkg_dir, 'test', 'launch', basename)
        self.assert_(os.path.exists(file), "%s doesn't exist!" % basename)

        launch_parser = ROSLaunchParser(file, quiet = True)
        self.assert_(not launch_parser.parse_test(), "%s parsed even though it does not have correct XML syntax" % basename)
        
    def test_missing_dep(self):
        basename = 'missing_dep.launch'
        file = os.path.join(my_pkg_dir, 'test', 'launch', basename)
        self.assert_(os.path.exists(file), "%s doesn't exist!" % basename)

        launch_parser = ROSLaunchParser(file, quiet = True)
        self.assert_(launch_parser.parse_test(), "%s did not parse even though it has correct XML syntax" % basename)
        self.assert_(not launch_parser.check_missing_deps(), "%s passed dependency check even though we are missing dependencies on node")

if __name__ == '__main__':
    rostest.unitrun(PKG, 'test_roslaunch_parse_tester', TestROSLaunchParseTester)
