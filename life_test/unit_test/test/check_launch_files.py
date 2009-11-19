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

PKG = 'life_test'

import roslib; roslib.load_manifest(PKG)
import rostest, unittest

import subprocess

##\brief Parses launch, tests.xml and configs.xml files in qualification
class LifeTestLaunchParser(unittest.TestCase):
    def setUp(self):
        pass

    ##\brief All .launch files must pass roslaunch_parse_tester
    def test_launch_file_parse(self):
        cmd = 'rosrun roslaunch_parse_tester package_parse_test.py %s --black_dir arm_life_test' % PKG
        p = subprocess.Popen(cmd, stdout = None, stderr = None, shell = True)
        p.communicate()
        retcode = p.returncode
        self.assert_(retcode == 0, "Launch files failed to parse. Run roslaunch_parse_tester to check output")


    def tearDown(self):
        pass

if __name__ == '__main__':
    rostest.unitrun(PKG, 'check_test_files', LifeTestLaunchParser)

