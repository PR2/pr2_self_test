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

##\author Kevin Watts

PKG = 'test_pr2_self_test'
QUAL_PKG = 'qualification'

import roslib; roslib.load_manifest(PKG)

from qualification.test import TestScript, SubTest, Test
from qualification.qual_frame import QualTestObject

import os

qual_pkg_dir = roslib.packages.get_pkg_dir(QUAL_PKG)

def make_qual_test():
    my_test = Test()

    my_test._name = 'Unit Test for Results'
    
    startup = TestScript(os.path.join(qual_pkg_dir, 'my_launch.launch'), 'My Startup')

    shutdown = TestScript(os.path.join(qual_pkg_dir, 'my_shutdown.launch'), 'My Startup')

    pre1 = TestScript(os.path.join(qual_pkg_dir, 'pre1.launch'), 'My Pre-Start 1')
    pre2 = TestScript(os.path.join(qual_pkg_dir, 'pre2.launch'), 'My Pre-Start 2')
    pre3 = TestScript(os.path.join(qual_pkg_dir, 'pre3.launch'), 'My Pre-Start 3')
    pre4 = TestScript(os.path.join(qual_pkg_dir, 'pre4.launch'), 'My Pre-Start 4')
    
    sub1 = SubTest(os.path.join(qual_pkg_dir, 'sub1.launch'), 1, 'My Sub 1')
    sub2 = SubTest(os.path.join(qual_pkg_dir, 'sub1.launch'), 2, 'My Sub 2')
    sub3 = SubTest(os.path.join(qual_pkg_dir, 'sub1.launch'), 3, 'My Sub 3')
    sub4 = SubTest(os.path.join(qual_pkg_dir, 'sub1.launch'), 4, 'My Sub 4')

    my_test.generate_onboard_test(QUAL_PKG, 'my_launch.launch', 'My Startup')
    my_test.add_subtests([sub1, sub2, sub3, sub4])
    my_test.pre_startup_scripts = [pre1, pre2, pre3, pre4]
    my_test._shutdown_script = shutdown

    return my_test

def make_qual_item():
    return QualTestObject('My Item', '6800000')
    
