#!/usr/bin/python
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

import roslib; roslib.load_manifest(PKG)
import rostest
import unittest

from test_pr2_self_test.qual_test_result import *

from qualification.srv import TestResultRequest, ScriptDoneRequest

from qualification.result import QualTestResult

from datetime import datetime

import os

class TestSubTestFailure(unittest.TestCase):
    def setUp(self):
        self.qual_item = make_qual_item()
        self.qual_test = make_qual_test()

        self.results = QualTestResult(self.qual_item, self.qual_test, datetime.now())

        msg_ok = ScriptDoneRequest(result = ScriptDoneRequest.RESULT_OK)
        
        self.results.add_prestartup_result(0, msg_ok)
        self.results.add_prestartup_result(1, msg_ok)
        self.results.add_prestartup_result(2, msg_ok)
        self.results.add_prestartup_result(3, msg_ok)
        
        r = make_subtest_data(result = TestResultRequest.RESULT_FAIL)

        self.results.add_sub_result(0, r)

    def test_write_to_file(self):
        files_ok = check_write_to_file(self.results)

        self.assert_(files_ok, "Bad files in results!")

    def test_prestarts_pass(self):
        for ps in self.results.get_prestarts():
            self.assert_(ps.get_pass_bool(), "Prestarts should have passed")


    def test_subtest_fail(self):
        self.assert_(not self.results.get_pass_bool(), "Result reported success, should be failure")
        self.assert_(not self.results.is_prestart_error(), "Result reported error, should be failure")
        

    def test_subtest_image_output(self):
        self.assert_(subresult_image_output(self.results.get_subresult(0)), "Subtest 0 didn't display images properly")

    def test_params_values_output(self):
        self.assert_(subresult_params_values_output(self.results.get_subresult(0)), "Subtest 0 didn't display parameters and values properly")

    def test_email_msg(self):
        self.assert_(self.results.make_email_message(), "Email message is None")

        
    def tearDown(self):
        self.results.close()

        
if __name__ == '__main__':
    rostest.unitrun(PKG, 'subtest_failure', TestSubTestFailure)

