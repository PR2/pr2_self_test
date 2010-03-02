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

class TestAllPassNoPrestarts(unittest.TestCase):
    def setUp(self):
        self.qual_item = make_qual_item()
        self.qual_test = make_qual_test()

        self.results = QualTestResult(self.qual_item, self.qual_test, datetime.now())

        msg_ok = ScriptDoneRequest(result = ScriptDoneRequest.RESULT_OK)
                
        self.results.add_sub_result(0, make_subtest_data(result = TestResultRequest.RESULT_PASS))
        self.results.add_sub_result(1, make_subtest_data(result = TestResultRequest.RESULT_PASS))
        self.results.add_sub_result(2, make_subtest_data(result = TestResultRequest.RESULT_PASS))
        self.results.add_sub_result(3, make_subtest_data(result = TestResultRequest.RESULT_PASS))

        self.results.add_shutdown_result(msg_ok)


    def test_write_to_file(self):
        files_ok = check_write_to_file(self.results)

        self.assert_(files_ok, "Bad files in results!")


    def test_prestarts_passed(self):
        self.assert_(len(self.results.get_prestarts()) == 0, "Should have no prestarts in test.")


    def test_subtests_passed(self):
        self.assert_(self.results.get_pass_bool(), "Result reported success, should be failure")
        self.assert_(self.results.get_subresult(0).get_pass_bool(), "Subtest 0 failed")
        self.assert_(self.results.get_subresult(1).get_pass_bool(), "Subtest 1 failed")
        self.assert_(self.results.get_subresult(2).get_pass_bool(), "Subtest 2 failed")
        self.assert_(self.results.get_subresult(3).get_pass_bool(), "Subtest 3 failed")

        self.assert_(not self.results.is_prestart_error(), "Result reported error")
        

    def test_subtest_image_output(self):
        self.assert_(subresult_image_output(self.results.get_subresult(0)), "Subtest 0 didn't display images properly")
        self.assert_(subresult_image_output(self.results.get_subresult(1)), "Subtest 1 didn't display images properly")
        self.assert_(subresult_image_output(self.results.get_subresult(2)), "Subtest 2 didn't display images properly")
        self.assert_(subresult_image_output(self.results.get_subresult(3)), "Subtest 3 didn't display images properly")



    def test_params_values_output(self):
        self.assert_(subresult_params_values_output(self.results.get_subresult(0)), "Subtest 0 didn't display parameters and values properly")
        self.assert_(subresult_params_values_output(self.results.get_subresult(1)), "Subtest 1 didn't display parameters and values properly")
        self.assert_(subresult_image_output(self.results.get_subresult(2)), "Subtest 2 didn't display images properly")
        self.assert_(subresult_image_output(self.results.get_subresult(3)), "Subtest 3 didn't display images properly")


    def test_email_msg(self):
        self.assert_(self.results.make_email_message(), "Email message is None")


        
    def tearDown(self):
        self.results.close()

        
if __name__ == '__main__':
    if False:
        suite = unittest.TestSuite()
        suite.addTest(TestAllPassNoPrestarts('test_write_to_file'))
        suite.addTest(TestAllPassNoPrestarts('test_params_values_output'))
        suite.addTest(TestAllPassNoPrestarts('test_subtest_image_output'))
        suite.addTest(TestAllPassNoPrestarts('test_subtests_passed'))
        suite.addTest(TestAllPassNoPrestarts('test_prestarts_passed'))
        suite.addTest(TestAllPassNoPrestarts('test_email_msg'))

        unittest.TextTestRunner(verbosity = 2).run(suite)
    else:
        rostest.unitrun(PKG, 'all_tests_pass', TestAllPassNoPrestarts)

