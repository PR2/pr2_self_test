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

from test_pr2_self_test.qual_test_result import make_qual_test, make_qual_item

from qualification.srv import TestResultRequest, ScriptDoneRequest

from qualification.result import QualTestResult

from datetime import datetime

class TestPrestartError(unittest.TestCase):
    def setUp(self):
        self.qual_item = make_qual_item()
        self.qual_test = make_qual_test()

        self.results = QualTestResult(self.qual_item, self.qual_test, datetime.now())

        msg_ok = ScriptDoneRequest(result = ScriptDoneRequest.RESULT_OK)
        msg_fail = ScriptDoneRequest(result = ScriptDoneRequest.RESULT_ERROR, failure_msg = 'my_msg ERROR')

        self.results.add_prestartup_result(0, msg_ok)
        self.results.add_prestartup_result(1, msg_fail)

        self.results.log("Prestart 1 passed")
        self.results.log("Prestart 2 failed")

    def test_prestart1_pass(self):
        prestart1 = self.results.get_prestarts()[0]

        self.assert_(prestart1.get_pass_bool(), "Prestart1 passed")

    def test_prestart2_error(self):
        self.assert_(not self.results.get_pass_bool(), "Result reported success, should be failure")
        self.assert_(self.results.is_prestart_error(), "Result reported failure, should be error")
                
        
    def test_written_output(self):
        my_note = 'My note is so awesome'
        self.results.set_notes(my_note)

        my_log = "my log entry"
        self.results.log(my_log)

        html_page = self.results.make_summary_page()

        self.assert_(html_page.find(my_note) > 0, "Note entered, but not found in output")
        self.assert_(html_page.find(my_log) > 0, "Log entered, but not found in output")

        self.assert_(html_page.find(self.qual_item.serial) > 0, "Serial number of qual item not found in results output")
        self.assert_(html_page.find(self.qual_item.name) > 0, "Name of qual item not found in results output")

    def test_email_msg(self):
        self.assert_(self.results.make_email_message(), "Email message is None")

    def tearDown(self):
        self.results.close()

        
if __name__ == '__main__':
    rostest.unitrun(PKG, 'prestart_error', TestPrestartError)

