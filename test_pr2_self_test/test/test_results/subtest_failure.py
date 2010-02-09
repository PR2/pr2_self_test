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
from qualification.msg import TestParam, TestValue, Plot

from qualification.result import QualTestResult

from datetime import datetime

import matplotlib.pyplot as plt
from StringIO import StringIO

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
        
        r = TestResultRequest()
        r.plots = []
        r.params = []
        r.params.append(TestParam('P Gain', '5.0'))
        r.params.append(TestParam('I Gain', '1.0'))
        r.params.append(TestParam('D Gain', '0.0'))
        r.params.append(TestParam('I Clamp', '0.0'))
        
        r.values = []
        r.values.append(TestValue('Effort', '4.0', '2.0', '5.0'))
        r.values.append(TestValue('Low Range', '-2.0', '', '-1.5'))
        r.values.append(TestValue('High Range', '2.0', '1.5', ''))

        plt.plot([1,2,3,4],[16, 9, 4, 1], 'ro')
        plt.xlabel("Pirates")
        plt.ylabel("Ninjas")
        stream = StringIO()
        plt.savefig(stream, format="png")
        image = stream.getvalue()
        
        p = Plot()
        p.image = image
        p.image_format = "png"
        p.title = "pirates_and_ninjas"

        r.plots.append(p)
        r.result = TestResultRequest.RESULT_FAIL

        r.html_result = "<p>Does the correlation between pirates and ninjas make sense?</p>\n<br><img src=\"IMG_PATH/pirates_and_ninjas.png\", width = 640, height = 480 />"

        self.results.add_sub_result(0, r)


    def test_prestarts_pass(self):
        for ps in self.results.get_prestarts():
            self.assert_(ps.get_pass_bool(), "Prestarts should have passed")


    def test_subtest_fail(self):
        self.assert_(not self.results.get_pass_bool(), "Result reported success, should be failure")
        self.assert_(not self.results.is_prestart_error(), "Result reported error, should be failure")
        

    def test_subtest_image_output(self):
        subresult = self.results.get_subresult(0)

        st_html_page = subresult.make_result_page()

        for plt in subresult.get_plots():
            title = '.'.join([plt.title, plt.image_format])
            
            self.assert_(st_html_page.find(title) > 0, "Image title, name not found in HTML output. Title: %s" % title)
            
            # Need to check the image actually exists
            img_start_str = '<img src="'
            path_start = st_html_page.find(img_start_str) + len(img_start_str)
            path_stop = st_html_page.find(title) + len(title)
            img_path = st_html_page[path_start:path_stop]
            self.assert_(os.path.exists(img_path), "Image file does not exist. Path %s" % img_path)

    def test_params_values_output(self):
        subresult = self.results.get_subresult(0)

        st_html_page = subresult.make_result_page()

        for param in subresult.get_params():
            self.assert_(st_html_page.find(param.key) > 0, "Param %s not found in html output" % param.key)
            self.assert_(st_html_page.find(param.value) > 0, "Param value %s not found in html output" % param.value)

        for val in subresult.get_values():
            self.assert_(st_html_page.find(val.key) > 0, "Param %s not found in html output" % val.key)
            self.assert_(st_html_page.find(val.value) > 0, "Param value %s not found in html output" % val.value)
            if val.min != '':
                self.assert_(st_html_page.find(val.min) > 0, "Param min %s not found in html output" % val.min)
            if val.max != '':
                self.assert_(st_html_page.find(val.max) > 0, "Param max %s not found in html output" % val.max)



        
    def tearDown(self):
        self.results.close()

        
if __name__ == '__main__':
    rostest.unitrun(PKG, 'prestart_failure', TestSubTestFailure)

