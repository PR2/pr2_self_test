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
from qualification.srv import TestResultRequest, ScriptDoneRequest
from qualification.msg import TestParam, TestValue, Plot

import os

import matplotlib.pyplot as plt
from StringIO import StringIO


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

def make_subtest_data(result = TestResultRequest.RESULT_PASS, summary = 'Summary'):
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
    r.result = result
    
    r.html_result = "<p>Does the correlation between pirates and ninjas make sense?</p>\n<br><img src=\"IMG_PATH/pirates_and_ninjas.png\", width = 640, height = 480 />"
    r.text_summary = summary

    return r
    

##\brief Returns true if subtest images properly stored and displayed
def subresult_image_output(subresult):
    st_html_page = subresult.make_result_page()
    
    for plt in subresult.get_plots():
        title = '.'.join([plt.title, plt.image_format])
        
        if not st_html_page.find(title) > 0:
            print "Image title, name not found in HTML output. Title: %s" % title
            return False
        
        # Need to check the image actually exists
        img_start_str = '<img src="'
        path_start = st_html_page.find(img_start_str) + len(img_start_str)
        path_stop = st_html_page.find(title) + len(title)
        img_path = st_html_page[path_start:path_stop]
        if not os.path.exists(img_path):
            print "Image file does not exist. Path %s" % img_path
            return False

    return True

##\brief Returns true if subtest parameters and values displayed in HTML output
def subresult_params_values_output(subresult):
    st_html_page = subresult.make_result_page()
    
    for param in subresult.get_params():
        if not st_html_page.find(param.key) > 0: 
            print "Param %s not found in html output" % param.key
            return False
        if not st_html_page.find(param.value) > 0:
            print "Param value %s not found in html output" % param.value
            return False
        
    for val in subresult.get_values():
        if not st_html_page.find(val.key) > 0:
            print "Param %s not found in html output" % val.key
            return False
        if not st_html_page.find(val.value) > 0:
            print "Param value %s not found in html output" % val.value
            return False
        if val.min != '':
            if not st_html_page.find(val.min) > 0:
                print "Param min %s not found in html output" % val.min
                return False
        if val.max != '':
            if not st_html_page.find(val.max) > 0:
                print "Param max %s not found in html output" % val.max
                return False
          
    return True
