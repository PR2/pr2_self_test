#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

# Author: Kevin Watts

import roslib
roslib.load_manifest('qualification')

import sys, os, time, string, subprocess
from xml.dom import minidom

from qualification.msg import *
from srv import *
from test import *

from time import strftime
from PIL import Image
from cStringIO import StringIO

import tarfile
import socket

import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.base import MIMEBase
from email import Encoders


RESULTS_DIR = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'results')
TEMP_DIR = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'results/temp/')

## Holds results from pre-startup and shutdown scripts
class TestScriptResult:
    ##@param test_script is test/TestScript that we tested
    def __init__(self, test_script, srv_result):
        self.name = test_script.get_name()
        self.launch = test_script.launch_file

        self._result = srv_result.result
        self.msg = srv_result.failure_msg
        
    def get_name(self):
        return self.name

    def get_launch(self):
        return os.path.basename(self.launch)

    def get_pass_bool(self):
        return self._result == 0

    def has_error(self):
        return self._result == 2

    def get_result_msg(self):
        result_dict = { 0: "Pass", 1: "Fail", 2: "Error"}
        return result_dict[self._result]
    
    def get_msg(self):
        return self.msg

## Stores results of subtests in any case. Completely encapsulates
## numeric values of result types from users. 
class SubTestResultType:
    __result_types = { 
        0: "Pass", 
        1: "Fail", 
        2: "Human required", 
        3: "Manual Failure", 
        4: "Manual Pass", 
        5: "Retry", 
        # Not currently used
        6: "Error" , 
        7: "Canceled" }

    ##@param result TestResultRequest.result : 0 - Pass, 1 - Fail, 2 - Human req'd
    def __init__(self, result):
        self._result = result

    def cancel(self):
        self._result = 7

    def retry(self):
        self._result = 5

    def manual_pass(self):
        # Record as pass if auto-pass
        if self._result == 0:
            return
        self._result = 4

    def manual_fail(self):
        # Record as failure if auto-fail
        if self._result == 1:
            return
        self._result = 3

    def error(self):
        self._result = 6

    def get_msg(self):
        return self.__result_types[self._result]

    def get_html_msg(self):
        if self._result == 0:
            status_html ='<div class="pass">%s</div>' % self.get_msg()
        elif self._result == 3 or self._result == 2 or self._result == 5:
            status_html ='<div class="warn">%s</div>' % self.get_msg()
        else:
            status_html ='<div class="error">%s</div>' % self.get_msg()

        return status_html

    def get_pass_bool(self, manual_ok = True):
        if manual_ok:
            return self._result == 0 or self._result == 4
        return self._result == 0

    def is_human_required(self):
        return self._result == 2

    def is_manual(self):
        return self._result == 3 or self._result == 4
    
    def is_retry(self):
        return self._result == 5

    def is_error(self):
        return self._result == 6

    def is_cancel(self):
        return self._result == 7

## Stores and displays result from qualification subtest
class SubTestResult:
    ##@param subtest test/SubTest : Subtest that completed
    ##@param msg srv/TestResultRequest : Received msg from analysis
    def __init__(self, subtest, msg):
        self._subtest = subtest
        
        self._result = SubTestResultType(msg.result)

        self._text_result = msg.html_result
        if msg.text_summary is not None:
            self._summary = msg.text_summary
        else:
            self._summary = ''

        self._subresult_note = ''
        
        self._plots = []
        if msg.plots is not None:
            for plt in msg.plots:
                self._plots.append(plt)

        self._retry_suffix = ''
        self._retry_name = ''

        self.write_images(TEMP_DIR)

    def retry_test(self, count):
        self._result.retry()
        self._retry_suffix = "_retry%d" % count
        self._retry_name = " Retry %d" % count

    def set_operator_result(self, pass_bool):
        if pass_bool:
            self._result.manual_pass()
        else:
            self._result.manual_fail()

    def get_name(self):
        return self._subtest.get_name()

    def set_name(self, name):
        self._subtest._name = name

    def get_pass_bool(self):
        return self._result.get_pass_bool()

    def set_note(self, txt):
        self._subresult_note = txt

    def filename_base(self):
        return self._subtest.get_name().replace(' ', '_').replace('/', '__') + str(self._subtest.get_key()) + self._retry_suffix

    def filename(self):
        return self.filename_base() + '.html'

    def write_images(self, path):
        for plot in self._plots:
            stream = StringIO(plot.image)
            im  = Image.open(stream)
            
            img_file = path + self.filename_base() + '_' + plot.title + '.' + plot.image_format
            im.save(img_file)

    def html_image_result(self, img_path):
        html = '<H5 ALIGN=CENTER>Result Details</H5>'
        
        # Put '<img src=\"IMG_PATH/%s.png\" /> % image_title' in html_result
        html += self._text_result.replace('IMG_PATH/', img_path + self.filename_base() + '_')

        return html

    ## Takes test title, status, summary, details and images and makes a 
    ## readable and complete results page.
    def make_result_page(self, back_link = False, link_dir = TEMP_DIR, prev = None, next = None):
        html = "<html><head><title>Qualification Test Results: %s</title>\
<style type=\"text/css\">\
body { color: black; background: white; }\
div.pass { background: green; padding: 0.5em; border: none; }\
div.warn { background: yellow; padding: 0.5em; border: none; }\
div.error { background: red; padding: 0.5em; border: none; }\
strong { font-weight: bold; color: red; }\
em { font-style:normal; font-weight: bold; }\
</style>\
</head><body>\n" % self._subtest.get_name()

        html += self.html_header()
        
        html += '<hr size="3">\n'
        
        # Parses through text and adds image source to file
        html += self.html_image_result(link_dir)

        html += '<hr size="3">\n'
        html += self.html_test_params()
        html += '<hr size="3">\n'        

        # Add link back to index if applicable
        # May want to make page "portable" so whole folder can move
        if back_link:
            if prev:
                html += '<p align=center><a href="%s%s">Previous: %s</a></p>\n' % (link_dir, prev.filename(), prev.get_name())
            
            back_index_file = link_dir + 'index.html'
            html += '<p align=center><a href="%s">Back to Index</a></p>\n' % back_index_file

            if next:
                html += '<p align=center><a href="%s%s">Next: %s</a></p>\n' % (link_dir, next.filename(), next.get_name())
           

        html += '</body></html>'
        
        return html 

    def html_test_params(self):
        html = '<H4 align=center>Subtest Parameters</H4>\n'

        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Parameter</b></td><td><b>Value</b></td></tr>\n'

        if self._subtest._pre_script:
            html += '<tr><td>Pre-test</td><td>%s</td></tr>\n' % os.path.basename(self._subtest._pre_script)
        else:
            html += '<tr><td>Pre-test</td><td>None</td></tr>\n' 

        if self._subtest._post_script:
            html += '<tr><td>Post-test</td><td>%s</td></tr>\n' % os.path.basename(self._subtest._post_script)
        else:
            html += '<tr><td>Post-test</td><td>None</td></tr>\n' 

        launch_file = self._subtest._test_script
        (path, launch_pkg) = roslib.packages.get_dir_pkg(launch_file)
        path_idx = launch_file.find(launch_pkg) + len(launch_pkg) + 1
        
        
        html += '<tr><td>Launch package</td><td>%s</td></tr>\n' % launch_pkg
        html += '<tr><td>Launch filepath</td><td>%s</td></tr>\n' % launch_file[path_idx:]

        html += '</table>\n'

        return html 

    # The header of a subtest result, also called when diplaying 
    # short version of tests in index.html file
    def html_header(self):
        html = '<H4 ALIGN=CENTER>Results of %s%s</H4>\n' % (self._subtest.get_name(), self._retry_name)
        
        html += "<p>Status: %s</p>\n" % self._result.get_html_msg()

        if self._summary != '':
            html += '<p><em>Summary</em></p>\n' 
            html += '<p>%s</p>\n' % self._summary
        if self._subresult_note != '':
            html += '<p><em>Operator\'s Notes:</em></p>\n' 
            html += '<p>%s</p>\n' % self._subresult_note

        return html

    def make_index_line(self, link, link_dir):
        result = self._result.get_html_msg()

        path = link_dir + self.filename()
        if link:
            hyperlink = '<a href=\"%s\">%s</a>' % (path, self._subtest.get_name())
        else:
            hyperlink = self._subtest.get_name()

        summary = self._summary + '\n' + self._subresult_note

        return '<tr><td>%s</td><td>%s</td><td>%s</td></tr>\n' % (hyperlink, summary, result)


class QualTestResult:
    def __init__(self, qual_item, qual_test, start_time):
        self._qual_test = qual_test

        self._subresults = {}
        self._subresults_by_index = {}

        # Should we do this?
        self._retrys = {}
        self._retrys_by_index = {}
        
        self._prestarts = {}
        self._prestarts_by_index = {}

        self._shutdown_result = None

        self._start_time = start_time
        self._start_time_filestr = self._start_time.strftime("%Y%m%d_%H%M")
        self._start_time_name = self._start_time.strftime("%Y/%m/%d %I:%M%p")

        self._item = qual_item
        self._serial = qual_item.serial
        self._item_name = qual_item.name

        # See if the qual_item is a configuration item
        try:
            config = self._item._config
            self._config_only = True
        except:
            self._config_only = False

        self._tar_filename = None

        self._results_name = '%s_%s' % (self._serial, self._start_time_filestr)

        # Record that directory made
        self._results_dir = os.path.join(RESULTS_DIR, self._results_name)
        if not os.path.isdir(self._results_dir):
            self._made_dir = self._results_dir
            os.mkdir(self._results_dir)
        else:
            self._made_dir = ''

        self._error = False
        self._canceled = False

        self._note = ''
        self._operator = ''

    def __del__(self):
        # Delete extra directories if empty
        if self._made_dir != '' and len(os.listdir(self._made_dir)) == 0:
            os.rmdir(self._made_dir)
       
    def set_notes(self, note):
        self._note = note

    def set_operator(self, name):
        self._operator = name

    def get_prestarts(self):
        kys = dict.keys(self._prestarts_by_index)
        kys.sort()
        vals = []
        for ky in kys:
            vals.append(self._prestarts_by_index[ky])
        
        # return sorted by index
        return vals


    def get_subresults(self):
        kys = dict.keys(self._subresults_by_index)
        kys.sort()
        vals = []
        for ky in kys:
            vals.append(self._subresults_by_index[ky])
        
        # return sorted by index
        return vals

    def get_retrys(self):
        kys = dict.keys(self._retrys_by_index)
        kys.sort()
        vals = []
        for ky in kys:
            vals.append(self._retrys_by_index[ky])
        
        # return sorted by index
        return vals

    # Come up with better enforcement of get functions
    def get_subresult(self, index):
        if not self._subresults_by_index.has_key(index):
            return None

        return self._subresults_by_index[index]

    def get_retry(self, index):
        if not self._retrys_by_index.has_key(index):
            return None

        return self._retrys_by_index[index]

    def add_shutdown_result(self, msg):
        script = self._qual_test.getShutdownScript()

        self._shutdown_result = TestScriptResult(script, msg)
    
    def add_prestartup_result(self, index, msg):
        test_script = self._qual_test.pre_startup_scripts[index]

        pre = TestScriptResult(test_script, msg)

        self._prestarts[test_script.get_name()] = pre
        self._prestarts_by_index[index] = pre

    def add_sub_result(self, index, msg):
        subtest = self._qual_test.subtests[index]

        sub = SubTestResult(subtest, msg)
        
        self._subresults[subtest.get_name()] = sub
        self._subresults_by_index[index] = sub
        return sub

    def cancel(self):
        self._canceled = True

    def error(self):
        self._error = True

    def retry_subresult(self, index, notes):
        retry_count = len(self._retrys.values()) + 1

        sub = self.get_subresult(index)
        if not sub:
            return

        sub.retry_test(retry_count)
        sub.set_note(notes)

        name = sub.get_name()

        retry_name = name + "_retry%d" % (retry_count)

        del self._subresults[name]
        del self._subresults_by_index[index]
        
        sub.write_images(TEMP_DIR, ) # Rewrite images for display
        self._retrys[retry_name] = sub
        self._retrys_by_index[retry_count] = sub

    def prestarts_ok(self):
        for prestart in self.get_prestarts():
            if not prestart.get_pass_bool():
                return False

        return True

    def is_prestart_error(self):
        error = False
        fail = False
        for prestart in self.get_prestarts():
            error = error and prestart.has_error()
            fail = fail and not prestart.get_pass_bool()

        return error and not fail

    def get_pass_bool(self):
        if self._canceled or self._error:
            return False

        if not self.prestarts_ok():
            return False

        if self._shutdown_result and not self._shutdown_result.get_pass_bool():
            return False

        if len(self.get_subresults()) == 0:
            return False
        
        pass_bool = True
        for res in self.get_subresults():
            pass_bool = pass_bool and res.get_pass_bool()
        return pass_bool

    def get_test_result_str(self):
        if len(self.get_subresults()) == 0 or not self.prestarts_ok():
            return "Fail"

        if self._canceled:
            return "Cancel"
        if self._error:
            return "Error"

        manual = False
        for res in self.get_subresults():
            if res._result.is_retry():
                continue

            if res._result.is_human_required():
                return "Human Required"

            if res._result.is_error():
                 return "Error"
            if res._result.is_cancel():
                return "Cancel"
            if not res._result.get_pass_bool():
                return "Fail"

            if res._result.is_manual():
                manual = True

        if manual:
            return "Operator Pass"
        return "Pass"


    def make_summary_page(self, link = True, link_dir = TEMP_DIR):
        html = "<html><head>\n"
        html += "<title>Qualification Test Result for %s as %s: %s</title>\n" % (self._serial, self._item_name, self._start_time_name)
        html += "<style type=\"text/css\">\
body { color: black; background: white; }\
div.pass { background: green; padding: 0.5em; border: none; }\
div.warn { background: yellow; padding: 0.5em; border: none; }\
div.error { background: red; padding: 0.5em; border: none; }\
strong { font-weight: bold; color: red; }\
em { font-style: normal; font-weight: bold; }\
</style>\
</head>\n<body>\n"

        if not self._config_only:
            html += '<H2 ALIGN=CENTER>Qualification of: %s</H2>\n<br>\n' % self._serial
        else:
            html += '<H2 ALIGN=CENTER>Configuration of: %s</H2>\n<br>\n' % self._serial
            st = self.get_subresult(0)
            if st is not None:
                html += st.html_header()
            html += '</body></html>'
            return html

        if self.get_pass_bool():
            result_header = '<H3>Test Result: <em>%s</em></H3>\n' % self.get_test_result_str()
        else:
            result_header = '<H3>Test Result: <strong>%s</strong></H3>\n' % self.get_test_result_str()

        html += result_header
        html += '<HR size="2">'

        if self._operator == '':
            operator = 'Unknown'
        else:
            operator = self._operator

        if self._note is None or self._note == '':
            self._note = 'No notes given.'
        
        html += '<H5><b>Test Engineer\'s Notes</b></H5>\n'
        html += '<p><b>Test Engineer: %s</b></p>\n' % operator        
        html += '<p>%s</p>\n' % self._note

        html += '<H5>Test Date</H5>\n'
        html += '<p>Completed Test at %s on %s.</p>\n' % (self._start_time_name, self._serial)
        html += '<H5><b>Results Directory</b></H5>\n<p>%s</p>\n' % self._results_dir

        if self._canceled:
            html += '<p><b>Test canceled by operator.</b></p>\n'

        if len(self.get_subresults()) == 0:
            if self.prestarts_ok():
                html += '<p>No subtests completed. Test may have ended badly.</p>\n'
            elif self.is_prestart_error():
                html += '<p>Error during pretests. Check system and retry.</p>\n'
            else:
                html += '<p>Prestartup failure. Component may be damaged.</p>\n'
        else:
            html += '<HR size="2">\n'
            # Index items link to each page if link 
            html += self.make_index(link, link_dir)
            
        if len(self.get_retrys()) > 0:
            html += '<hr size="2">\n'
            html += self.make_retry_index(link, link_dir)


        startup = self._qual_test.getStartupScript()
        if startup:
            html += '<hr size="2">\n'
            html += self.make_startup_data()
            
        # Make table for prestartup scripts
        if len(self._prestarts) > 0:
            html += '<hr size="2">\n'
            html += self.make_prestart_table()

        if self._qual_test.getShutdownScript():
            html += '<hr size="2">\n'
            html += self.make_shutdown_results()

        html += '<hr size="2">\n'
        html += self.make_log_table()
        html += '<hr size="2">\n'

        html += '</body></html>'

        return html

    def make_startup_data(self):
        startup = self._qual_test.getStartupScript()
        
        html = '<H4 align=center>Startup Script</H4>\n'
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Parameter</b></td><td><b>Value</b></td></b>\n'
        html += '<tr><td>Name</td><td>%s</td>' % startup.get_name()

        # Get launch pkg, filepath for startup
        launch_file = startup.launch_file
        (path, launch_pkg) = roslib.packages.get_dir_pkg(launch_file)
        path_idx = launch_file.find(launch_pkg) + len(launch_pkg) + 1
                
        html += '<tr><td>Launch package</td><td>%s</td></tr>\n' % launch_pkg
        html += '<tr><td>Launch filepath</td><td>%s</td></tr>\n' % launch_file[path_idx:]
        html += '</table>\n'

        return html


    def make_shutdown_results(self):
        shutdown = self._qual_test.getShutdownScript()

        html = '<H4 align=center>Shutdown Script</H4>\n'

        if not self._shutdown_result:
            html += '<p>Shutdown script: %s</p>\n' % shutdown.get_name()
            html += '<p>No shutdown results.</p>\n'
            return html

        name = self._shutdown_result.get_name()
        launch = self._shutdown_result.get_launch()
        res = self._shutdown_result.get_result_msg()
        msg = self._shutdown_result.get_msg()


        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Name</b></td><td><b>Launch File</b></td><td><b>Result</b></td><td><b>Message</b></td></tr></b>\n'
        html += '<tr><td>%s</td><td>%s</td>' % (name, launch)
        html += '<td>%s</td><td>%s</td></tr>\n' % (res, msg)
        html += '</table>\n'

        return html


    def line_summary(self):
        if self._config_only and self.get_pass_bool():
            sum = "Reconfigured %s as %s." % (self._serial, self._qual_test.getName())
        else:
            sum = "Qualification of %s, Result: %s" % (self._serial, self.get_test_result_str())

        return sum

    def make_retry_index(self, link, link_dir):
        html = '<H4 AlIGN=CENTER>Retried Subtest Index</H4>\n'
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Test Name</b></td><td><b>Summary</b></td><td><b>Final Result</b></td></tr></b>\n'

        for st in self.get_retrys():
            html += st.make_index_line(link, link_dir)

        html += '</table>\n'
        
        return html

    def make_index(self, link, link_dir):
        html = '<H4 AlIGN=CENTER>Results Index</H4>\n'
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Test Name</b></td><td><b>Summary</b></td><td><b>Final Result</b></td></tr></b>\n'

        for st in self.get_subresults():
            html += st.make_index_line(link, link_dir)

        html += '</table>\n'
        
        return html
        
    # Made table of all log entries for this test series
    def make_log_table(self):
        if self._test_log is None or self._test_log == '':
            return ''

        # Sort test log by times
        kys = dict.keys(self._test_log)
        kys.sort()
        
        html = '<H4 AlIGN=CENTER>Test Log Data</H4>\n'
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Time</b></td><td><b>Log Message</b></td></tr></b>\n'
        for ky in kys:
            time_str = ky.strftime("%m/%d/%Y %H:%M:%S")
            html += '<tr><td>%s</td><td>%s</td></tr>\n' % (time_str, self._test_log[ky])
        html += '</table>\n'

        return html

    def make_prestart_table(self):
        if len(self.get_prestarts()) == 0:
            return '<p>No prestartup scripts.</p>\n'
    
        html = '<H4 ALIGN=CENTER>Prestartup Script Data</H4>\n'
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Script</b></td><td><b>Launch File</b></td>'
        html += '<td><b>Result</b></td><td><b>Message</b></td></tr></b>\n'
        for prestart in self.get_prestarts():
            html += '<tr><td>%s</td><td>%s</td>\n' % (prestart.get_name(), prestart.get_launch())
            html += '<td>%s</td><td>%s</td></tr>\n' % (prestart.get_result_msg(), prestart.get_msg())
        html += '</table>\n'

        return html

    def write_results_to_file(self, temp = True, local_link = False):
        # Write into temp or final dir
        # Temp for display in Qual GUI, final for storing for later
        write_dir = TEMP_DIR
        if not temp:
            write_dir = self._results_dir
        
        # Use local links for image sources, etc if true
        link_dir = write_dir
        if local_link:
            link_dir = ''

        if (not os.path.isdir(write_dir)):
            os.mkdir(write_dir)
              
        index_path = write_dir + 'index.html'
        index = open(index_path, 'w')
        index.write(self.make_summary_page(True, link_dir))
        index.close()
        
        for i in self._subresults_by_index:
            st = self.get_subresult(i)
            
            #prev = None
            #next = None
            #if i > 0:
            prev = self.get_subresult(i - 1)
            #if i < len(self._subresults_by_index):
            next = self.get_subresult(i + 1)

            # for st in self.get_subresults():
            st_path = write_dir + st.filename()
            st_file = open(st_path, 'w')
            st_file.write(st.make_result_page(True, link_dir, prev, next))
            st_file.close()

            st.write_images(write_dir)

        for i in self._retrys_by_index:
            st = self.get_retry(i)
            
            #prev = None
            #next = None
            #if i > 0:
            prev = self.get_retry(i - 1)
            #if i < len(self._subresults_by_index):
            next = self.get_retry(i + 1)

            # for st in self.get_subresults():
            st_path = write_dir + st.filename()
            st_file = open(st_path, 'w')
            st_file.write(st.make_result_page(True, link_dir, prev, next))
            st_file.close()

            st.write_images(write_dir)
        
        # Make tar file of results
        if not temp:
            self.write_tar_file()
        
    # Dumps all files in results directory into tar file
    def write_tar_file(self):
        self._tar_filename = '%s/%s.tar' % (self._results_dir, self._results_name)
        
        # Change filename to basename when adding to tar file
#        tar = tarfile.open(self._tar_filename, 'w:gz')
        tar = tarfile.open(self._tar_filename, 'w:')
        for filename in os.listdir(self._results_dir):
            # Use only file base names in tar file
            fullname = os.path.join(self._results_dir, filename)
            tar.add(fullname, arcname=filename)
        tar.close()

    # Make invent results pretty, HTML links work
    def log_results(self, invent):
        # Write results to results dir, with local links
        self.write_results_to_file(False, True)

        print 'Wrote results'

        if invent == None:
            return False, "Attempted to log results to inventory, but no invent client found."
        if len(self.get_subresults()) == 0:
            return False, "No subtest results found, not logging to invent."
        if self.is_prestart_error():
            return False, "Test recorded internal error, not submitting to inventory system."
        
        prefix = self._start_time_filestr + "_" # Put prefix first so images sorted by date
        
        reference = self._serial
            
        print 'Writing invent note'

        invent.setNote(self._serial, self.line_summary())

        if self._config_only:
            sub = self.get_subresult(0) # Only subresult of configuration
            invent.add_attachment(self._serial, prefix + sub.filename(), 'text/html', sub.make_result_page())
            return True, 'Logged reconfiguration in inventory system.'

        print 'Setting test status'

        invent.setKV(self._serial, "Test Status", self.get_test_result_str())

        print 'Adding summary'
        invent.add_attachment(reference, prefix + "summary.html", "text/html", self.make_summary_page(False))
        
        print 'adding attachment'

        # Use add attachment...
        if True:
            try:
                # Need to get tar to bit stream
                f = open(self._tar_filename, "rb")
                tar = f.read()
                invent.add_attachment(reference, 
                                      prefix + os.path.basename(self._tar_filename),
                                      'applicaton/tar', tar)

                f.close()            
                return True, 'Wrote tar file, uploaded to inventory system.'
            except Exception, e:
                import traceback
                traceback.print_exc()
                print "filename", self._tar_filename
                print 'Caught exception uploading tar file. %s' % str(e)
                return False, 'Caught exception loading tar file to inventory. %s' % str(e)

        else:
            _path = os.path.join(roslib.packages.get_pkg_dir("qualification"), "src", "qualification", "add_attachment.py")
            print "_path", _path
            cmd = [_path, "--username=" + invent.username, "--password=" + invent.password, reference, self._tar_filename]
            print cmd
            retcode = subprocess.call(cmd)
            print "retcode:", retcode

            if retcode == 0:
                return True, 'Wrote tar file, uploaded to inventory system.'
            return False, 'Received retcode %s from invent.add_attachment, unable to upload to inventory system.' % retcode

    def get_qual_team(self):
        if socket.gethostname() == 'nsf': # Debug on NSF
            return 'watts@willowgarage.com'

        return 'qualdevteam@lists.willowgarage.com'

    # Email qualification team results as HTML summary and tar file
    def email_qual_team(self):
        try:
            msg = MIMEMultipart('alternative')
            msg['Subject'] = "--QualResult-- %s" % self.line_summary()
            msg['From'] = "qual.test@willowgarage.com" 
            msg['To'] = self.get_qual_team()

            msg.attach(MIMEText(self.make_summary_page(False), 'html'))
            
            # Add results as tar file
            if self._tar_filename is not None and self._tar_filename != '':
                part = MIMEBase('application', 'octet-stream')
                part.set_payload( open(self._tar_filename, 'rb').read())
                Encoders.encode_base64(part)
                part.add_header('Content-Disposition', 'attachment; filename="%s"' 
                                % os.path.basename(self._tar_filename))
                msg.attach(part)


            s = smtplib.SMTP('localhost')
            s.sendmail('qual.test@willowgarage.com', self.get_qual_team(), msg.as_string())
            s.quit()
            return True
        except Exception, e:
            print 'Unable to sent mail, caught exception!'
            print e
            return False
