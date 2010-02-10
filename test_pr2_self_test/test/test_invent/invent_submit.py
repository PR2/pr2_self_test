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

import os, sys, traceback

from optparse import OptionParser

from invent_client.invent_client import Invent

def make_results(reference):
    qual_item = make_qual_item(reference)
    qual_test = make_qual_test()
    
    results = QualTestResult(qual_item, qual_test, datetime.now())
    
    msg_ok = ScriptDoneRequest(result = ScriptDoneRequest.RESULT_OK)
    
    results.add_prestartup_result(0, msg_ok)
    results.add_prestartup_result(1, msg_ok)
    results.add_prestartup_result(2, msg_ok)
    results.add_prestartup_result(3, msg_ok)
    
    results.add_sub_result(0, make_subtest_data(result = TestResultRequest.RESULT_HUMAN_REQUIRED))
    results.retry_subresult(0, 'Retry test')
    
    results.add_sub_result(0, make_subtest_data(result = TestResultRequest.RESULT_PASS))
    
    results.add_sub_result(1, make_subtest_data(result = TestResultRequest.RESULT_FAIL))

    return results


if __name__ == '__main__':
    parser = OptionParser(usage="%prog --username=USERNAME --password=PASSWORD [options]", 
                          prog='invent_submit.py')
    parser.add_option('-u', '--username', action="store", dest="username",
                      default=None, metavar="USERNAME",
                      help="Username for WG inventory system")
    parser.add_option('-p', '--password', action="store", dest="password",
                      default=None, metavar="USERNAME",
                      help="Username for WG inventory system")
    parser.add_option('-r', '--reference', action="store", dest="reference",
                      default="faketest", metavar="REFERENCE",
                      help="WG serial number for component. Default: 'faketest'")
    parser.add_option('-n', '--no-submit', action="store_true", dest="no_submit",
                      default=False, help="Don't submit to WG inventory system, default False")
    parser.add_option('-e', '--email', action="store_true", default=False, dest="email",
                      help='Email qualification team at Willow Garage, default False')
    options, args = parser.parse_args()

    if not options.no_submit and (not options.username or not options.password):
        parser.error("Username and password must be specified for invent submission.")

    if options.no_submit and not options.email:
        parser.error("Must submit to inventory and/or email results. No action selected.")

    my_results = make_results(options.reference)

    all_ok = True
    msg = []

    if not options.no_submit:
        iv = Invent(options.username, options.password)
        if not iv.login:
            parser.error("Invalid username and password for WG inverntory system.")

        try:
            submit_ok, my_msg = my_results.log_results(iv)
            msg.append(my_msg)
        except:
            submit_ok = False
            msg.append('Caught exception submitting results to inventory system.\n%s' % traceback.format_exc())
            
        all_ok = submit_ok and all_ok

    if options.email:
        email_ok = my_results.email_qual_team()
        all_ok = email_ok and all_ok
        if not email_ok:
            msg.append('Emailing qual team failed')
        else:
            msg.append('Emailing qual team successful')

    if all_ok:
        print 'Invent logging success!'
        print '\n'.join(msg)
        sys.exit(0)
    else:
        print 'Failure submitting results to inventory system.'
        print '\n'.join(msg)
        sys.exit(1)
    
