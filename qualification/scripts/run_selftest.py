#!/usr/bin/env python
#
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

import roslib
roslib.load_manifest('qualification')

import rospy
from qualification.srv import TestResult, TestResultRequest
from diagnostic_msgs.srv import SelfTest, SelfTestRequest

import sys
from time import sleep

import traceback

from optparse import OptionParser



node_name = 'node'
node_id = 'NONE'
extramsg = ""

class EmptyReferenceID(Exception): pass
class SelfTestFailed(Exception): pass

def add_reference(reference):
    from invent_client.invent_client import Invent
    username = rospy.get_param('/invent/username', '')
    password = rospy.get_param('/invent/password', '')
    serial = rospy.get_param('/qual_item/serial', None)
    
    iv = Invent(username, password)
    if not iv.login() or serial == None:
        extramsg = '<p>Unable to login to invent to store item reference. Serial: %s. Reference: %s.</p>\n' % (serial, node_id)
        return False
    
    iv.addItemReference(serial, '', node_id)

    return True


def get_error_result(msg):
    rospy.logerr(msg)

    r = TestResultRequest()
    r.html_result = '<p>%s</p>' % (msg)
    r.text_summary = msg
	
    r.result = r.RESULT_FAIL

    return r


def write_selftest(node_name, add_ref, srv):
    pf_dict = { True: 'PASS', False: 'FAIL' }

    r = TestResultRequest()
    if (result.passed):
        r.result = r.RESULT_PASS
    else:
        r.result = r.RESULT_FAIL

    html = ["<p><b>Item ID: %s, using node name %s.</b></p>" % (srv.id, node_name)]

    if add_ref:
        html.append('<p>Added item reference %s to inventory system under %s.</p>' % (srv.id, rospy.get_param('/qual_item/serial', '')))

    statdict = {0: 'OK', 1: 'WARN', 2: 'ERROR'}
    
    for i, stat in enumerate(srv.status):
        html.append("<p><b>Test %2d) %s</b>" % (i + 1, stat.name))
        html.append( '<br>Result %s: %s</p>' % (statdict[stat.level], stat.message))
        
        if len(stat.values) > 0:
            html.append("<table border=\"1\" cellpadding=\"2\" cellspacing=\"0\">")
            html.append("<tr><td><b>Label</b></td><td><b>Value</b></td></tr>")
            for val in stat.values:
                html.append("<tr><td>%s</td><td>%s</td></tr>" % (val.key, val.value))
            html.append("</table>")
        
        html.append("<hr size=\"2\">")
        
    r.html_result = '\n'.join(html)
    r.text_summary = 'Node ID: %s, node name %s. Self test result: %s' % (srv.id, node_name, pf_dict[result.passed])

    return r
    
              

def call_selftest(node_name):
    selftestname = node_name + '/self_test'
    rospy.logdebug('Testing %s' % selftestname)

    test_service = rospy.ServiceProxy(selftestname, SelfTest)

    try:
        rospy.wait_for_service(selftestname, 15)
    except:
        rospy.logerr('Wait for service %s timed out. Unable to process self test data.' % selftestname)
        return False, None

    try:
    	st_result = test_service.call(SelfTestRequest())
    except: 
        rospy.logerr("Self test exited with an exception. It probably failed to run.")
        return False, None

    rospy.logdebug('Received self test service.')
    return True, st_result


if __name__ == '__main__':
    parser = OptionParser(usage="./%prog [--add_ref]", prog="run_selftest.py")
    parser.add_option("-a", "--add_ref", default=False, dest="add_ref", action="store_true",
                      help="Add a reference in invent for the item's hardware ID")
    options, args = parser.parse_args(rospy.myargv())
    	
    rospy.init_node('selftest_caller')

    result_service = rospy.ServiceProxy('test_result', TestResult)
    
    node_name = rospy.resolve_name('node_name')
    if node_name == 'node_name':
        rospy.logwarn('\'node_name\' did not remap. Attempting to call service \'node_name/self_test\'. This is probably an error')


    ok, result = call_selftest(node_name)

    try:
        rospy.wait_for_service('test_result', 5)
    except:
        rospy.logfatal('Unable to contact result service. Exiting')
        sys.exit(-1)

    sent_results = False

    if not ok:
        r = get_error_result('Selftest of %s didn\'t get a result. Unable to record results' % node_name)
        result_service.call(r)
        sent_results = True
        rospy.spin() # Wait to get killed
        sys.exit()

    if not result.id or result.id == '':
        r = get_error_result('Selftest of  %s returned with an empty reference ID.' % node_name)
        result_service.call(r)
        sent_results = True
        rospy.spin() # Wait to get killed
        sys.exit()

    # Add item reference to invent
    if options.add_ref:
        if not add_reference(result.id):
            r = get_error_result('Failed to add reference id to inventory. Node: %s, ID: %s' % (node_name, result.id))
            result_service.call(r)
            sent_results = True
            rospy.spin() # Wait to get killed
            sys.exit()

    if not sent_results:
        r = write_selftest(node_name, options.add_ref, result)
        result_service.call(r)
        rospy.spin()


sys.exit(0)


