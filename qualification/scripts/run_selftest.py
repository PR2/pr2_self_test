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

rospy.init_node("run_selftest", anonymous=True)

node_name = 'node'
node_id = 'NONE'
extramsg = ""

class EmptyReferenceID(Exception): pass
class SelfTestFailed(Exception): pass


try:
    node_name = rospy.resolve_name('node_name')
    selftestname = node_name + '/self_test'
    rospy.loginfo('Testing %s' % selftestname)

    result_service = rospy.ServiceProxy('test_result', TestResult)
    test_service = rospy.ServiceProxy(selftestname, SelfTest)

    add_ref = len(rospy.myargv()) > 1 and rospy.myargv()[1] == '--add_ref'

    rospy.wait_for_service(selftestname, 15)
    sleep(5)

    try:
    	result = test_service.call(SelfTestRequest())
    except: 
        rospy.logerr("Self test exited with an exception. It probably failed to run.")
	raise SelfTestFailed
	
    rospy.logout('Received self test service.')
    
    r = TestResultRequest()
    if (result.passed):
        r.result = r.RESULT_PASS
    else:
        r.result = r.RESULT_FAIL

    passfail = 'PASS'
    i = 1
    
    if result.id is not None and result.id != '':
        node_id = result.id
    else:
        rospy.logerr('Service %s returned with an empty reference ID.\n' % selftestname)
        raise EmptyReferenceID

    # Add item reference to invent
    if add_ref:
        from invent_client.invent_client import Invent
        username = rospy.get_param('/invent/username', '')
        password = rospy.get_param('/invent/password', '')
        serial = rospy.get_param('/qual_item/serial', None)
        
        iv = Invent(username, password)
        if not iv.login() or serial == None:
            extramsg = '<p>Unable to login to invent to store item reference. Serial: %s. Reference: %s.</p>\n' % (serial, node_id)
            raise

        iv.addItemReference(serial, '', node_id)


    html = "<p><b>Item ID: %s, using node name %s.</b></p>\n" % (node_id, node_name)
    if add_ref:
        html += '<p>Added item reference %s to inventory system under %s.</p>\n' % (node_id, rospy.get_param('/qual_item/serial', ''))

    statdict = {0: 'OK', 1: 'WARN', 2: 'ERROR'}
    
    for stat in result.status:
        if (stat.level > 1):
            passfail = 'FAIL'
        html += "<p><b>Test %2d) %s</b>\n" % (i, stat.name)
        html +=  '<br>Result %s: %s</p>\n' % (statdict[stat.level], stat.message)
        
        if len(stat.values) > 0:
            html += "<table border=\"1\" cellpadding=\"2\" cellspacing=\"0\">\n"
            html += "<tr><td><b>Label</b></td><td><b>Value</b></td></tr>\n"
            for val in stat.values:
                html += "<tr><td>%s</td><td>%s</td></tr>\n" % (val.key, val.value)
            html += "</table>\n"
        
        html += "<hr size=\"2\">\n"
        
        i += 1

    r.plots = []
    r.html_result = html
    r.text_summary = 'Node ID: %s, node name %s. Self test result: %s' % (node_id, node_name, passfail)
    
    rospy.wait_for_service('test_result')
    result_service.call(r)
    rospy.spin()

except KeyboardInterrupt:
    pass
except Exception, e:
    msg = 'Caught exception testing device id %s, node name %s.' % (node_id, node_name)
    rospy.logerr(msg)
    rospy.logerr(traceback.format_exc())
    rospy.wait_for_service('test_result', 10)
    r = TestResultRequest()
    r.plots = []
    r.html_result = '<p>%s</p><p><b>Exception:</b><br>%s</p>' % (msg, traceback.format_exc())
    r.text_summary = msg
	
    r.result = r.RESULT_FAIL
    result_service.call(r)
    rospy.spin()

sys.exit(0)


