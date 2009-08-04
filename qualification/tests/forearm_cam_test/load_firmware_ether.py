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

import sys
import rospy
from std_srvs.srv import *
from qualification.srv import *
import qualification.msg
import std_msgs
import rospy
import subprocess
import os
import os.path

rospy.init_node("load_firmware", anonymous=True)

r = TestResultRequest()
r.plots = []

try:
    impactdir=rospy.get_param("~impactdir")
    url=rospy.get_param("~url")
except:
    import traceback
    traceback.print_exc()
#if (len(sys.argv) != 2):
    #print >> sys.stderr, 'must specify impact directory (%i) args given'%len(sys.argv);
    print >> sys.stderr, 'impactdir option must indicate impact project directory';
    print >> sys.stderr, 'url option must indicate camera url';
    r.html_result = "<p>Bad arguments to load_firmware.py.</p>"
    r.text_summary = "Error in test."
    r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
    print "error"
else:
    try:
        os.chdir(impactdir);
        p = subprocess.Popen(['rosrun', 'forearm_cam', 'upload_mcs', 'default.mcs', url], stdout=subprocess.PIPE)
        upload_mcs_out = p.communicate()[0]
    except Exception, e:
        upload_mcs_out = str(e)
   
    upload_mcs_out = upload_mcs_out.replace('\n','<br>')

    if '''Success!''' in upload_mcs_out:
        r.text_summary = "Firmware download succeeded."
        r.html_result = "<p>Test passed.</p><p>"+upload_mcs_out+"</p>" 
        r.result = TestResultRequest.RESULT_PASS
        print "pass"
    else:
        r.text_summary = "Firmware download failed."
        r.result = TestResultRequest.RESULT_FAIL
        r.html_result = "<p>Test Failed.</p><p>"+upload_mcs_out+"</p>"
        print "fail"
        print upload_mcs_out
    
result_service = rospy.ServiceProxy('test_result', TestResult)

rospy.sleep(5);

# block until the test_result service is available
rospy.wait_for_service('test_result')
result_service.call(r)

