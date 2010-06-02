#!/usr/bin/env python
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
##\brief Checks that wge100 camera is present using "discover"

PKG = 'qualification'
import roslib; roslib.load_manifest(PKG)

import rospy

from pr2_self_test_msgs.srv import ScriptDone, ScriptDoneRequest
from pr2_self_test_msgs.srv import ConfirmConf, ConfirmConfResponse, ConfirmConfRequest

import subprocess, sys


SRV_NAME = 'prestartup_done'
finish = rospy.ServiceProxy(SRV_NAME, ScriptDone)

confirm_proxy = rospy.ServiceProxy('mcb_conf_results', ConfirmConf)

##\brief Returns True if user wants to try again
def _report_no_cameras(interface):
    conf = ConfirmConfRequest()
    conf.message = "No cameras found on interface %s. Check camera lights. Click OK to retry." % interface
    conf.details = "No cameras found. This may be a problem with the cables to the camera.\nCamera light codes:\n\tGreen - Power\n\tOrange - Connection\n\nIf lights are on, unplug and plug in camera and retry."

    resp = confirm_proxy.call(conf)
    return resp.retry == ConfirmConfResponse.RETRY


def check_camera(interface = 'eth2'):
    while not rospy.is_shutdown():
        p = subprocess.Popen('rosrun wge100_camera discover %s' % interface, 
                             stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, shell=True)

        o,e = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            if _report_no_cameras(interface):
                continue
            print >> sys.stderr, "Unable to run discover. Camera may not be present"
            return False, "Unable to run discover. Camera may not be present"
        
        try_again = False
        for ln in e.split('\n'):
            if ln.find('No cam') > -1:
                if not _report_no_cameras(interface):
                    print >> sys.stderr, "No cameras found"
                    return False, "No cameras found"
                else:
                    try_again = True
                    break
        if try_again:
            continue
            
        found = 0
        for ln in o.split('\n'):
            if ln.find('No cam') > -1:
                if _report_no_cameras(interface):
                    continue
                print >> sys.stderr, "No cameras found"
                return False, "No cameras found"
            elif ln.find('Found') == 0:
                found += 1
        if found > 0:
            return True, ''
        if not _report_no_cameras(interface):
           return False, "No cameras"

    return False, 'Rospy shutdown'




if __name__ == '__main__':
    rospy.init_node('check_wge100_present')
    args = rospy.myargv()
    if len(args) > 1:
        interface = args[1]
    else:
        interface = 'eth2'
    

    val, msg = check_camera(interface)
    print val, msg

    r = ScriptDoneRequest()
    r.result = 0
    r.failure_msg = 'Found wge100 camera on interface %s' % interface
    if not val:
        r.result = 1
        r.failure_msg = msg

    rospy.wait_for_service(SRV_NAME, 5)
    finish.call(r)
        
    rospy.spin()
