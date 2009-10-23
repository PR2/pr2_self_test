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

##\author Kevin Watts
##\brief Wraps any executable into prestartup script

PKG = 'qualification'
import roslib; roslib.load_manifest(PKG)

import rospy
import subprocess

from qualification.srv import ScriptDone, ScriptDoneRequest

import sys

finish = rospy.ServiceProxy('prestartup_done', ScriptDone)

if __name__ == '__main__':
    rospy.init_node('prestartup_script_caller')
    args = rospy.myargv()

    if len(args) < 3:
        print 'Usage: ./prestartup_caller.py <pkg> <program> <args>'
        rospy.wait_for_service('prestartup_done', 10)
        finish.call(1, 'Usage: ./prestartup_caller.py <program> <args>')
        sys.exit(2)
    
    cmd = "rosrun %s %s " % (args[1], args[2]) 
    if len(args) > 3:
        cmd += " ".join(args[3:])

    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                         stderr=subprocess.PIPE, shell=True)
    stdout, stderr = p.communicate()
    retcode = p.returncode
    
    rospy.wait_for_service('prestartup_done', 10)
    r = ScriptDoneRequest()
    r.result = 1
    r.failure_msg = 'CMD: %s\n%s\n%s' % (cmd, stdout, stderr)
    if retcode == 0:
        r.result = 0
        r.failure_msg = ''
        finish.call(r)
    else:
        finish.call(r) 

