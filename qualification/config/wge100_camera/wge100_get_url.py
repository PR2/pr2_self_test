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
import rospy.client
import subprocess
import traceback
from invent_client.invent_client import Invent
import sys

rospy.init_node("wge100_get_url")

def getparam(name):
    val = rospy.get_param(name, None)
    if val == None:
        print >> sys.stderr, "Parameter %s not set"%name
        exit(-1)
    return val

try:
    print >> sys.stderr, "This node converts the serial number in /qualification/serial into a camera url."

    # Get inventory password from qualification
    username = getparam('/invent/username')
    password = getparam('/invent/password')
    barcode = getparam('qual_item/serial')
     
    # Fail if invalid username/password
    i = Invent(username, password)
    if not i.login():
        print >> sys.stderr, "Could not connect to invent."
        exit(-1)
    
    # Get camera url
    try:
        print >> sys.stderr, "Searching for barcode ", barcode
        camera_url = i.getItemReferences(barcode)["camera_url"]
        if camera_url == '':
            raise KeyError
    except KeyError:
        print >> sys.stderr, "Could not get camera url from invent in wge100_get_url.py"
        exit(-1)

    myargv = rospy.client.myargv()
    if len(myargv) == 2:
        camera_url = camera_url + myargv[1]
    sys.stdout.write(camera_url) # Use write to avoid trailing characters.
    print >> sys.stderr, "Url is:", camera_url
                                            
except:
    raise
    #print >> sys.stderr, "Exception caught in wge100_get_url.py"
