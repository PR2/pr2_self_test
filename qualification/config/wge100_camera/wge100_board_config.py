#!/usr/bin/python
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
#

# Author: Blaise Gassend

import roslib; roslib.load_manifest('qualification')
import rospy
import rospy.client
from invent_client.invent_client import Invent;
from wge100_camera.srv import BoardConfig
import sys

myargv = rospy.client.myargv()

if len(myargv) != 2:
    print "WARNING! This script can only be run once per camera."
    print "Make sure you enter the right barcode!"
    print
    print "Enter barcode:"
    barcode = raw_input()
else:
    barcode = myargv[1]
print "Preparing to configure forearm camera",barcode

rospy.init_node('forearm_config')

# Parse barcode to get serial number.
if len(barcode) != 12 or not barcode.isdigit():
    print "The item id", barcode, "should be 12 numeric digits."
    exit(-1)
if not barcode[0:7] in [ "6805018", "6805027" ]:
    print "Part number", barcode[0:7], "is not a forearm camera."
    exit(-1)
serial = int(barcode[5:12])

if serial >= 1800000 and serial <= 1800015: # First 15 cameras followed a different system
    serial = serial - 1800000
   
url = "serial://%i"%serial

print "Camera url is:", url

# Get inventory password from qualification
username = rospy.get_param('/invent/username', None)
password = rospy.get_param('/invent/password', None)

# Fail if invalid username/password
i = Invent(username, password)
if not i.login():
    print "Could not connect to invent."
    exit(-1)

# Was this camera already put on invent with a different serial?
# Write camera serial number to invent if it is not already there
try:
    prevserial = i.getItemReferences(barcode)["camera_url"]
    if not prevserial in [ url, '']:
        print "This part was already stored in invent with a different serial '%s'. This should never happen."%prevserial
        exit(-1)
except KeyError:
    prevserial = ''
    pass

if prevserial != url:
    print "Writing camera_url to invent"
    i.addItemReference(barcode, "camera_url", url)
else:
    print "camera_url already in invent"

# Get MAC address from invent.
i.generateWGMacaddr(barcode, "lan0")
refs = i.getItemReferences(barcode)
macstr = refs["lan0"]
print "Camera MAC is:", macstr
mac = []
if len(macstr.rstrip("1234567890abcdefABCDEF")) != 0 or len(macstr) != 12:
    print "The MAC should be 12 hex digits."
    exit(-1)
for i in range(0, 6):
    mac.append(int(macstr[2*i:2*i+2], 16))

# Wait for service, and call it.
print "Waiting for board_config service."
rospy.wait_for_service('board_config', 10)
board_config = rospy.ServiceProxy('board_config', BoardConfig)
rslt = board_config(serial, "".join([chr(x) for x in mac]) );
print "Result is", rslt.success
exit(rslt == 1) # Returns 0 on success
