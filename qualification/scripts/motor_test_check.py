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

# Author: Kevin Watts

import roslib
roslib.load_manifest('qualification')
import rospy

from invent_client.invent_client import Invent
from qualification.srv import ScriptDone, ScriptDoneRequest

class MotorStatusVerifier:
    def __init__(self):
        rospy.init_node('motor_tester')
        self.done_service = rospy.ServiceProxy('prestartup_done', ScriptDone)

        self.has_finished = False

    def finished(self, pass_bool, msg = ''):
        if self.has_finished:
            return

        try:
            result = ScriptDoneRequest()
            if pass_bool:
                result.result = 0 # OK
            else:
                result.result = 1 # Fail

            result.failure_msg = msg
                
            rospy.wait_for_service('prestartup_done', 10)
            self.done_service.call(result)
            self.has_finished = True
        except Exception, e:
            rospy.logerr('Unable to report results. Data: %s. Exception:\n%s' % (msg, traceback.format_exc()))

    def check_motor(self):
        username = rospy.get_param('/invent/username', None)
        password = rospy.get_param('/invent/password', None)

        if username == None or password == None:
            self.finished(False, 'Invalid username or password to inventory.')
            return

        self.invent = Invent(username, password)
        if not self.invent.login():
            self.finished(False, 'Unable to login to invent.')
            return 

        item = rospy.get_param('qual_item/serial', None)
        if item is None:
            self.finished(False, 'No item found')
            return 

        if len(item) != 12:
            self.finished(False, 'Invalid serial number for motor: %s' % item)
            return 

        ##\todo Verify SN is right type

        pf = self.invent.getKV(item, 'Test Status')
        if pf == 'PASS':
            self.finished(True, 'Motor has passed EE test')
            return
        self.finished(False, 'Motor has failed EE test')
        return
        

if __name__ == '__main__':
    rospy.init_node('motor_tester')

    verifier = MotorStatusVerifier()
    try:
        verifier.check_motor()
        rospy.spin()
    except KeyboardInterrupt:
        pass
    except:
        import traceback
        rospy.logerr(traceback.format_exc())
        verifier.finished(False, 'Caught exception in check_motor loop. %s' % traceback.format_exc())
