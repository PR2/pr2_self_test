#!/usr/bin/env python
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

##\author Kevin Watts
##\brief Logs Results of PR2 burn in test to CSV file, uploads to invent

from __future__ import with_statement

PKG = 'life_test'
import roslib; roslib.load_manifest(PKG)

from life_test.test_param import LifeTest
from life_test.test_record import TestRecord
from pr2_self_test_msgs.msg import TestStatus

import sys, os
import rospy

import threading
import getpass
from optparse import OptionParser
from invent_client.invent_client import Invent

class PR2TestLogger:
    def __init__(self, robot_serial, iv):
        my_test = LifeTest('PR2', 'PR2 Burn in Test', 'PR2 Burn', 1, 'PR2 Burn in Test',
                           'Burn in', 'pr2_test/pr2_burn_in_test.launch', False, [])
        self._record = TestRecord(my_test, robot_serial)

        self._mutex = threading.Lock()

        self._status_sub = rospy.Subscriber('test_status', TestStatus, self._status_cb)

        self._iv = iv
        self._serial = robot_serial
        self._closed = False

        self._record.update(False, False, False, 'Started Logging', '')

    def _status_cb(self, msg):
        with self._mutex:
            self._record.update(True, msg.test_ok == 0, False, '', msg.message)

    def close(self):
        if self._closed:
            return 

        # Don't log if we didn't start running
        if self._record.get_cum_time() < 1:
            print >> sys.stderr, "No robot burn in data record. Unable to log to inventory system"
            return

        self._record.update(False, False, False, '', '')

        f = open(self._record.csv_filename(), 'rb')
        csv_file = f.read()
        f.close()

        my_note = 'PR2 Burn in test completed. Total active time: %s' % self._record.get_active_str()

        try:
            self._iv.add_attachment(self._serial, 
                                    os.path.basename(self._record.csv_filename()),
                                    'text/csv', csv_file, my_note)
            print 'Submitted log to inventory system.'
            self._closed = True
        except:
            print >> sys.stderr, "Unable to log data into inventory system. Upload file manually: %s" % (self._record.csv_filename())
            import traceback
            traceback.print_exc()
    

if __name__ == '__main__':
    parser = OptionParser(usage="%prog -u USERNAME -r ROBOT", prog="pr2_test_logger.py")
    parser.add_option('-u', '--username', action="store", dest="username",
                      default=None, metavar="USERNAME",
                      help="Username for WG inventory system")
    parser.add_option('-r', '--robot', action="store", dest="robot",
                      default=None, metavar="ROBOT",
                      help="Robot SN to store data. Ex: 680296701000")
    
    options,args = parser.parse_args()
    if not options.username:
        parser.error("Must provide username to WG inverntory system")
    if not options.robot:
        parser.error("Must provide valid robot SN to log")
    
    if not len(options.robot) == 12:
        parser.error("%s is not a valid robot serial number" % options.robot)
    if not options.robot.startswith('6802967'):
        parser.error("%s is not a correct serial number. Must be WGPN 68-02967" % options.robot)

    print 'Enter your password to the Willow Garage Inventory system'
    my_pass = getpass.getpass()

    iv = Invent(options.username, my_pass)
    if not iv.login():
        parser.error("Must provide valid username and password to WG inventory system")
    if not iv.check_serial_valid(options.robot):
        parser.error("Robot serial number %s is invalid" % options.robot)
    
    rospy.init_node('pr2_test_logger', disable_signals = True)

    pr2_logger = PR2TestLogger(options.robot, iv)
    rospy.on_shutdown(pr2_logger.close)
    
    print "Logging PR2 burn in test status..."
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down logging, reporting to inventory system"
        pr2_logger.close()
    except Exception, e:
        import traceback
        traceback.print_exc()
