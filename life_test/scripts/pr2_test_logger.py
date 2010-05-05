#!/usr/bin/env python

PKG = 'life_test'
import roslib; roslib.load_manifest(PKG)

from life_test.test_param import LifeTest
from life_test.test_record import TestRecord
from pr2_self_test_msgs.msg import TestStatus

import sys
import rospy

import threading

class PR2TestLogger:
    def __init__(self, filename):
        my_test = LifeTest('PR2', 'PR2 Burn in Test', 'PR2 Burn', 1, 'PR2 Burn in Test',
                           'Burn in', 'pr2_test/pr2_burn_in_test.launch', False, [])
        self._record = TestRecord(my_test, 'PR2')

        self._mutex = threading.Lock()

        self._status_sub = rospy.Subscriber('test_status', TestStatus, self._status_cb)

    def _status_cb(self, msg):
        with self._mutex:
            self._record.update(True, msg.test_ok == 0, False, '', msg.message)
        
def print_usage(code = 0):
    print >> sys.stderr, "./pr2_test_logger.py [filename='pr2_hw_test_log.csv']"
    sys.exit(code)
    

if __name__ == '__main__':
    rospy.init_node('pr2_test_logger')

    filepath = 'pr2_hw_test_log.csv'
    if len(rospy.myargv()) > 1:
        filepath = rospy.myargv()[1]

    pr2_logger = PR2TestLogger(filepath)
    
    rospy.spin()
