#! /usr/bin/env python

import roslib
roslib.load_manifest('mechanism_control')
import rospy, sys
import std_srvs.srv
reset = rospy.ServiceProxy("reset_motors", std_srvs.srv.Empty)
reset()

