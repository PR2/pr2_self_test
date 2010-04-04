#! /usr/bin/env python


##\author Kevin Watts
##\brief Restarts base driving when joy button is called

PKG = 'life_test'

import roslib; roslib.load_manifest(PKG)
import rospy
from joy.msg import Joy
from std_srvs.srv import Empty

# Joy buttons. R2 is resume
DEADMAN = 10
RESUME = 9

_start_driving = rospy.ServiceProxy('start_driving', Empty)

last_restart = 0

def start_driving():
    global last_restart
    if rospy.get_time() - last_restart < 0.5:
        return

    last_restart = rospy.get_time()

    try:
        _start_driving()
        rospy.loginfo('Restarted drive command')
        return True
    except Exception, e:
        rospy.logwarn('Unable to restart driving. Service may be unavailable')
        return False

def joy_cb(msg):
    if len(msg.buttons) < DEADMAN + 1 or len(msg.buttons) < RESUME + 1:
        return

    if msg.buttons[DEADMAN] == 0 and msg.buttons[RESUME] == 1:
        start_driving()



if __name__ == '__main__':
    rospy.init_node('pr2_drive_restart')
    
    joy_sub = rospy.Subscriber('joy', Joy, joy_cb)

    rospy.spin()
