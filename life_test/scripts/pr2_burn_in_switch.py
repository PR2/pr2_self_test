#!/usr/bin/env python

PKG = 'life_test'
import roslib; roslib.load_manifest(PKG)
import rospy

import sys, os

from optparse import OptionParser

from roslaunch_caller import roslaunch_caller

from time import sleep

if __name__ == '__main__':
    rospy.init_node('pr2_burn_in_switch')

    parser = OptionParser(usage="./%prog FILES [options]", prog="pr2_burn_switch.py")
    parser.add_option("-d", "--duration", action="store", default=0.5,
                      help="Duration for each launch file, in minutes",
                      metavar="DURATION", type="float")
    
    options, args = parser.parse_args(rospy.myargv())

    if len(args) < 2:
        parser.error("Must specify at least one launch file")
        sys.exit(1)

    for f in args:
        if not os.path.exists(f):
            parser.error("File %s doesn't exist" % f)
            sys.exit(1)

    index = 1
    while not rospy.is_shutdown():
        my_filename = args[index]
        my_pkg = 'None'
        if roslib.packages.is_pkg_dir(os.path.dirname(my_filename)):
            my_pkg = roslib.packages.get_dir_pkg(os.path.dirname(my_filename))
        rospy.loginfo("Launching file %s, package %s" % (os.path.basename(my_filename), my_pkg))
        
        my_file = open(my_filename, 'r').read()
        launcher = roslaunch_caller.ScriptRoslaunch(my_file, None)
        launcher.start()

        rospy.loginfo('Launched file %s' % os.path.basename(my_filename))
        end_time = rospy.get_time() + 60 * float(options.duration)
        while not rospy.is_shutdown() and rospy.get_time() < end_time:
            sleep(0.1)
            launcher.spin_once()

        rospy.loginfo("Shutting down file %s" % os.path.basename(my_filename))
        launcher.shutdown()

        index += 1
        if index >= len(args):
            index = 1

    # Should probably publish some type of message explaining everything's OK at some point
        
        
