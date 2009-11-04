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
##\brief Spawns controllers for caster test, used for generating controller name

import roslib
roslib.load_manifest('qualification')
import rospy, sys, time
import os.path
import signal

#from pr2_mechanism_control import mechanism
from pr2_mechanism_msgs.srv import LoadController, UnloadController, SwitchController

from xml.dom.minidom import parse, parseString
import xml.dom

def print_usage(exit_code = 0):
    print 'caster_test_spawner.py [--stopped] <controller names>'
    sys.exit(exit_code)

rospy.wait_for_service('load_controller')
load_controller = rospy.ServiceProxy('load_controller', LoadController)
unload_controller = rospy.ServiceProxy('unload_controller', UnloadController)
switch_controller = rospy.ServiceProxy('switch_controller', SwitchController)

spawned = None
prev_handler = None

def shutdown(sig, stackframe):
    global spawned

    if spawned is not None:
        switch_controller([], [spawned], 2)
        for i in range(3):
            try:
                rospy.logout("Trying to unload %s" % spawned)
                unload_controller(spawned)
                rospy.logout("Succeeded in unloading %s" % spawned)
                break
            except rospy.ServiceException:
                raise
                rospy.logerr("ServiceException while killing %s" % spawned)
    # We're shutdown.  Now invoke rospy's handler for full cleanup.
    if prev_handler is not None:
        prev_handler(signal.SIGINT,None)

def main():
    if len(sys.argv) < 4:
        print "caster_test_spawner.py <controller_name> <joint_param_name> <joint_base_name>"
        sys.exit(2)

    rospy.init_node('caster_test_spawner')

    side = rospy.get_param("caster_test/side") # 'fl', etc
    wheel = rospy.get_param("caster_test/wheel") # 'l' or 'r'

    # Override rospy's signal handling.  We'll invoke rospy's handler after
    # we're done shutting down.
    global prev_handler
    prev_handler = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, shutdown)

    # Controller name
    controller = rospy.myargv()[1]
    # Parameter that stores joint name for controller
    joint_param_name = rospy.myargv()[2] 
    # Caster base name looks like "SIDE_caster_WHEEL_wheel_joint"
    joint_base_name = rospy.myargv()[3]

    # Get full caster joint name
    joint_name = joint_base_name.replace('WHEEL', wheel).replace('SIDE', side)
    # Set the name of the joint we want to control
    rospy.set_param(joint_param_name, joint_name)

    global spawned

    resp = load_controller(controller)
    if resp.ok != 0:
        spawned = controller
        rospy.loginfo("Loaded controller: %s" % controller)
    else:
        time.sleep(1) # give error message a chance to get out
        rospy.logerr("Failed to load %s" % controller)

    resp = switch_controller([spawned], [], 2)
    if resp.ok != 0:
        rospy.loginfo("Started controllers: %s" % spawned)
    else:
        rospy.logerr("Failed to start controller: %s" % spawned)

    rospy.spin()

if __name__ == '__main__':
    main()
