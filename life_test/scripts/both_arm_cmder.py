#! /usr/bin/env python
#
# Copyright (c) 2009, Willow Garage, Inc.
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
##\brief Sends goals to arm to move it in collision free way

PKG = 'life_test'
import roslib; roslib.load_manifest(PKG)
import rospy
import actionlib
import random

from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint

r_ranges = {
    'r_shoulder_pan_joint': (-2.0, 0.4),
    'r_shoulder_lift_joint': (-0.4, 1.25),
    'r_upper_arm_roll_joint': (-3.65, 0.45),
    'r_elbow_flex_joint': (-2.0, -0.05),
    'r_forearm_roll_joint': (-3.14, 3.14),
    'r_wrist_flex_joint': (-1.8, -0.2),
    'r_wrist_roll_joint': (-3.14, 3.14)
}


l_ranges = {
    'l_shoulder_pan_joint': (-0.4, 2.0),
    'l_shoulder_lift_joint': (-0.4, 1.25),
    'l_upper_arm_roll_joint': (-0.45, 3.65),
    'l_elbow_flex_joint': (-2.0, -0.05),
    'l_forearm_roll_joint': (-3.14, 3.14),
    'l_wrist_flex_joint': (-1.8, -0.2),
    'l_wrist_roll_joint': (-3.14, 3.14)
}


# R pan: (-2.0, 0.4)
# L pan: (-0.4, 2.0)

if __name__ == '__main__':
    rospy.init_node('arm_cmder_client')
    r_client = actionlib.SimpleActionClient('collision_free_arm_trajectory_action_right_arm', 
                                          JointTrajectoryAction)
    rospy.loginfo('Waiting for server for right collision free arm commander')
    r_client.wait_for_server()

    l_client = actionlib.SimpleActionClient('collision_free_arm_trajectory_action_left_arm', 
                                          JointTrajectoryAction)
    rospy.loginfo('Waiting for server for left collision free arm commander')
    l_client.wait_for_server()

    rospy.loginfo('Right, left arm commanders ready')
    my_rate = rospy.Rate(2.0)

    while not rospy.is_shutdown():
        r_goal = JointTrajectoryGoal()
        l_goal = JointTrajectoryGoal()
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(0)

        r_goal.trajectory.points.append(point)
        l_goal.trajectory.points.append(point)

        for joint, range in r_ranges.iteritems():
            r_goal.trajectory.joint_names.append(joint)
            r_goal.trajectory.points[0].positions.append(random.uniform(range[0], range[1]))

        for joint, range in l_ranges.iteritems():
            l_goal.trajectory.joint_names.append(joint)
            l_goal.trajectory.points[0].positions.append(random.uniform(range[0], range[1]))
            
        rospy.logdebug('Sending goal to arms.')
        r_client.send_goal(r_goal)
        l_client.send_goal(l_goal)

        r_client.wait_for_result(rospy.Duration.from_sec(3))
        l_client.wait_for_result(rospy.Duration.from_sec(3))
        my_rate.sleep()
