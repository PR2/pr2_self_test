#! /usr/bin/env python
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
##\brief Sends goals to arm to move it in collision free way

PKG = 'life_test'
import roslib; roslib.load_manifest(PKG)
import rospy
import actionlib
import random
import copy
import math
from time import sleep

from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint
from mapping_msgs.msg import CollisionObject, CollisionObjectOperation
from geometric_shapes_msgs.msg import Shape
from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatus

COLLISION_TOPIC = "collision_object"
FAILED_OUT = 10

ranges = {
    'r_shoulder_pan_joint': (-2.0, 0.4),
    'r_shoulder_lift_joint': (-0.4, 1.25),
    'r_upper_arm_roll_joint': (-3.65, 0.45),
    'r_elbow_flex_joint': (-2.0, -0.05),
    'r_forearm_roll_joint': (-3.14, 3.14),
    'r_wrist_flex_joint': (-1.8, -0.2),
    'r_wrist_roll_joint': (-3.14, 3.14),
    'l_shoulder_pan_joint': (-0.4, 2.0),
    'l_shoulder_lift_joint': (-0.4, 1.25),
    'l_upper_arm_roll_joint': (-0.45, 3.65),
    'l_elbow_flex_joint': (-2.0, -0.05),
    'l_forearm_roll_joint': (-3.14, 3.14),
    'l_wrist_flex_joint': (-1.8, -0.2),
    'l_wrist_roll_joint': (-3.14, 3.14)
}

##\brief Sets up a virtual table in front of the robot
def get_virtual_table(height = 0.42):
    table_msg = CollisionObject()

    table_msg.operation.operation = CollisionObjectOperation.ADD    

    table_msg.header.stamp = rospy.get_rostime()
    table_msg.header.frame_id = "base_footprint"
    
    side_box = Shape()
    side_box.type = Shape.BOX
    side_box.dimensions = [ 3.0, 1.0, height ]

    front_box = Shape()
    front_box.type = Shape.BOX
    front_box.dimensions = [ 1.0, 3.0, height ]

    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = height / 2
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1

    l_side_pose = copy.deepcopy(pose)
    l_side_pose.position.y = 0.85

    r_side_pose = copy.deepcopy(pose)
    r_side_pose.position.y = -0.85

    front_pose = copy.deepcopy(pose)
    front_pose.position.x = 0.85

    table_msg.shapes = [ side_box, side_box, front_box ]
    table_msg.poses = [ l_side_pose, r_side_pose, front_pose ]

    return table_msg

##\brief Moves arms to 0 position using unsafe trajectory
def get_recovery_goal():
    # Send to right, left arm controllers
    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration.from_sec(0)    
    
    goal = JointTrajectoryGoal()
    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.get_rostime()

    positions = {}
    for joint, range in ranges.iteritems():
        positions[joint] = random.uniform(range[0], range[1])
    positions['r_shoulder_pan_joint'] = - math.pi / 4
    positions['l_shoulder_pan_joint'] = math.pi / 4

    for joint, position in positions.iteritems():
        goal.trajectory.joint_names.append(joint)
        goal.trajectory.points[0].positions.append(position)
        goal.trajectory.points[0].velocities.append(0)

    return goal

def get_both_arms_goal():
    goal = JointTrajectoryGoal()
    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration.from_sec(0)
    
    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.get_rostime()    

    positions = {}
    for joint, range in ranges.iteritems():
        positions[joint] = random.uniform(range[0], range[1])
    positions['r_shoulder_pan_joint'] = positions['l_shoulder_pan_joint'] - math.pi / 2

    for joint, range in ranges.iteritems():
        goal.trajectory.joint_names.append(joint)
        goal.trajectory.points[0].positions.append(positions[joint])

    return goal

if __name__ == '__main__':
    rospy.init_node('arm_cmder_client')
    client = actionlib.SimpleActionClient('collision_free_arm_trajectory_action_both_arms', 
                                          JointTrajectoryAction)
    rospy.loginfo('Waiting for server for right collision free arm commander')
    client.wait_for_server()

    rospy.loginfo('Right, left arm commanders ready')
    #my_rate = rospy.Rate(0.5)

    table_pub = rospy.Publisher(COLLISION_TOPIC, CollisionObject, latch = True)
    table_pub.publish(get_virtual_table())

    # Recovery trajectory clients
    recovery_client = actionlib.SimpleActionClient('both_arms_controller/joint_trajectory_action',
                                                JointTrajectoryAction)
    
    fail_count = 0
    while not rospy.is_shutdown():
        rospy.logdebug('Sending goal to arms.')   
        goal = get_both_arms_goal()
        client.send_goal(goal)

        # Wait for result, or wait 10 seconds to allow full travel
        client.wait_for_result(rospy.Duration.from_sec(10))
        my_result = client.get_state()
        
        if my_result == GoalStatus.SUCCEEDED:
            fail_count = 0
	    rospy.loginfo('Goal succeeded')
        else:
	    rospy.loginfo('Goal failed')
            fail_count += 1

        rospy.logdebug('Got return state: %d from goal' % my_result)
        
        if fail_count > FAILED_OUT:
	    fail_count = 0
            client.cancel_goal()

            rospy.loginfo('Sending recovery trajectory, arms are stuck')
            recovery_goal = get_recovery_goal()
            recovery_client.send_goal(recovery_goal)
            recovery_client.wait_for_result(rospy.Duration.from_sec(10))

        sleep(2.0)
        #my_rate.sleep()

