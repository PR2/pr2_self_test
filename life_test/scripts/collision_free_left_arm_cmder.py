#! /usr/bin/env python

##\author Kevin Watts
##\brief Sends goals to left arm to move it in collision free way

PKG = 'life_test'
import roslib; roslib.load_manifest(PKG)
import rospy
import actionlib
import random
random.seed()

from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint

ranges = {
    'l_shoulder_pan_joint': (-0.4, 2.0),
    'l_shoulder_lift_joint': (-0.4, 1.25),
    'l_upper_arm_roll_joint': (-0.5, 3.7),
    'l_elbow_flex_joint': (-2.0, -0.05) 
}

if __name__ == '__main__':
    rospy.init_node('arm_cmder_client')
    client = actionlib.SimpleActionClient('collision_free_arm_trajectory_action_left_arm', 
                                          JointTrajectoryAction)
    rospy.loginfo('Waiting for server for collision free arm commander')
    client.wait_for_server()

    rospy.loginfo('Arm goals canceled')
    my_rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        goal = JointTrajectoryGoal()
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(0)

        goal.trajectory.points.append(point)

        for joint, range in ranges.iteritems():
            goal.trajectory.joint_names.append(joint)
                        
            goal.trajectory.points[0].positions.append(random.uniform(range[0], range[1]))
            
        rospy.logdebug('Sending goal to arm.')
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(3))
        my_rate.sleep()
