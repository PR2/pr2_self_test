#! /usr/bin/env python

##\author Kevin Watts
##\brief Sends goals to arm to move it in collision free way

PKG = 'life_test'
import roslib; roslib.load_manifest(PKG)
import rospy
import actionlib
import random
random.seed()

from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint

FAILED_OUT = 5

ranges = {
    'r_shoulder_pan_joint': (-2.0, 0.4),
    'r_shoulder_lift_joint': (-0.4, 1.25),
    'r_upper_arm_roll_joint': (-3.7, 0.5),
    'r_elbow_flex_joint': (-2.0, -0.05) 
}

##\brief Moves arms to 0 position using unsafe trajectory
def get_recovery_goal():
    # Send to right, left arm controllers
    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration.from_sec(5)    
    
    goal = JointTrajectoryGoal()
    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.get_rostime()

    for joint in ranges.keys():
        goal.trajectory.joint_names.append(joint)
        goal.trajectory.points[0].positions.append(0)
        goal.trajectory.points[0].velocities.append(0)

    return goal

if __name__ == '__main__':
    rospy.init_node('arm_cmder_client')
    client = actionlib.SimpleActionClient('collision_free_arm_trajectory_action_right_arm', 
                                          JointTrajectoryAction)
    rospy.loginfo('Waiting for server for collision free arm commander')
    client.wait_for_server()

    rospy.loginfo('Arm goals canceled')
    my_rate = rospy.Rate(1.0)

    recovery_client = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_action',
                                                JointTrajectoryAction)

    fail_count = 0
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
        client.wait_for_result(rospy.Duration.from_sec(5))
        my_result = client.get_state()

        if my_result == GoalStatus.SUCCEEDED:
            fail_count = 0
        else:
            fail_count += 1

        if fail_count > FAILED_OUT:
            fail_count = 0
            client.cancel_goal()
            recovery_goal = get_recovery_goal()
            recovery_client.send_goal(recovery_goal)
            recovery_client.wait_for_result(rospy.Duration.from_sec(10))

        my_rate.sleep()
