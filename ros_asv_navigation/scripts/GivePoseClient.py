#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Quaternion


# this method will make the robot move to the goal location
def move_to_goal(uGoal):

    # define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")

    goal = MoveBaseGoal()

    # set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # moving towards the goal*/

    goal.target_pose.pose.position = Point(uGoal[0], uGoal[1], uGoal[2])
    goal.target_pose.pose.orientation = Quaternion(
        uGoal[3], uGoal[4], uGoal[5], uGoal[6])

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)

    ac.wait_for_result(rospy.Duration(120))

    if(ac.get_state() == GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the destination")
        return True
    else:
        rospy.loginfo("The robot failed to reach the destination")
        return False


if __name__ == '__main__':
    rospy.init_node('map_navigation', anonymous=False)
    goals = [-2.02880191803, 4.02200937271, 0, 0, 0, 0, 1]
    print('start go to goal')
    move_to_goal(goals)
    rospy.spin()
