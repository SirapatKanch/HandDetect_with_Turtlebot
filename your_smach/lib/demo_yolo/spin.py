#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class Spin_Base(object):
    def __init__(self):
        #rospy.init_node('rotate_turtlebot')
        self.rate = rospy.Rate(10)
    def spin(self):
        #rospy.init_node('rotate_turtlebot')
        pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
        rot = Twist()
        rot.angular.z = 0.5
        while not rospy.is_shutdown():
            pub.publish(rot)
            self.rate.sleep()

if __name__ == "__main__":
    spin = Spin_Base()
    spin.spin()




#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Quaternion, Pose, Point
from std_msgs.msg import Float32
from your_smach.srv import Location


class go_to_goal():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)

    def go_to_location(self):
        move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        move_base.wait_for_server(rospy.Duration(5))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_footprint'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.a #3 meters
        goal.target_pose.pose.orientation.w = 1.0 #go forward
        #start moving
        move_base.send_goal(goal)
        #allow TurtleBot up to 60 seconds to complete task
        success = move_base.wait_for_result(rospy.Duration(60))
        if not success:
            move_base.cancel_goal()
            rospy.loginfo("The base failed to move forward %s meters for some reason",self.a)
        else:
            # We made it!
            state = move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("The base moved 3 meters forward")

    def shutdown(self):
        stop_goal = MoveBaseGoal()
        self.move_base.send_goal(stop_goal)
        rospy.loginfo("Stop")

    def robot_navi_server(self):
        rospy.init_node('robot_navi_server')
        s = rospy.Service('robot_following', Location, self.go_to_location)
        print("Ready to move.")
        rospy.spin()

if __name__ == '__main__':
    main = go_to_goal()
    main.robot_navi_server()