#!/usr/bin/env python3
import rospy
from your_package.msg import Data
from std_msgs.msg import Float32MultiArray
def callback_coor(data):
	print(data.data)
def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('CenterCoor',Float32MultiArray, callback_coor)
	rospy.spin()

if __name__ == '__main__':
	listener()
