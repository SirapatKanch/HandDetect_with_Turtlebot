#!/usr/bin/env python3
import rospy
from your_package.msg import Data
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np

def callback_depth(data):
	depth_arr = np.array(data)
	print(type(depth_arr))
	
def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('/camera/depth/image',Image, callback_depth)
	rospy.spin()

if __name__ == '__main__':
	listener()
