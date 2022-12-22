#!/usr/bin/env python3
from yolo_hand_detection.srv import Hand_Coor , Hand_CoorResponse
import rospy
def server_callback(req):
    x_coor_center = req.x+(req.w/2)
    y_coor_center = req.y+(req.h/2)
    return Hand_CoorResponse(x_coor_center,y_coor_center)
def trigger_server():
    rospy.init_node('trigger_server')
    s = rospy.Service('hand_coor', Hand_Coor, server_callback)
    print("Ready to do something.")
    rospy.spin()
if __name__ == "__main__":
    trigger_server()
