#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import rospkg
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import *

path = rospkg.RosPack().get_path("yolo_hand_detection")

os.chdir(path)

class HandDetection(object):

    def __init__(self):
        self.bridge = CvBridge()
        #rospy.init_node("object_detect", anonymous=True)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.update_frame_callback)
        rospy.wait_for_message("/camera/rgb/image_raw", Image)
        self.rate = rospy.Rate(10)
    def update_frame_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8") 

    def main(self):
        pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
        rot = Twist()
        rot.angular.z = 0.5


        net = cv2.dnn.readNet("weight/cross-hands.weights", "cfg/cross-hands.cfg")
        classes = []
        with open("cfg/coco.names", "r") as f:
            classes = [line.strip() for line in f.readlines()]

        output_layers = [layer_name for layer_name in net.getUnconnectedOutLayersNames()]
        color = [162,0,37]
        while not rospy.is_shutdown():
            frame = self.image
            height, width, channels = frame.shape
            blob = cv2.dnn.blobFromImage(frame, scalefactor=0.00392, size=(320, 320), mean=(0, 0, 0), swapRB=True, crop=False)
            net.setInput(blob)
            outputs = net.forward(output_layers)
            boxes = []
            confs = []
            class_ids = []
            for output in outputs:
                for detect in output:
                    scores = detect[5:]
                    class_id = np.argmax(scores)
                    conf = scores[class_id]
                    if conf > 0.8:
                        center_x = int(detect[0] * width)
                        center_y = int(detect[1] * height)
                        w = int(detect[2] * width)
                        h = int(detect[3] * height)
                        x = int(center_x - w/2)
                        y = int(center_y - h / 2)
                        boxes.append([x, y, w, h])
                        confs.append(float(conf))
                        class_ids.append(class_id)
            indexes = cv2.dnn.NMSBoxes(boxes, confs, 0.5, 0.4)
            font = cv2.FONT_HERSHEY_PLAIN
            if bool(boxes) == True:
                for i in range(1):
                    if i in indexes:
                        x, y, w, h = boxes[i]
                        label = str(classes[class_ids[i]])
                        cv2.rectangle(frame, (x,y), (x+w, y+h), color, 2)
                        cv2.putText(frame, label, (x, y - 5), font, 1, color, 1)
                        #change state
            cv2.imshow("Image", frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
            if bool(boxes) == True:
                status = 'HaveHand'
                return x,y,w,h,status
if __name__ == "__main__":
    hand = HandDetection()
    hand.main()
