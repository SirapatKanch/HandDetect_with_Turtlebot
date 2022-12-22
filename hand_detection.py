#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import rospkg
from yolo_hand_detection.srv import Hand_Coor , Hand_CoorResponse

path = rospkg.RosPack().get_path("yolo_hand_detection")

os.chdir(path)

class ObjectDetection(object):

    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("object_detect", anonymous=True)
        rospy.Subscriber("/camera/rgb/image_color", Image, self.update_frame_callback)
        rospy.wait_for_message("/camera/rgb/image_color", Image)

    def update_frame_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8") 

    def main(self):
        net = cv2.dnn.readNet("weight/cross-hands.weights", "cfg/cross-hands.cfg")
        classes = []
        with open("cfg/coco.names", "r") as f:
            classes = [line.strip() for line in f.readlines()]
            print(f"class = {classes}") 

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
            font = cv2.FONT_HERSHEY_PLAIN
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
                        x, y, w, h = boxes[0]
                        self.find_center(x, y, w, h)
                        label = str(classes[class_ids[0]])
                        cv2.rectangle(frame, (x,y), (x+w, y+h), color, 2)
                        cv2.putText(frame, label, (x, y - 5), font, 1, color, 1)
                        cv2.imshow("Image", frame)
                        key = cv2.waitKey(1)
                        if key == 27:
                            break
    def find_center(self,x,y,w,h):
        rospy.wait_for_service('hand_coor')
        try:
            coor = rospy.ServiceProxy('hand_coor', Hand_Coor)
            resp1 = coor(x,y,w,h)
            print(resp1)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            '''
            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = str(classes[class_ids[i]])
                    cv2.rectangle(frame, (x,y), (x+w, y+h), color, 2)
                    cv2.putText(frame, label, (x, y - 5), font, 1, color, 1)
            '''

if __name__ == "__main__":
    obj = ObjectDetection()
    obj.main()
