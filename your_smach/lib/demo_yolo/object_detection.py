#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image,PointCloud
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import os
import rospkg
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import *
#from demo_yolo.srv import hand_coor , hand_coorResponse

path = rospkg.RosPack().get_path("demo_yolo")

os.chdir(path)

class ObjectDetection(object):

    def __init__(self):
        self.bridge = CvBridge()
        #rospy.init_node("object_detect", anonymous=True)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.update_frame_callback)
        rospy.wait_for_message("/camera/rgb/image_raw", Image)
        #rospy.Subscriber("/camera/depth_registed/image", PointCloud, self.update_frame_depth_callback)
        #rospy.wait_for_message("/camera/depth_registered/image", PointCloud)
        self.rate = rospy.Rate(10)
    def update_frame_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8") 
        
    def update_frame_depth_callback(self, data):
        #self.depth = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        print(f"Depth = {data}")

    def main(self):
        pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
        rot = Twist()
        rot.angular.z = 0.5
        net = cv2.dnn.readNet("/home/sirapat/tutorial_ws/src/your_smach/lib/demo_yolo/weight/yolov3.weights", "/home/sirapat/tutorial_ws/src/your_smach/lib/demo_yolo/cfg/yolov3.cfg")
        classes = []
        with open("/home/sirapat/tutorial_ws/src/your_smach/lib/demo_yolo/cfg/coco.names", "r") as f:
            classes = [line.strip() for line in f.readlines()] 

        output_layers = [layer_name for layer_name in net.getUnconnectedOutLayersNames()]
        color = [162,0,37]
        
        while not rospy.is_shutdown():
            pub.publish(rot)
            self.rate.sleep()
            frame = self.image
            height, width, channels = frame.shape
            blob = cv2.dnn.blobFromImage(frame, scalefactor=0.00392, size=(320, 320), mean=(0, 0, 0), swapRB=True, crop=False)
            net.setInput(blob)
            outputs = net.forward(output_layers)
            boxes = []
            confs = []
            class_ids = []
            label = " "
            for output in outputs:
                for detect in output:
                    scores = detect[5:]
                    class_id = np.argmax(scores)
                    conf = scores[class_id]
                    if conf > 0.3:
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

            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = str(classes[class_ids[i]])
                    print(label)
                    if label=='person':
                        #center_x,center_y = rospy.Service(x,y,w,h)
                        cv2.rectangle(frame, (x,y), (x+w, y+h), color, 2)
                        cv2.putText(frame, label, (x, y - 5), font, 1, color, 1)
            cv2.imshow("Image", frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
            if label=='person':
                return 'Have'
                
if __name__ == "__main__":
    obj = ObjectDetection()
    obj.main()
