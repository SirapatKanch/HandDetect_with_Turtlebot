#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image,PointCloud
from cv_bridge import CvBridge
import os
import rospkg

path = rospkg.RosPack().get_path("demo_yolo")

os.chdir(path)

class ObjectDetection(object):

    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("object_detect", anonymous=True)
        rospy.Subscriber("/camera/rgb/image_color", Image, self.update_frame_callback)
        rospy.wait_for_message("/camera/rgb/image_color", Image)
        #rospy.Subscriber("/camera/depth_registed/image", PointCloud, self.update_frame_depth_callback)
        #rospy.wait_for_message("/camera/depth_registered/image", PointCloud)

    def update_frame_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8") 
        
    def update_frame_depth_callback(self, data):
        #self.depth = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        print(f"Depth = {data}")

    def main(self):
        net = cv2.dnn.readNet("weight/yolov3.weights", "cfg/yolov3.cfg")
        classes = []
        with open("cfg/coco.names", "r") as f:
            classes = [line.strip() for line in f.readlines()] 

        output_layers = [layer_name for layer_name in net.getUnconnectedOutLayersNames()]
        #colors = np.random.uniform(0, 255, size=(len(classes), 3))
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
                    	#color = colors[i]
                    	cv2.rectangle(frame, (x,y), (x+w, y+h), color, 2)
                    	cv2.putText(frame, label, (x, y - 5), font, 1, color, 1)
            cv2.imshow("Image", frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
#colors = [[230.03560855 46.53242328  80.01571526]]
if __name__ == "__main__":
    obj = ObjectDetection()
    obj.main()
