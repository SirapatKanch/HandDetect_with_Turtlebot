#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
#from std_srvs.srv import *
#from your_smach.srv import GoToPosition, GoToPositionRequest
# from ../../demo_yolo.object_detection import ObjectDetection
from your_smach.srv import Hand_Coor , Hand_CoorResponse , Depth_Hand , Depth_HandResponse
from lib.demo_yolo.spin import Spin_Base
from lib.demo_yolo.object_detection import ObjectDetection
from lib.yolo_hand_detection.hand_detection import HandDetection

class FindPerson(smach.State):
    def __init__(self,
                outcomes=['success','fail'],
                input_keys=['person_x_in','person_y_in'],
                output_keys=['hand_coor_x','hand_coor_y']):
        super().__init__(outcomes,input_keys,output_keys)
    
    def execute(self, ud):
        rospy.loginfo("Executing state FindPerson")
        ud.hand_coor_x = ud.person_x_in
        ud.hand_coor_y = ud.person_y_in
        obj = ObjectDetection()
        person = obj.main()
        if person == 'Have':
            return 'success'

class FindHand(smach.State):
    def __init__(self,
                outcomes=['success', 'fail'],
                input_keys=['hand_coor_x_in','hand_coor_y_in'],
                output_keys=['hand_coor_x','hand_coor_y']):
        super().__init__(outcomes,input_keys,output_keys)
    
    def execute(self, ud):
        rospy.loginfo("Executing state FindHand")
        obj = HandDetection()
        x,y,w,h,status = obj.main()
        if status == 'HaveHand':
            client = rospy.ServiceProxy('hand_coor', Hand_Coor)
            hand_coor = client(x,y,w,h)
            ud.hand_coor_x = hand_coor.center_x
            ud.hand_coor_y = hand_coor.center_y 
            print(f"hand_coor.center_x = {hand_coor.center_x}") 
            print(f"hand_coor.center_y = {hand_coor.center_y}")            
            return 'success'

class FindDepth(smach.State):
    def __init__(self, outcomes=['success', 'fail'],
                input_keys=['hand_coor_x','hand_coor_y'],depth=0):
                #output_keys=['depth']):
        super().__init__(outcomes,input_keys)
    
    def execute(self, ud):
        rospy.loginfo("Executing state FindDepth")
        print(f"ud.hand_coor_x = {ud.hand_coor_x}")
        print(f"ud.hand_coor_y = {ud.hand_coor_y}")
        client = rospy.ServiceProxy('depth_hand', Depth_Hand)
        center = client(ud.hand_coor_x,ud.hand_coor_y)
        print(f"Depth = {center}")
        return 'success'
'''
class Navigate(smach.State):
    def __init__(self, outcomes=['success', 'fail']):
        super().__init__(outcomes)
        self.gotopos = rospy.ServiceProxy('robot/naigation', GoToPosition)
    
    def execute(self, ud):
        rospy.loginfo("Executing state Navigation")
        req = GoToPositionRequest()
        req.position_name = 'home'
        self.gotopos(req)
        return 'success'
'''
class RobotState(object):
    def __init__(self) -> None:
        rospy.init_node('robot_state', anonymous=True)
        rospy.Service('hand_coor', Hand_Coor, self.hand_server_callback)
        rospy.Service('depth_hand', Depth_Hand, self.depth_server_callback)
        sm = smach.StateMachine(outcomes=['---finish---'])
        sm.userdata.sm_data_x = 0
        sm.userdata.sm_data_y = 0
        self.depth = 0
        self.depth_cv = np.empty([640, 480])
        with sm:
            smach.StateMachine.add('FindPerson', FindPerson(), 
                               transitions={'success':'FindHand','fail':'FindPerson'},
                               remapping={'person_x_in':'sm_data_x','person_y_in':'sm_data_y','hand_coor_x':'sm_data_x','hand_coor_y':'sm_data_y'})
		                       
            smach.StateMachine.add('FindHand', FindHand(), 
                                transitions={'success':'FindDepth', 'fail':'FindHand'},
                                remapping={'hand_coor_x_in':'sm_data_x','hand_coor_y_in':'sm_data_y','hand_coor_x':'sm_data_x','hand_coor_y':'sm_data_y'})

            smach.StateMachine.add('FindDepth', FindDepth(), 
                               transitions={'success':'---finish---', 'fail':'FindDepth'},
                               remapping={'hand_coor_x':'sm_data_x',
                                            'hand_coor_y':'sm_data_y'})
                                            #'depth':'sm_data_x'})

            '''smach.StateMachine.add('Navigation', DoSth(), 
                               transitions={'success':'FindPerson', 'fail':'Navigation'})'''
            
        outcome = sm.execute()


    def hand_server_callback(self,req):
        x_coor_center = req.x+(req.w/2)
        y_coor_center = req.y+(req.h/2)
        return Hand_CoorResponse(x_coor_center,y_coor_center)

    def depth_server_callback(self,req):
        bridge = CvBridge()

        self.x = int(req.x)
        self.y = int(req.y)
        rospy.Subscriber("/camera/depth/image_rect_raw'", Image, self.update_frame_callback)
        rospy.wait_for_message("/camera/depth/image_rect_raw", Image)

        self.depth = self.depth_cv[self.x][self.y]
        print(self.depth)

        return Depth_HandResponse(self.depth)

    def update_frame_callback(self, data):
        self.depth_cv = np.array(bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')) 
        print(self.depth_cv)


if __name__ == "__main__":
    RobotState()
