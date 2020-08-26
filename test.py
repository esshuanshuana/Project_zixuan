#!/usr/bin/env python


import rospy
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

import sys

import rospy.rostime
from rospy.rostime import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Empty

from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
import baxter_interface
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import tf
import tf2_msgs.msg
import geometry_msgs.msg

class image_converter:
    def __init__(self):
        self._image_sub = rospy.Subscriber("/cameras/left_hand_camera/image", Image, self._image_callback)
        self._bridge = CvBridge()
 
        self._original_image = None
 
        self._bin_img = None
 
        self._obj_pose_pic = None
 
        self._homomatrix = None
 
        self._color_dict = {'blue':[[90,130,30],[130,200,150]]}
	#self._color_dict = {'red':[[156,43,46],[180,255,255]]}
 
 

    def _image_callback(self, img_data):
        try:
            self._original_image = self._bridge.imgmsg_to_cv2(img_data, "bgr8")
        except CvBridgeError as e:
            print e
    
    

    def _image_process(self, color):
 
        # Convert BGR to HSV
        hsv = cv2.cvtColor(self._original_image, cv2.COLOR_BGR2HSV)

        lower_color = np.array(self._color_dict[color][0])
        upper_color = np.array(self._color_dict[color][1])
        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_color, upper_color)
 
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(self._original_image, self._original_image, mask= mask)
 
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))
        open_img = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)
 
        gray_img = cv2.cvtColor(open_img, cv2.COLOR_BGR2GRAY)
        ret, self._bin_img = cv2.threshold(gray_img, 10, 255, cv2.THRESH_BINARY)

    def _get_homomatrix(self):

        board_size = (8, 6)
 
        rospy.loginfo("Detecting the checkerboard..........")
        gray_img = cv2.cvtColor(self._original_image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray_img, board_size, None)
        if corners == None:
            return "Error No found the checkerboard"
 
        # 24.5mm
        unit = 24.5
        table_points = []
        for i in range(board_size[1]):
            for j in range(board_size[0]):
                    temp = [unit * (j + 1), unit * (i + 1)]
                    table_points.append(temp)
 
        corners = np.squeeze(np.array(corners), axis=(1,))
        table_points = np.array(table_points).astype(np.float32)

        
        self._homomatrix, status = cv2.findHomography(corners, table_points, cv2.RANSAC)
        if self.flag == 0:
            
            self.flag = 1
        print self._homomatrix
        return 1

    def _get_obj_world_pose(self, color):

        self._image_process(color)
 
        self._get_homomatrix()
 
        contours, hierarchy = cv2.findContours(self._bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #CHAIN_APPROX_NONE
        cv2.drawContours(self._original_image, contours, -1, (0, 0, 255), 3)

        contours_array = np.array(contours)
        contours_total_num = contours_array.shape[0]
 
        object_contour = contours_array[0]
        
        # Get min area rect.
        rect = cv2.minAreaRect(object_contour)
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
 
        x, y = rect[0]
        width, height = rect[1]
        angle = rect[2]
 
        

        self._obj_pose_pic = [x, y, angle]
 
        x_img = self._obj_pose_pic[0]
        y_img = self._obj_pose_pic[1]
        angle_img = self._obj_pose_pic[2]
        width_img = self._original_image.shape[1]    # 640
        height_img = self._original_image.shape[0]   # 400
 

        obj_img = np.array([x_img, y_img, 1])    #1*3 numpy
        #obj_world = np.dot(self._homomatrix, obj_img)
	print(obj_img)
 
        center_img = np.array([width_img/2.0, height_img/2.0, 1])
        #center_world = np.dot(self._homomatrix, center_img)
	print(center_img)
 
        delta_world = obj_world - center_world
        #print delta_world
 
        if delta_world[0] > 0:
            print "move right(cm): ", delta_world[0]/10.0
        else:
            print "move left(cm): ", abs(delta_world[0]/10.0)
 
        if delta_world[1] > 0:
            print "move back(cm): ", delta_world[1]/10.0
        else:
            print "move front(cm): ", abs(delta_world[1]/10.0)
 
        angle = angle_img
        delta_pose_info = [-delta_world[1]/1000.0, -delta_world[0]/1000.0, angle]
        delta_pose_info[0] = delta_pose_info[0] - 0.041
        delta_pose_info[1] = delta_pose_info[1] + 0.024
 
        return delta_pose_info

class tf_listener():
    def __init__(self):

        rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, self.frame_handler)

        self.tf_baxter = tf.TransformerROS()
        
        self.trans_left = None
        self.trans_right = None
    
 
    def frame_handler(self, tf_message):
        temp_msg = geometry_msgs.msg.TransformStamped()

        for i in range(0,len(tf_message.transforms)):
            temp_msg = tf_message.transforms[i]
            self.tf_baxter.setTransform(temp_msg)
        

        try:
            self.trans_left = self.tf_baxter.lookupTransform('torso','left_gripper',rospy.Time())
            self.trans_right = self.tf_baxter.lookupTransform('torso','right_gripper',rospy.Time())
            
    	except Exception as e:
	    print(e)
            rospy.loginfo("Trying to sync with tf............. \n")
        
    def current_pose(self, name):

        if name == 'left':
            ans = Pose()
            ans.position.x = self.trans_left[0][0]
            ans.position.y = self.trans_left[0][1]
            ans.position.z = self.trans_left[0][2]
            ans.orientation.x = self.trans_left[1][0]
            ans.orientation.y = self.trans_left[1][1]
            ans.orientation.z = self.trans_left[1][2]
            ans.orientation.w = self.trans_left[1][3]
            return ans
        else:
            ans = Pose()
            ans.position.x = self.trans_right[0][0]
            ans.position.y = self.trans_right[0][1]
            ans.position.z = self.trans_right[0][2]
            ans.orientation.x = self.trans_right[1][0]
            ans.orientation.y = self.trans_right[1][1]
            ans.orientation.z = self.trans_right[1][2]
            ans.orientation.w = self.trans_right[1][3]
            return ans
            

class baxter_ik_srv:
    def __init__(self):
        rospy.wait_for_service('/ExternalTools/left/PositionKinematicsNode/IKService')
        self.ik_service = rospy.ServiceProxy('/ExternalTools/left/PositionKinematicsNode/IKService',SolvePositionIK)

    def solve(self,pose):
        posestamped = geometry_msgs.msg.PoseStamped()
        posestamped.pose = pose
        posestamped.header.stamp = rospy.Time.now()
        posestamped.header.frame_id = 'base'
        req = SolvePositionIKRequest()
        req.pose_stamp.append(posestamped)
        rospy.wait_for_service('/ExternalTools/left/PositionKinematicsNode/IKService')
        try:
            resp = self.ik_service(req)
            if resp.isValid[0] == True:
                return resp
            else:
                rospy.logerr("Resolver without solution..........\n")
                return None
        except rospy.ServiceException as exc:
            rospy.logerr("Error requesting service:" + str(exc))

class baxter_control():
    def __init__(self):

        self.IK_srv = baxter_ik_srv()
        self.left_arm = baxter_interface.Limb('left')
        self.left_arm.set_joint_position_speed(0.2)
 

        self.left_gripper = baxter_interface.Gripper('left')
        if self.left_gripper.calibrated() == False:
            self.left_gripper.calibrate()
        self.left_gripper.open(block=True)
        self.left_gripper.set_velocity(5)
        self.left_gripper.set_moving_force(10)
        self.left_gripper.set_holding_force(5)
 

        self.left_range = baxter_interface.AnalogIO('left_hand_range')
 
    def go(self,pose):

        ik_response = self.IK_srv.solve(pose)
        try:
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            self.left_arm.move_to_joint_positions(limb_joints)
        except:
            rospy.logerr("Can't reach the target location")
 
    def go_start_position(self):
        start_pose = Pose()
        start_pose.position.x = 0.65
        start_pose.position.y = 0.14
        start_pose.position.z = 0.27
        start_pose.orientation.x = 0.0
        start_pose.orientation.y = 1.0
        start_pose.orientation.z = 0.0
        start_pose.orientation.w = 0.0
        self.go(start_pose)


    

if __name__ == '__main__':
    rospy.loginfo("hello")
    try:
        rospy.init_node('pick_demo_no_moveit', anonymous=True)
	baxter_ctrl = baxter_control()
    	visual_processor = image_converter()
    	pose_processor = tf_listener()
    	rospy.sleep(5)
 
    	baxter_ctrl.go_start_position()
    	delta_pose = visual_processor._get_obj_world_pose('blue')
    	tar = pose_processor.current_pose('left')
    	tar.position.x += delta_pose[0]
    	tar.position.y += delta_pose[1]
    	baxter_ctrl.go(tar)
    	tar.position.z -= 0.21
    	baxter_ctrl.go(tar)
    	baxter_ctrl.left_gripper.close(block=True)


        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
