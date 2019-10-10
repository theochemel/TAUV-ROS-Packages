#!/usr/bin/env python
import rospy
import roslib
import time
import cv2 as cv
import cv2
import math
from pynput import keyboard
import sys

from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt


def template_matching(src, template):
    
    #src[:, :, 0] = 0
    frame = src.copy()
    s = src.copy()
    frame = cv.convertScaleAbs(frame, alpha=6.0, beta=0.0)
    frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    img = frame.copy()

    # Perform thresholding
    ret,thresh1 = cv.threshold(img,200,255,cv.THRESH_BINARY)
    img = thresh1

    # Perform template matching
    w, h = template.shape
    method_str = 'cv.TM_CCOEFF_NORMED'
    method = eval(method_str)
    for i in range(4):
        resize_img = cv.resize(img, None, fx=1/(2**(0.5*i)), fy=1/(2**(0.5*i)))
        result = cv.matchTemplate(resize_img, template, method)
        loc = np.where(result >= 0.75)
        for pt in zip(*loc[::-1]):
            x = (pt[0]*int(2**(0.5*i)), pt[1]*int(2**(0.5*i)))
            y = (pt[0]*int(2**(0.5*i)) + w, pt[1]*int(2**(0.5*i)) + h)
            cv.rectangle(src, x,y, (255,0,0), 1)
    cv.imshow("Matching Result", result)
    cv.imshow("Detected Image", thresh1)

template = cv.imread('/home/matthew/tartan_ws/src/TAUV-ROS-Packages/tauv_mission/src/tasks/rod_img_small.png')
print(np.shape(template))
template = cv.cvtColor(template, cv.COLOR_BGR2GRAY)

bridge = CvBridge()
class QualRunner:
    def __init__(self):
        rospy.init_node('qualification_runner', anonymous=True) 
        self.pub = rospy.Publisher("/controller/input", Pose, queue_size=1)
        self.sub = rospy.Subscriber("/odometry/filtered", Odometry, self.get_state)
        self.image_sub = rospy.Subscriber("/manta/manta/camerafront/camera_image",Image, self.get_image)
        self.current_state = 0
        self.current_pose = Pose()
        self.target_pose = Pose()
        self.image_array = []
        self.image_count = 10
        self.force = [0.0, 0.0, 0.0]
        self.vector = [0.0, 0.0, -math.pi*1.2/4] #simulator yaw angle
        self.send_input()
        #time.sleep(3) # wait until aligned

    def movement_one(self): #decrease depth until aligned with goal
        self.set_heave(0)
        self.set_surge(0)

    def movement_two(self): #move forward until goal is certain size
        pass

    def movement_three(self): #adjust angle until facing between goalposts
        pass

    def movement_four(self): # move forward through goal
        pass

    def get_image(self, data):
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_array.append(cv_image)
            if len(self.image_array) >= self.image_count:
                image = self.image_array[0].astype(np.int32)
               # print(image[0, 0, 0])
                for i in range(1, len(self.image_array)):
                #    print("Adding: " + str(image[0, 0, 0]))
                #    print("To this: " + str(self.image_array[i][0,0,0]))
                    image = np.add(image, self.image_array[i])
                #    print("Result: " + str(image[0,0,0]))
                
                #print(len(self.image_array))
                image = (image / len(self.image_array)).astype(np.uint8)
                #print(image[0, 0, 0])
                self.image_array = []

                cv.imshow('image', image)
                cv.waitKey(10)
                template_matching(image, template)
                #print(image.shape)
        except CvBridgeError as e:
            print(e)

    def set_heave(self, v):
        self.force[2] = v

    def set_surge(self, v):
        self.force[0] = v
    
    def set_sway(self, v):
        self.force[1] = v

    def set_pitch(self, v):
        self.vector[1] = v

    def set_roll(self, v):
        self.vector[0] = v

    def set_yaw(self, v): # from 0 to 2pi
        self.vector[2] = v

    def rotate_yaw(self, a): # any angle
        self.vector[2] += a

    def get_state(self, s):
        pass

    def send_input(self):
        vector_quat = quaternion_from_euler(self.vector[0], self.vector[1], self.vector[2])
        print("Target Force: \n" + str(self.force))
        print("Target Vector: \n" + str(self.vector))
        self.target_pose.position.x = self.force[0]
        self.target_pose.position.y = self.force[1]
        self.target_pose.position.z = self.force[2]
        self.target_pose.orientation.x = vector_quat[0]
        self.target_pose.orientation.y = vector_quat[1]
        self.target_pose.orientation.z = vector_quat[2]
        self.target_pose.orientation.w = vector_quat[3]
        print("Target Orientation: \n" + str(self.target_pose.orientation))
        msg = self.target_pose
        self.pub.publish(msg)

    def spin(self):
        if (self.current_state == 0):
            self.movement_one()
        self.send_input()
        time.sleep(0.01)

def main():
    qr = QualRunner()
    while(not rospy.is_shutdown()):
        qr.spin()