#!/usr/bin/env python
import rospy
import roslib
import time
import cv2 as cv2
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

class QualRunner:
    def __init__(self):
        rospy.init_node('qualification_runner', anonymous=True) 
        self.pub = rospy.Publisher("/controller/input", Pose, queue_size=1)
        self.sub = rospy.Subscriber("/odometry/filtered", Odometry, self.get_state)
        self.current_pose = Pose()
        self.target_pose = Pose()
    def get_state(self, s):
        self.current_pose = s.pose.pose
        vector = [0.0, -0.0, 0.0] #roll, pitch, yaw
        vector_quat = quaternion_from_euler(vector[0], vector[1], vector[2])
        print("Vector: \n" + str(vector))
        self.target_pose.orientation.x = vector_quat[0]
        self.target_pose.orientation.y = vector_quat[1]
        self.target_pose.orientation.z = vector_quat[2]
        self.target_pose.orientation.w = vector_quat[3]

        print("Orientation: \n" + str(self.target_pose.orientation))
        
    def send_input(self):
        msg = self.target_pose
        self.pub.publish(msg)

    def spin(self):
        self.send_input()
        time.sleep(0.01)

def main():
    qr = QualRunner()
    while(not rospy.is_shutdown()):
        qr.spin()