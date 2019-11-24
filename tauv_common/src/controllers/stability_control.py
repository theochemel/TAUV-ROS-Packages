#!/usr/bin/env python
# Author: Alice Lai 2019

"""

tf transform implementation based on:
        wiki.ros.org/tf/TfUsingPython#TransformListener

Note: must manually publish a std_msgs/Float64 to /setpoint for now
"""
import roslib
import rospy
import numpy
import geometry_msgs.msg
import std_msgs.msg
from   std_msgs.msg import Float64
import sensor_msgs.msg
import time
import sys
import tf

pitch_output = 0.0

# subscriber callback for pitch ros pid controller
def pitch_callback(data):
    global pitch_output
    pitch_output = data.data

def stability_control():
    rospy.init_node('stability_control', anonymous=True)
    thrust_pub = rospy.Publisher('/albatross/thruster_manager/input',
                                geometry_msgs.msg.Wrench, queue_size=1)

    pid_pub = rospy.Publisher('/state', Float64, queue_size=10)
    pid_sub = rospy.Subscriber('/pid_output', Float64, pitch_callback)
    tf_sub = tf.TransformListener()
    # arbitrary rate
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        if (tf_sub.frameExists("albatross/base_link") and
                            tf_sub.frameExists("albatross/base_stabilized")):

            t = tf_sub.getLatestCommonTime("albatross/base_link",
                                           "albatross/base_stabilized")
            pos, quat = tf_sub.lookupTransform("albatross/base_stabilized",
                                           "albatross/base_link", t)

            # get transform from current frame to base stabilized frame
            # put transform data into PoseStamped() message

            p_data = geometry_msgs.msg.PoseStamped()
            p_data.header.frame_id = "albatross/base_link"
            p_data.header.stamp = t

            p_data.pose.orientation.x = quat[0]
            p_data.pose.orientation.y = quat[1]
            p_data.pose.orientation.z = quat[2]
            p_data.pose.orientation.w = quat[3]

            p_data.pose.position.x    = pos[0]
            p_data.pose.position.y    = pos[1]
            p_data.pose.position.z    = pos[2]


            # transform current pose data into base_stabilized frame
            # publish to rospid node for pitch
            p_base = tf_sub.transformPose("albatross/base_stabilized", p_data)
            p_base_pitch = std_msgs.msg.Float64()
            p_base_pitch.data = p_base.pose.orientation.y
            pid_pub.publish(p_base_pitch)

            # idk why this is here
            time.sleep(1)

            new_wrench = geometry_msgs.msg.Wrench()
            new_wrench.torque.y = pitch_output

            new_wrench.torque.x = 0.0
            new_wrench.torque.z = 0.0
            new_wrench.force.x = 0.0
            new_wrench.force.y = 0.0
            new_wrench.force.z = 0.0
            thrust_pub.publish(new_wrench)

            # print("Pitch of Albatross in base_stabilized frame: ",
                            # p_base.pose.orientation.y)
            # print("Position of Albatross in IMU frame:")
            # print(p_base)
        else:
            if ((tf_sub.frameExists("albatross/base_link") == False)):
                print("albatross/base_link frame does not exist")
            else:
                print("albatross/base_stabilized frame does not exist")
        rate.sleep()

def main():
    node = stability_control()
