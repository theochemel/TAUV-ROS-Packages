# keyboard_controller.py
#
# Simple controller that publishes a force wrench based on keyboard input
# Primarily for simulator testing and use as an example controller
#
#!/usr/bin/env python
import rospy
import time
from pynput import keyboard
import atexit
import curses
from geometry_msgs.msg import Accel
from std_msgs.msg import Header


class UserController:
    def __init__(self):
        self.gain = [30, 30, 60, 20, 20, 8]      
        rospy.init_node('keyboard_controller', anonymous=True)
        self.pub = rospy.Publisher("cmd_acc", Accel, queue_size=1)

        self.output_frame = rospy.get_param("~output_frame")

        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()
        self.keys = dict()
        self.vector = [0, 0, 0, 0, 0, 0]

    def on_press(self, key):
        k = str(key)
        k = k.strip("u\'")
        self.keys[k] = True

    def on_release(self, key):
        k = str(key)
        k = k.strip("u\'")
        self.keys[k] = False

    def is_pressed(self, key):
        if key in self.keys and self.keys[key] == True:
            return True
        return False

    def get_input(self):
        self.vector = [0, 0, 0, 0, 0, 0]
        if self.is_pressed("w"):
            self.vector[0] = self.gain[0]
        if self.is_pressed("s"):
            self.vector[0] = -self.gain[0]
        if self.is_pressed("a"):
            self.vector[1] = self.gain[1]
        if self.is_pressed("d"):
            self.vector[1] = -self.gain[1]
        if self.is_pressed("r"):
            self.vector[2] = self.gain[2]
        if self.is_pressed("f"):
            self.vector[2] = -self.gain[2]
        if self.is_pressed("Key.down"):
            self.vector[4] = -self.gain[4]
        if self.is_pressed("Key.up"):
            self.vector[4] = self.gain[4]
        if self.is_pressed("Key.left"):
            self.vector[5] = self.gain[5]
        if self.is_pressed("Key.right"):
            self.vector[5] = -self.gain[5]
        if self.is_pressed("e"):
            self.vector[3] = self.gain[3]
        if self.is_pressed("q"):
            self.vector[3] = -self.gain[3]        

    def send_thrust(self):
        command = self.vector
        msg = Accel()
        # msg.header = Header()
        # #msg.header.stamp = rospy.Time.now()
        # msg.header.frame_id = self.output_frame
        msg.linear.x = command[0]
        msg.linear.y = command[1]
        msg.linear.z = command[2]
        msg.angular.x = command[3]
        msg.angular.y = command[4]
        msg.angular.z = command[5]

        self.pub.publish(msg)

    def spin(self):
        self.get_input()
        self.send_thrust()
        time.sleep(0.01)


def shutdownHandler():
    curses.endwin()


def main():
    stdscr = curses.initscr()
    curses.noecho()
    atexit.register(shutdownHandler)
    uc = UserController()
    while not rospy.is_shutdown():
        uc.spin()
