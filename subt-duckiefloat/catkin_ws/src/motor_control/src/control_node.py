#!/usr/bin/env python

import rospy
from motor_hat_driver import MotorHatDriver
from geometry_msgs.msg import Twist

class ControlNode(object):
    def __init__(self):
        self.motors = MotorHatDriver()

        self.sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.cb_cmd, queue_size = 1)

    def cb_cmd(self, msg):
        # rospy.loginfo("cmd_cb")
        left_x = msg.linear.x - msg.angular.z
        right_x = msg.linear.x + msg.angular.z
        
        left_up = -msg.linear.z + msg.linear.y
        right_up = -msg.linear.z - msg.linear.y

        self.motors.setMotorSpeed(left_x=left_x, left_up=left_up, right_x=right_x, right_up=right_up)


if __name__ == "__main__":
    rospy.init_node("control_node")

    node = ControlNode()
    rospy.spin()
