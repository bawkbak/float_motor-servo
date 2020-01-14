#!/usr/bin/env python
import rospy
from spaceship_msgs.msg import MotorsCmd, Twist2D
from numpy import *
import yaml
import time
import os.path

# Inverse Kinematics Node
# Author: Robert Katzschmann, Shih-Yuan Liu

class InverseKinematicsNode(object):
    def __init__(self):
	# Get node name and vehicle name
        self.node_name = rospy.get_name()

        # Set local variable by reading parameters

        self.gain = self.setup_parameter("~gain", 0.6)
        self.trim = self.setup_parameter("~trim", 0.0)
        self.baseline = self.setup_parameter("~baseline", 0.1)
        self.radius = self.setup_parameter("~radius", 0.6)
        self.k = self.setup_parameter("~k", 27.0)
        self.limit = self.setup_parameter("~limit", 0.8)
        self.limit_max = 1.0
        self.limit_min = 0.0

        self.v_max = 999.0     # TODO: Calculate v_max !
        self.omega_max = 999.0     # TODO: Calculate v_max !

        # Setup the publisher and subscriber
        self.sub_vel_cmd = rospy.Subscriber("/spaceship/vel_cmd", Twist2D, self.vel_cmd_callback)
        self.pub_motors_cmd = rospy.Publisher("/spaceship/motors_cmd", MotorsCmd, queue_size=1)

        rospy.loginfo("[%s] Initialized.", self.node_name)
        #self.printValues()


    def printValues(self):
        rospy.loginfo("[%s] gain: %s trim: %s baseline: %s radius: %s k: %s limit: %s" % (self.node_name, self.gain, self.trim, self.baseline, self.radius, self.k, self.limit))

    def vel_cmd_callback(self, msg_vel_cmd):
	# assuming same motor constants k for both motors
        k_r = self.k
        k_l = self.k

        # adjusting k by gain and trim
        k_r_inv = (self.gain + self.trim) / k_r
        k_l_inv = (self.gain - self.trim) / k_l

	omega_para = 1

        omega_r = (msg_vel_cmd.v + omega_para * msg_vel_cmd.omega ) / self.radius
        omega_l = (msg_vel_cmd.v - omega_para * msg_vel_cmd.omega ) / self.radius

        # conversion from motor rotation rate to duty cycle
        # u_r = (gain + trim) (v + 0.5 * omega * b) / (r * k_r)

        u_r = omega_r * k_r_inv
        # u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
        u_l = omega_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = max(min(u_r, self.limit), -self.limit)
        u_l_limited = max(min(u_l, self.limit), -self.limit)

	# Allow the motors can do straight turn
	if u_r_limited != 0:
	    if (u_r_limited > 0) and (u_r_limited < 0.32):
		u_r_limited = 0.32
            if (u_r_limited < 0) and (u_r_limited > -0.32):
                u_r_limited = -0.32
	if u_l_limited != 0:
            if (u_l_limited > 0) and (u_l_limited < 0.32):
                u_l_limited = 0.32
            if (u_l_limited < 0) and (u_l_limited > -0.32):
                u_l_limited = -0.32

	print('%f, %f' %(u_r_limited, u_l_limited))

        # Put the wheel commands in a message and publish
        msg_motors_cmd = MotorsCmd()
        msg_motors_cmd.vel_front_right = u_r_limited
        msg_motors_cmd.vel_front_left = u_l_limited
        self.pub_motors_cmd.publish(msg_motors_cmd)

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == '__main__':
    rospy.init_node('inverse_kinematics_node', anonymous=False)
    inverse_kinematics_node = InverseKinematicsNode()
    rospy.spin()
