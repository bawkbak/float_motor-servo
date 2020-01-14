#!/usr/bin/env python
import rospy
import math
from motor_hat_driver import MotorHatDriver

class ControlNode(object):
	def __init__(self):
		self.motors = MotorHatDriver()
		#self.motors.setMotorSpeed(left_x = 0, left_up = 0, right_x = 0, right_up = 10)
	
if __name__ == "__main__":
	rospy.init_node("control_node", anonymous = True)
	node = ControlNode()
	rospy.loginfo("10 pwm")
	node.motors.setMotorSpeed(left_x = 0, left_up = 0.1, right_x = 0, right_up = 0.1)
	rospy.sleep(5)
	rospy.loginfo("250 pwm")
	node.motors.setMotorSpeed(left_x = 0, left_up = 0.4, right_x = 0, right_up = 0.9)
	rospy.sleep(5)
	rospy.loginfo("0 pwm")
	node.motors.setMotorSpeed(left_x = 0, left_up = 0, right_x = 0, right_up = 0)
	rospy.sleep(5)
	rospy.spin()
