#!/usr/bin/env python
import rospy
import math
from motor_hat_driver import MotorHatDriver
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray


class ControlNode(object):
	def __init__(self):
		self.motors = MotorHatDriver()
		self.sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.cmdCallback, queue_size = 1)
		self.pub_cmd_servo = rospy.Publisher("/command", Int32MultiArray, queue_size = 1)

	def cmdCallback(self, msg):
		servo_msg = Int32MultiArray()
		raw_linear_x = msg.linear.x
		raw_linear_z = msg.linear.z
		raw_angular_z = msg.angular.z
		#angulat_z > 0 == left / angular_z < 0 == right
		

		if(raw_linear_x >= 0):
			raw_angle_pitch = math.atan2(raw_linear_z, raw_linear_x)
			mapping_angular = raw_angle_pitch/3.1415926*180*31 + 5200
			mapping_angular_rev = 5200 - raw_angle_pitch/3.1415926*180*31
			servo_msg.data = [mapping_angular_rev, mapping_angular, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
			self.pub_cmd_servo.publish(servo_msg)


			motor_flank = math.fabs(math.sqrt((math.pow(raw_linear_z, 2) + math.pow(raw_linear_x, 2))))
			
			motor_rear = raw_angular_z

			if(motor_rear > 0):
				#left
				motor_rear = math.fabs(motor_rear)*-1
				self.motors.setMotorSpeed(left_x = 0, left_up = motor_flank, right_x = motor_rear, right_up = motor_flank)

			elif(motor_rear < 0):
				#right
				motor_rear = math.fabs(motor_rear)
				
				self.motors.setMotorSpeed(left_x = motor_rear, left_up = motor_flank, right_x = 0, right_up = motor_flank)

			elif(motor_rear == 0 or raw_angle_pitch == 0):
				self.motors.setMotorSpeed(left_x = 0, left_up = motor_flank, right_x = 0, right_up = motor_flank)

			#self.motors.setMotorSpeed(left, flank, right, flank)

			rospy.loginfo("flank speed : %f\n", motor_flank)
			rospy.loginfo("rear speed: %f\n", motor_rear)
			rospy.loginfo("pitch angle: %f\n", mapping_angular)
			


	
if __name__ == "__main__":
	rospy.init_node("control_node", anonymous = True)
	node = ControlNode()
	rospy.spin()


