#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from spaceship_msgs.msg import MotorsCmd, Twist2D
from std_msgs.msg import Float32MultiArray
import time
from simple_pid import PID

class AutonomousControlNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))
		self.sub_current_pose = rospy.Subscriber("/state_estimator_node/current_pose", Float32MultiArray, self.cbcp, queue_size = 10)
		self.sub_target_pose = rospy.Subscriber("/state_estimator_node/target_pose", Float32MultiArray, self.cbtp, queue_size = 10)
		self.pub_vel_cmd = rospy.Publisher("/spaceship/vel_cmd", Twist2D, queue_size = 1)
		
		self.target_pose = np.zeros(2)
		self.current_pose = np.zeros(2)
		self.time = -1

		#self.pid = PID(1.0, 0, 0.7, sample_time = 0.1, setpoint = 0, output_limits = [-1, 1])
		
		self.pid = PID(1.3, 0, 2, sample_time = 0.1, setpoint = 0, output_limits = [-1, 1])
		

	def cbcp(self, msg):
		self.current_pose[0] = msg.data[0]
		self.current_pose[1] = msg.data[1]
	def cbtp(self, msg):
		self.target_pose[0] = msg.data[0]
		self.target_pose[1] = msg.data[1]

	def shutdown(self):
		msg = Twist2D()
		msg.v = 0
		msg.omega = 0
		self.pub_vel_cmd.publish(msg)


if __name__ == '__main__':
	rospy.init_node('autonomous_control_node', anonymous=False)
	node = AutonomousControlNode()
	rospy.on_shutdown(node.shutdown)
	vel_count = 0
	start_time = -1
	while not rospy.is_shutdown():
		#get loop time
		if node.time == -1:
			node.time = time.time()
		else:
			node.pid.sample_time = time.time()-node.time
			node.time = time.time()
		node.pid.setpoint = node.target_pose[0]+node.current_pose[1]*0.6
		control = node.pid(node.current_pose[0])
		print control
		msg = Twist2D()
		if abs(node.current_pose[0]-node.pid.setpoint)<0.05:
			msg.v = 0.15
			#if vel_count < 30:
			#	msg.v = 0.2
			#	vel_count+=1
			#else:
			#	msg.v = 0
		elif abs(node.current_pose[0]-node.pid.setpoint)>0.3 and abs(node.current_pose[0]-node.pid.setpoint)<0.4:
			msg.v = -0.4
		elif abs(node.current_pose[0]-node.pid.setpoint)>0.4:
			msg.v = -0.8
		else:
			msg.v = 0.1
			#if vel_count < 30:
			#	msg.v = 0.1
			#	vel_count+=1
			#else:
			#	msg.v = 0
		if vel_count > 30 and rospy.time.now()-start_time > 10:
			start_time = rospy.time.now()
			vel_count = 0
		msg.omega = -control
		node.pub_vel_cmd.publish(msg)
