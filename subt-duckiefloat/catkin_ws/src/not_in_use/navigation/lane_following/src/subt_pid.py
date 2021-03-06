#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
import tf
import struct
import math
import time
from sensor_msgs.msg import Image, LaserScan
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, Twist
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from control.cfg import pos_PIDConfig, ang_PIDConfig
from std_srvs.srv import SetBool, SetBoolResponse
from spaceship_msgs.msg import MotorsCmd, Twist2D

from PID import PID_control

class Robot_PID():
	def __init__(self):
		self.node_name = rospy.get_name()
		self.dis4constV = 3. # Distance for constant velocity
		self.pos_ctrl_max = 0.6
		self.pos_ctrl_min = 0
		self.ang_ctrl_max = 0.4
		self.ang_ctrl_min = -0.4
		self.turn_threshold = 30
		self.cmd_ctrl_max = 0.1
		self.cmd_ctrl_min = -0.1
		self.arrived_dis = 0.5 # meters
		self.frame_id = 'map'
		self.emergency_stop = False
		self.final_goal = None # The final goal that you want to arrive
		self.goal = self.final_goal
		self.cmd = Twist()
		self.spaceship_cmd = MotorsCmd()

		rospy.loginfo("[%s] Initializing " %(self.node_name))

		self.sub_goal = rospy.Subscriber("pursue_point", PoseStamped, self.goal_cb, queue_size=1)
		self.pub_spaceship_cmd = rospy.Publisher("/spaceship/motors_cmd", MotorsCmd, queue_size = 1)
		self.pub_cmd = rospy.Publisher("/husky_velocity_controller/cmd_vel", Twist, queue_size = 1)

		
		self.pub_cmd2d = rospy.Publisher("/spaceship/vel_cmd", Twist2D, queue_size = 1)
		self.cmd2d = Twist2D()
	
		self.pub_goal = rospy.Publisher("/goal_point", Marker, queue_size = 1)
		self.emergency_stop_srv = rospy.Service("/emergency_stop", SetBool, self.emergency_stop_cb)

		self.pos_control = PID_control("Position")
		self.ang_control = PID_control("Angular")

		self.pos_srv = Server(pos_PIDConfig, self.pos_pid_cb, "Position")
		self.ang_srv = Server(ang_PIDConfig, self.ang_pid_cb, "Angular")
		
		self.initialize_PID()


		while not rospy.is_shutdown():
			#self.pub_spaceship_cmd.publish(self.spaceship_cmd)
			#self.pub_cmd.publish(self.cmd)
			##### omni version #####
			self.pub_cmd2d.publish(self.cmd2d)

			if self.goal is not None:
				self.publish_goal(self.goal)
			rospy.sleep(0.1)

	def goal_cb(self, p):
		robot_position = [0, 0]
		self.final_goal = [p.pose.position.x, p.pose.position.y]
		if self.final_goal == [0, 0]:
			cmd_msg = Twist()
			cmd_msg.linear.x = 0
			cmd_msg.angular.z = 0
			self.cmd = cmd_msg
			return
		self.goal = self.final_goal
		goal_distance = self.get_distance(robot_position, self.goal)
		goal_angle = self.get_goal_angle(0, robot_position, self.goal)
		if goal_distance < self.arrived_dis or self.emergency_stop:
			rospy.loginfo("Stop!!!")
			pos_output, ang_output = (0, 0)
		else:
			pos_output, ang_output = self.control(goal_distance, goal_angle)
		
		cmd_msg = Twist()
		cmd_msg.linear.x = pos_output
		cmd_msg.angular.z = ang_output
		self.cmd = cmd_msg

		##### omni version #####
		cmd2d = Twist2D()
		cmd2d.v = pos_output + 0.2
		cmd2d.omega = ang_output
		self.cmd2d = cmd2d
		print('===',self.cmd2d)
		
		#spaceship_cmd_msg = MotorsCmd()
		#self.spaceship_cmd.vel_front_left = 1300 + (pos_output + ang_output) * 600 / (self.pos_ctrl_max + self.ang_ctrl_max)
	        #self.spaceship_cmd.vel_front_right = 1300 + (pos_output - ang_output) * 600 / ((self.pos_ctrl_max + self.ang_ctrl_max))

	def control(self, goal_distance, goal_angle):
		self.pos_control.update(goal_distance)
		self.ang_control.update(goal_angle)

		# pos_output will always be positive
		pos_output = self.pos_constrain(-self.pos_control.output/self.dis4constV)

		# -1 = -180/180 < output/180 < 180/180 = 1
		ang_output = self.ang_constrain(self.ang_control.output*3/180.)
		if abs(self.ang_control.output) > self.turn_threshold:
			if pos_output > 0.1:
				pos_output = 0.1
		return pos_output, ang_output

	def emergency_stop_cb(self, req):
		if req.data == True:
			self.emergency_stop = True
		else:
			self.emergency_stop = False
		res = SetBoolResponse()
		res.success = True
		res.message = "recieved"
		return res

	def cmd_constarin(self, input):
		if input > self.cmd_ctrl_max:
			return self.cmd_ctrl_max
		if input < self.cmd_ctrl_min:
			return self.cmd_ctrl_min
		return input

	def pos_constrain(self, input):
		if input > self.pos_ctrl_max:
			return self.pos_ctrl_max
		if input < self.pos_ctrl_min:
			return self.pos_ctrl_min
		return input

	def ang_constrain(self, input):
		if input > self.ang_ctrl_max:
			return self.ang_ctrl_max
		if input < self.ang_ctrl_min:
			return self.ang_ctrl_min
		return input

	def initialize_PID(self):
		self.pos_control.setSampleTime(1)
		self.ang_control.setSampleTime(1)

		self.pos_control.SetPoint = 0.0
		self.ang_control.SetPoint = 0.0

	def get_goal_angle(self, robot_yaw, robot, goal):
		robot_angle = np.degrees(robot_yaw)
		p1 = [robot[0], robot[1]]
		p2 = [robot[0], robot[1]+1.]
		p3 = goal
		angle = self.get_angle(p1, p2, p3)
		result = angle - robot_angle
		result = self.angle_range(-(result + 90.))
		return result

	def get_angle(self, p1, p2, p3):
		v0 = np.array(p2) - np.array(p1)
		v1 = np.array(p3) - np.array(p1)
		angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
		return np.degrees(angle)

	def angle_range(self, angle): # limit the angle to the range of [-180, 180]
		if angle > 180:
			angle = angle - 360
			angle = self.angle_range(angle)
		elif angle < -180:
			angle = angle + 360
			angle = self.angle_range(angle)
		return angle

	def get_distance(self, p1, p2):
		return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

	def publish_goal(self, goal):
		marker = Marker()
		marker.header.frame_id = self.frame_id
		marker.header.stamp = rospy.Time.now()
		marker.ns = "pure_pursuit"
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.pose.orientation.w = 1
		marker.pose.position.x = goal[0]
		marker.pose.position.y = goal[1]
		marker.id = 0
		marker.scale.x = 0.6
		marker.scale.y = 0.6
		marker.scale.z = 0.6
		marker.color.a = 1.0
		marker.color.g = 1.0
		self.pub_goal.publish(marker)

	def pos_pid_cb(self, config, level):
		print("Position: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
		Kp = float("{Kp}".format(**config))
		Ki = float("{Ki}".format(**config))
		Kd = float("{Kd}".format(**config))
		self.pos_control.setKp(Kp)
		self.pos_control.setKi(Ki)
		self.pos_control.setKd(Kd)
		return config

	def ang_pid_cb(self, config, level):
		print("Angular: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
		Kp = float("{Kp}".format(**config))
		Ki = float("{Ki}".format(**config))
		Kd = float("{Kd}".format(**config))
		self.ang_control.setKp(Kp)
		self.ang_control.setKi(Ki)
		self.ang_control.setKd(Kd)
		return config

if __name__ == '__main__':
	rospy.init_node('PID_control')
	foo = Robot_PID()
	rospy.spin()
