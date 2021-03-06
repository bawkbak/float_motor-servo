#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from spaceship_msgs.msg import MotorsCmd
from spaceship_msgs.msg import AltitudeCmd
from spaceship_msgs.srv import EStop
import Adafruit_PCA9685
import time
from dagu_wheels_driver import DaguWheelsDriver

class MotorsDriverNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

	# Control
	self.driver = DaguWheelsDriver()
	self.block = False

	self.throttle = 0.0
	self.left_speed = 0.0
	self.right_speed = 0.0

	self.showTime = 0
	self.altitude_gain = 20.0
	self.throttle_limit = 240

	self.shutDown = False

        # Setup subscribers
        self.sub_topic = rospy.Subscriber("/spaceship/motors_cmd", MotorsCmd, self.cbMotorsCmd, queue_size=1)
	self.sub_altitude = rospy.Subscriber("/spaceship/altitude_cmd", AltitudeCmd, self.cdAltitudeCmd, queue_size=1)
#	self.sub_estop = rospy.Subscriber("/spaceship/estop", EstopCmd, self.estop, queue_size=1)
#	self.sub_altitude_pwm = rospy.Subscriber("/duckiefloat/altitude_cmd", AltitudeCmd, self.cdAltitudeCmd_pwm, queue_size=1)

	self.estopService = rospy.Service('/estop', EStop, self.handle_estop)

    def handle_estop(self, req):
	if req.estop_cmd == True:
	    self.shutDown = True
	    self.shutDownAll()
	    return True

	elif req.cancel_cmd == True:
	    self.shutDown = False
	    return False
	else:
	    return self.shutDown

    def estop(self, msg):
	if msg.stop:
	    if self.shutDown:
		self.shutDown = False
	    else:
		self.shutDown = True
	    self.shutDownAll()

	#self.shutDown = msg.stop
	#self.shutDownAll()

    def cdAltitudeCmd(self, msg):
	if self.shutDown:
	    self.driver.setThrottle(throttle = 0.0)
	    self.throttle = 0.0
	    return
	if self.block:
	    return
	self.block = True	

	throttle = msg.vel_up

	throttle = min(self.throttle_limit, max(-1*self.throttle_limit, throttle))
	self.driver.setThrottle(throttle)
	
	self.throttle = throttle

	self.block = False

    def cbMotorsCmd(self,msg):
	if self.shutDown:
		print("!!!shut down!!!")
		self.shutDownAll()
		return
	vl = msg.vel_front_left
	vr = msg.vel_front_right

	print(vl)

	self.driver.setWheelsSpeed(left = vl, right = vr)

    def	shutDownAll(self):
	self.driver.setThrottle(throttle = 0.0)
	self.driver.setWheelsSpeed(left=0.0,right=0.0)

    def on_shutdown(self):
	self.shutDownAll()

        rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    rospy.init_node('motors_driver_node', anonymous=False)
    node = MotorsDriverNode()
    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
