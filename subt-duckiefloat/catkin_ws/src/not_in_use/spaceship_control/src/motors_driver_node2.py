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

        self.motorhat = Adafruit_PCA9685.PCA9685()

        self.pwm_freq = 500
        self.motorhat.set_pwm_freq(self.pwm_freq)

	# [dir, v, forward esc, backward esc]
        self.left = np.array([0, 0, 0, 1])
	self.right = np.array([0, 0, 2, 3])

	# Altitude Control
	self.driver = DaguWheelsDriver()
	self.block = False
	self.throttle = 0.0
	self.showTime = 0
	self.altitude_gain = 10.0
	self.throttle_limit = 200

	self.shutDown = False

	self.offset = 1100
	self.interval = 800

	self.pauseTime = 0.02

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

    def set_pwm_pulsewidth(self, channel, pulsewidth):
	pulsewidth = (min(max(pulsewidth, 1000), 2000))
        pwmValue = int (4096 * pulsewidth/2000)
        self.motorhat.set_pwm(channel, 0, pwmValue)

    def estop(self, msg):
	if msg.stop:
	    if self.shutDown:
		self.shutDown = False
	    else:
		self.shutDown = True
	    self.shutDownAll()

	#self.shutDown = msg.stop
	#self.shutDownAll()

    def setup(self, channel):
	self.motorhat.set_pwm(channel, 0, 4096)
	time.sleep(self.pauseTime)
	self.motorhat.set_pwm(channel, 0, 2048)
	time.sleep(self.pauseTime)

    def cdAltitudeCmd(self, msg):
	if self.shutDown:
	    self.driver.setThrottle(throttle = 0.0)
	    self.throttle = 0.0
	    return
	if self.block:
	    return
	self.block = True	

	throttle = self.throttle + self.altitude_gain*msg.vel_up

	throttle = min(self.throttle_limit, max(-1*self.throttle_limit, throttle))
	self.driver.setThrottle(throttle)
	
	self.throttle = throttle

	self.block = False

    def cbMotorsCmd(self,msg):
	if self.shutDown:
		print("!!!shut down!!!")
		self.shutDownAll()
		return

	v_left = msg.vel_front_left * self.interval
	v_right = msg.vel_front_right * self.interval
	
	if self.showTime == 10:
		print("left: ", v_left,"right: ", v_right)
		self.showTime = 0

	self.setup2(v_right, self.right)
	self.setup2(v_left, self.left)

	self.set_velocity(self.left)
	self.set_velocity(self.right)
	
	self.showTime = self.showTime+1
    def set_velocity(self, data):
	if data[0] > 0:
	    self.set_pwm_pulsewidth(data[2], data[1])
	elif data[0] < 0:
	    self.set_pwm_pulsewidth(data[3], data[1])	

    def setup2(self, v, data):
	if v > 0:# forward
	    if data[0] <= 0:
		self.set_pwm_pulsewidth(data[3], 0)
		time.sleep(self.pauseTime)
		self.setup(data[2])
	    data[0] = 1
	    
	elif v < 0:
	    if data[0] >= 0:
		self.set_pwm_pulsewidth(data[2], 0)
		time.sleep(self.pauseTime)
		self.setup(data[3])
	    data[0] = -1
	data[1] = abs(v) + self.offset

    def	shutDownAll(self):
	for i in range(15):
	    self.set_pwm_pulsewidth(i, 0)

    def on_shutdown(self):
        #self.driver.setWheelsSpeed(left=0.0,right=0.0)
	self.shutDownAll()

	self.set_pwm_pulsewidth(self.left_up_esc, 0)
	self.set_pwm_pulsewidth(self.right_up_esc, 0)
        rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    rospy.init_node('motors_driver_node', anonymous=False)
    node = MotorsDriverNode()
    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
