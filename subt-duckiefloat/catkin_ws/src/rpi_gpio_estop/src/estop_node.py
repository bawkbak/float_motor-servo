#!/usr/bin/env python
import rospy
import numpy as np
import time
import RPi.GPIO as GPIO
from duckiefloat_msgs.srv import EStop

class Estop_node(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
	
	GPIO.setmode(GPIO.BCM)

	GPIO.setup(16, GPIO.IN)
#	GPIO.setup(20, GPIO.OUT)
	GPIO.setup(26, GPIO.OUT)
	
	self.cancel = False
	self.estop = False

	self.run()

    def run(self):
	while True:
	    inputValue = GPIO.input(16)
	    inputValue = False
	    if inputValue == True:
		self.estop = True
	  	self.request()
	    else:
		self.estop = False
		self.request()


    def request(self):
	rospy.wait_for_service('/estop')
	self.estopClient = rospy.ServiceProxy('/estop', EStop)
	response = self.estopClient(self.estop, False)
	if response.estop_curr == True:
#	    GPIO.output(20, GPIO.HIGH)
#	    print("GPIO ESTOP")
	    GPIO.output(26, GPIO.HIGH)
	else:
#	    GPIO.output(20, GPIO.LOW)
            GPIO.output(26, GPIO.LOW)

if __name__ == '__main__':
    rospy.init_node('estop_node', anonymous=False)
    node = Estop_node()
    # Setup proper shutdown behavior
    #rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    #rospy.spin()
