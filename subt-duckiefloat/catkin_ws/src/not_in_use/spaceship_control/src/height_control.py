#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from spaceship_msgs.msg import MotorsCmd
from spaceship_msgs.msg import AltitudeCmd
from sensor_msgs.msg import Range
import time
from simple_pid import PID

class HeightControlNode(object):
    def __init__(self):	
	self.node_name = rospy.get_name()
    	rospy.loginfo("[%s] Initializing " %(self.node_name))
    	# subscribers
    	self.sub_altitude = rospy.Subscriber("/height", Range, self.cbAltitude, queue_size = 5)
    	# publishers
    	self.pub_altitude_cmd = rospy.Publisher("/spaceship/altitude_cmd_pid", AltitudeCmd, queue_size=1)
    	#pid
    	self.pid = PID(Kp = 20000.0, Ki = 0.0, Kd = 2000.0, sample_time = 0.1, setpoint = 0.8, output_limits = [-10000, 10000])
    	self.time = -1
    	self.altitude = -1

    def cbAltitude(self,msg):
    	self.altitude = msg.range
    def shutdown(self):
    	msg = AltitudeCmd()
    	msg.vel_up = 0
    	self.pub_altitude_cmd.publish(msg)



if __name__ == '__main__':
    rospy.init_node('height_control_node', anonymous=False)
    node = HeightControlNode()
    rospy.on_shutdown(node.shutdown)
    while not rospy.is_shutdown():
    	rospy.sleep(0.1)
	#get loop time
    	if node.time == -1:
    		node.time = time.time()
                
    	else:
    		node.pid.sample_time = time.time()-node.time
		node.time = time.time()
    	control = node.pid(node.altitude)
        print "////////////", node.altitude, " ", control
    	msg = AltitudeCmd()
    	control = node.pid(node.altitude)
    	msg.vel_up = control+120
    	node.pub_altitude_cmd.publish(msg)
    	 
