#!/usr/bin/env python
import sys
import time
import os
import rospy
import signal
import Adafruit_ADS1x15
from std_msgs.msg import Empty
from sensor_msgs.msg import Range


class IRFinder(object):
    def __init__(self):
        self.adc = Adafruit_ADS1x15.ADS1115()
	self.GAIN = 1
        self.distance = 0
        # values used to define the slope and intercept of
        # distance as a function of voltage
        self.m = 5e06
        self.p = -1.33

	### Publisher ###
	self.pub = rospy.Publisher("/infrared", Range, queue_size=1)


    def get_range(self):
        """Read the data from the adc and update the distance and
        smoothed_distance values."""
        voltage = self.adc.read_adc(0, self.GAIN, data_rate = 860)
        if voltage <= 0:
            voltage = 1
            print "ERROR: BAD VOLTAGE!!!"
#        self.distance = pow(voltage, self.p) * self.m
        self.distance = voltage


#	print("distance = ", self.distance)
#	self.publish_range(self.distance)

    def publish_range(self, range):
        """Create and publish the Range message to publisher."""
        msg = Range()
        msg.max_range = 0.8
        msg.min_range = 0
        msg.range = range
        msg.header.frame_id = "base"
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)

#	time.sleep(150)
#	self.get_range()

    def on_shutdown(self):
        """Gracefully quit the infrared_pub node"""
        print "\nCaught ctrl-c! Stopping node."

if __name__ == '__main__':
    rospy.init_node('infrared_pub', anonymous=False)
    node = IRFinder()
    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
#    node.get_range()
    # Keep it spinning to keep the node alive
    while not rospy.is_shutdown():
#        ir.heartbeat_pub.publish(Empty())
        node.get_range()
        node.publish_range(node.distance)
	time.sleep(0.01)
