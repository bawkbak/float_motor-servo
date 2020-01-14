#!/usr/bin/env python
import serial
import rospy
import struct
import math
import time
import copy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Header

DEFAULT_PORT = rospy.get_param("~port","/dev/ttyUSB1")
# 0x 7E 00 0B 88 01 49 53 00 01 00 02 00 00 00 D5
HIGH_STATE = bytearray([126, 0, 11, 136, 1, 73, 83, 0, 1, 0, 2, 0, 0, 2, 213])
# 0x 7E 00 0B 88 01 49 53 00 01 00 02 00 00 00 D7
LOW_STATE = bytearray([126, 0, 11, 136, 1, 73, 83, 0, 1, 0, 2, 0, 0, 0, 215])

ESTOP_STATE = False
pub_estop = rospy.Publisher("/e_stop_xbee", Bool, queue_size=1)
serial_link = serial.Serial(DEFAULT_PORT, 9600)


def Estop_Monitor(event):
    # note = 0x 7E 00 04 08 01 49 53 5A
    note = bytearray([126, 0, 4, 8, 1, 73, 83, 90])
    print "Write to xbee: ", list(note)
    serial_link.write(note)
    time.sleep(0.1)

    status = serial_link.read(serial_link.inWaiting())
    print "Serial status: ", list(status)

    if status == LOW_STATE:
        print "Normal State"
        pub_estop.publish(False)

    elif status == HIGH_STATE:
        print "Emergency Stop !!!"
        ESTOP_STATE = True
        pub_estop.publish(True)
        
    else:
        print "Other State"


if __name__ == "__main__":
    rospy.init_node("estop_monitor", anonymous=False)
    rospy.loginfo("[%s] Initializing " % (rospy.get_name()))
    rospy.Timer(rospy.Duration(1), Estop_Monitor)
    rospy.spin()
