#!/usr/bin/env python
import rospy
import math 

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from duckiefloat_msgs.srv import EStop

class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.v_gain = 0.88

        # Publishers
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Subscribers
        self.sub_joy = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        
        # Service Proxy
        rospy.wait_for_service('/estop')
        self.estop = rospy.ServiceProxy('estop', EStop)

    def cbJoy(self, msg):
        self.processButtons(msg)
        self.processAxes(msg)

    # Axis List index of joy.axes array:
    # 0: Left Horizontal (Left +)
    # 1: Left Vertical (Up +)
    # 2: LT (0 ~ 1)
    # 3: Right Horizontal (Left +)
    # 4: Right Vertical (Up +)
    # 5: RT (0 ~ 1)
    # 6: CrossKey Horizontal (Left 1)
    # 7: CrossKey Vertical (Up 1)
    def processAxes(self, joy_msg):
        # rospy.loginfo("axes processed")
        cmd_msg = Twist()
        cmd_msg.linear.x = joy_msg.axes[4]
        #cmd_msg.linear.y = joy_msg.axes[3]
        cmd_msg.linear.z = joy_msg.axes[1]
        cmd_msg.angular.z = joy_msg.axes[3]
        self.pub_cmd_vel.publish(cmd_msg)

    # Button List index of joy.buttons array:
    # 0: A 
    # 1: B 
    # 2: X
    # 3: Y 
    # 4: Left Back 
    # 5: Right Back
    # 6: Back
    # 7: Start
    # 8: Logitek 
    # 9: Left joystick
    # 10: Right joystick
    def processButtons(self, joy_msg):
        # Button B
        if (joy_msg.buttons[1] == 1):
            pass
        # Button Y
        if (joy_msg.buttons[3] == 1):
            pass


        # Left back button
        if (joy_msg.buttons[4] == 1):
            # EStop ON
            status = self.estop(True, False)
        # Right back button
        elif (joy_msg.buttons[5] == 1):
            # EStop OFF
            status = self.estop(False, True)

if __name__ == "__main__":
    rospy.init_node("joy_mapper_node",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.spin()
