from __future__ import division
#!/usr/bin/env python
import rospy
from spaceship_msgs.msg import MotorsCmd
import Adafruit

class MotorsDriverNode(object):

    MAX_SPEED = 2000

    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.motorhat = Adafruit_PCA9685.PCA9685()

        self.pwm_freq = 500
        self.motorhat.set_pwm_freq(self.pwm_freq)

        self.leftSpeed = 0
        self.leftDir = 0
        self.rightSpeed = 0
        self.rightDir = 0

        self.left_forward_esc = 0
        self.left_backward_esc = 1
        self.right_forward_esc = 2
        self.right_backward_esc = 3
        
        # Setup subscribers
        self.sub_topic = rospy.Subscriber("/spaceship/motors_cmd", MotorsCmd, self.cbMotorsCmd, queue_size=1)

    def set_pwm_pulsewidth(self, channel, pulsewidth):
        pwmValue = 4096 * pulsewidth/MAX_SPEED
        self.motorhat.set_pwm(channel, 0, pwmValue)

    def cbMotorsCmd(self,msg):
        self.leftSpeed = msg.vel_front_left
        self.rightSpeed = msg.vel_front_right

        if self.leftSpeed < 0:
            vl = abs(self.leftSpeed)
            self.set_pwm_pulsewidth(self.left_backward_esc, vl)
        else:
            vl = abs(self.leftSpeed)
            self.set_pwm_pulsewidth(self.left_forward_esc, vl)

        if self.rightSpeed < 0:
            vr = abs(self.rightSpeed)
            self.set_pwm_pulsewidth(self.right_backward_esc, vr)
        else:
            vr = abs(self.rightSpeed)
            self.set_pwm_pulsewidth(self.right_forward_esc, vr)

    def on_shutdown(self):
        self.driver.setWheelsSpeed(left=0.0,right=0.0)
        rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    rospy.init_node('motors_driver_node', anonymous=False)
    node = MotorsDriverNode()
    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
