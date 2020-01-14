#!/usr/bin/env python

from Adafruit_MotorHAT import Adafruit_MotorHAT
from math import fabs, floor

class MotorHatDriver(object):
    MOTOR_FORWARD = Adafruit_MotorHAT.FORWARD
    MOTOR_BACKWARD = Adafruit_MotorHAT.BACKWARD
    MOTOR_RELEASE = Adafruit_MotorHAT.RELEASE

    DEADZONE = 5.e-2

    def __init__(self, addr=0x60):
        self.motor_hat = Adafruit_MotorHAT(addr=addr)
        self.left_x = self.motor_hat.getMotor(4)
        self.right_x = self.motor_hat.getMotor(3)

        self.left_up = self.motor_hat.getMotor(2)
        self.right_up = self.motor_hat.getMotor(1)

        self.left_x_speed = 0
        self.left_up_speed = 0
        self.right_x_speed = 0
        self.right_up_speed = 0

    # all values can be given between -1 and 1
    def setMotorSpeed(self, left_x=None, left_up=None, right_x=None, right_up=None):        
        if left_x is not None:
            self.left_x_speed = min(1, max(-1, left_x))
        if left_up is not None:
            self.left_up_speed = min(1, max(-1, left_up))
        if right_x is not None:
            self.right_x_speed = min(1, max(-1, right_x))
        if right_up is not None:
            self.right_up_speed = min(1, max(-1, right_up))
        self.updatePWM()

    def updatePWM(self):
        pwm_l_x = int(floor(fabs(self.left_x_speed) * 255))
        pwm_l_u = int(floor(fabs(self.left_up_speed) * 255))
        pwm_r_x = int(floor(fabs(self.right_x_speed) * 255))
        pwm_r_u = int(floor(fabs(self.right_up_speed) * 255))
        if fabs(self.left_x_speed) < self.DEADZONE:
            left_x_mode = self.MOTOR_RELEASE
        elif self.left_x_speed > 0:
            left_x_mode = self.MOTOR_FORWARD
        elif self.left_x_speed < 0:
            left_x_mode = self.MOTOR_BACKWARD

        if fabs(self.left_up_speed) < self.DEADZONE:
            left_u_mode = self.MOTOR_RELEASE
        elif self.left_up_speed > 0:
            left_u_mode = self.MOTOR_FORWARD
        elif self.left_up_speed < 0:
            left_u_mode = self.MOTOR_BACKWARD

        if fabs(self.right_x_speed) < self.DEADZONE:
            right_x_mode = self.MOTOR_RELEASE
        elif self.right_x_speed > 0:
            right_x_mode = self.MOTOR_FORWARD
        elif self.right_x_speed < 0:
            right_x_mode = self.MOTOR_BACKWARD

        if fabs(self.right_up_speed) < self.DEADZONE:
            right_u_mode = self.MOTOR_RELEASE
        elif self.right_up_speed > 0:
            right_u_mode = self.MOTOR_FORWARD
        elif self.right_up_speed < 0:
            right_u_mode = self.MOTOR_BACKWARD

        self.left_x.setSpeed(pwm_l_x)
        self.left_x.run(left_x_mode)
        self.left_up.setSpeed(pwm_l_u)
        self.left_up.run(left_u_mode)
        self.right_x.setSpeed(pwm_r_x)
        self.right_x.run(right_x_mode)
        self.right_up.setSpeed(pwm_r_u)
        self.right_up.run(right_u_mode)

    def __del__(self):
        self.left_x.run(self.MOTOR_RELEASE)
        self.left_up.run(self.MOTOR_RELEASE)
        self.right_x.run(self.MOTOR_RELEASE)
        self.right_up.run(self.MOTOR_RELEASE)
        del self.motor_hat
