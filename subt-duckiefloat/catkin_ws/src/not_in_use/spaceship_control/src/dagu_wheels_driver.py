#!/usr/bin/python

# Wrapping the Adafruit API to talk to DC motors with a simpler interface
#
# date:    11/17/2015
#
# authors: Valerio Varricchio <valerio@mit.edu>
#          Luca Carlone <lcarlone@mit.edu>
#          Dmitry Yershov <dmitry.s.yershov@gmail.com>
#          Shih-Yuan Liu <syliu@mit.edu>

from Adafruit_MotorHAT import Adafruit_MotorHAT
from math import fabs, floor

class DaguWheelsDriver:
    LEFT_MOTOR_MIN_PWM = 60        # Minimum speed for left motor  
    LEFT_MOTOR_MAX_PWM = 255       # Maximum speed for left motor  
    RIGHT_MOTOR_MIN_PWM = 60       # Minimum speed for right motor  
    RIGHT_MOTOR_MAX_PWM = 255      # Maximum speed for right motor  
    # AXEL_TO_RADIUS_RATIO = 1.0     # The axel length and turning radius ratio
    SPEED_TOLERANCE = 1.e-2       # speed tolerance level

    def __init__(self, verbose=False, debug=False, left_flip=False, right_flip=False):
        self.motorhat = Adafruit_MotorHAT(addr=0x60)

        self.leftMotor = self.motorhat.getMotor(3)
        self.rightMotor = self.motorhat.getMotor(4)

	self.upright = self.motorhat.getMotor(1)
	self.upleft = self.motorhat.getMotor(2)

        self.verbose = verbose or debug
        self.debug = debug
        
        self.left_sgn = 1.0
        if left_flip:
            self.left_sgn = -1.0

        self.right_sgn = 1.0
        if right_flip:
            self.right_sgn = -1.0

	self.throttle_sgn = 1.0
        if right_flip:
            self.throttle_sgn = -1.0

        self.leftSpeed = 0.0
        self.rightSpeed = 0.0
	self.throttle = 0.0

        self.updatePWM()

    def PWMvalue(self, v, minPWM, maxPWM):
        pwm = v
        if fabs(v) > self.SPEED_TOLERANCE:
            pwm = int(floor(fabs(v) * (maxPWM - minPWM) + minPWM))
        return (min(pwm, maxPWM))

    def updatePWM(self):
        global upMotorMode

        vl = self.leftSpeed*self.left_sgn
        vr = self.rightSpeed*self.right_sgn
	vup = self.throttle*self.throttle_sgn

        pwml = self.PWMvalue(vl, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)
        pwmr = self.PWMvalue(vr, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)
	pwmup = int(max(60, min(255, fabs(vup))))

	print(pwml, pwmr, pwmup)

        if self.debug:
            print "v = %5.3f, u = %5.3f, vl = %5.3f, vr = %5.3f, pwml = %3d, pwmr = %3d" % (v, u, vl, vr, pwml, pwmr)

        if fabs(vl) < self.SPEED_TOLERANCE:
            leftMotorMode = Adafruit_MotorHAT.RELEASE
            pwml = 0
        elif vl > 0:
            leftMotorMode = Adafruit_MotorHAT.FORWARD
        elif vl < 0: 
            leftMotorMode = Adafruit_MotorHAT.BACKWARD

        if fabs(vr) < self.SPEED_TOLERANCE:
            rightMotorMode = Adafruit_MotorHAT.RELEASE
            pwmr = 0
        elif vr > 0:
            rightMotorMode = Adafruit_MotorHAT.FORWARD
        elif vr < 0: 
            rightMotorMode = Adafruit_MotorHAT.BACKWARD

	if fabs(vup) < self.SPEED_TOLERANCE:
            upMotorMode = Adafruit_MotorHAT.RELEASE
            pwmup = 0
        elif vup > 0:
            upMotorMode = Adafruit_MotorHAT.FORWARD
        elif vup < 0:
            upMotorMode = Adafruit_MotorHAT.BACKWARD

        self.leftMotor.setSpeed(pwml)
        self.leftMotor.run(leftMotorMode)
        self.rightMotor.setSpeed(pwmr)
        self.rightMotor.run(rightMotorMode)

	self.upleft.setSpeed(pwmup)
        self.upleft.run(upMotorMode)
	self.upright.setSpeed(pwmup)
        self.upright.run(upMotorMode)


    def setWheelsSpeed(self, left, right):
        self.leftSpeed = left
        self.rightSpeed = right
        self.updatePWM()

    def setThrottle(self, throttle):
	self.throttle = throttle
	self.updatePWM()

    def __del__(self):
        self.leftMotor.run(Adafruit_MotorHAT.RELEASE)
        self.rightMotor.run(Adafruit_MotorHAT.RELEASE)
	self.upleft.run(Adafruit_MotorHAT.RELEASE)
        self.upright.run(Adafruit_MotorHAT.RELEASE)

        del self.motorhat

