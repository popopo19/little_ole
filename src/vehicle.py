#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from little_ole.srv import ManualControl, ManualControlResponse, AutoNav, AutoNavResponse
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Joy

class Vehicle:
	def __init__(self):
		rospy.init_node("vehicle", anonymous=False)
		rospy.loginfo("======== Vehicle Node initialized ========")

		self.joy = Joy()
		self.is_upright = Bool()

		self.pmotor_pub = rospy.Publisher("/power_motor", Float32, queue_size=10)
		self.smotor_pub = rospy.Publisher("/steer_motor", Float32, queue_size=10)
		self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb)
		self.tilt_ball_sub = rospy.Subscriber("/tilt_ball_switch", Bool, self.tilt_ball_cb)
		
		self.manual_srv = rospy.Service("manual_control", ManualControl, self.manual_control)
		self.autonav_srv = rospy.Service("autonav", AutoNav, self.autonav)

		rospy.on_shutdown(self.shutdownhook)

		rospy.spin()

	def manual_control(self, req):
		if not req.using_keyboard:
			while not rospy.is_shutdown():
				if self.joy.buttons[0] > 0:
					self.forward()
				elif self.joy.buttons[1] > 0:
					self.backward()
				else:
					self.stop()

				if self.joy.axes[0] > 1:
					self.left()
				if self.joy.axes[0] < 1:
					self.right()

				if self.joy.buttons[4]:
					return "======== Manual Controll Ended ========"

	def autonav(self, req):
		pass

	def left(self):
		self.smotor_pub.publish(1.0)

	def right(self):
		self.smotor_pub.publish(-1.0)

	def forward(self):
		self.pmotor_pub.publish(1.0)

	def backward(self):
		self.pmotor_pub.publish(-1.0)

	def stop(self):
		self.pmotor_pub.publish(0.0)

	def joy_cb(self, msg):
		self.joy = msg

	def tilt_ball_cb(self, msg):
		self.is_upright = msg

	def shutdownhook(self):
		rospy.loginfo("======== Vehicle is shutting down ========")

if __name__ == "__main__":
	vehicle = Vehicle()
