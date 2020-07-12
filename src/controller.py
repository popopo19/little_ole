#!/usr/bin/env python

import rospy, sys
from sensor_msgs.msg import Joy, NatSatFix
from std_msgs.msg import Float32, Int32, Bool

class Controller:
	def __init__(self):
		rospy.init_node("controller", anonymous=True)
		print("\nController Node Initialized\n")
		
		self.state_pub = rospy.Publisher("/controller/state", Int32, queue_size=10)
		self.pmotor_rotation_pub = rospy.Publisher("/power_motor/rotation", Float32, queue_size=10)
		self.smotor_rotation_pub = rospy.Publisher("/steer_motor/rotation", Float32, queue_size=10)
		self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb)
		self.tilt_pub = rospy.Subscriber("/tilt_switch_ball", Bool, self.tilt_cb)
		self.gps_sub = rospy.Subscriber("/gps", NatSatFix, self.gps_cb)
		
		self.state = 0
		
		self.joy = Joy()
		self.gps = NatSatFix()
		self.on_back = False; 
		self.rate = rospy.Rate(20.0)
		
		rospy.on_shutdown(self.shutdownhook)

		while not rospy.is_shutdown():
			if self.state == 1:
				self.dualshock_control()
			elif self.state == 2:
				self.explore()
			else:
				self.menu_mode()

	def menu_mode(self):
		self.state = input("[1] - Manual Control\n2] - Explore\nChoice: ")
		self.state_pub.publish(self.state)
	
	def joy_cb(self, msg):
		self.joy = msg

	def tilt_cb(self, msg):
		self.on_back = msg.data

	def gps_cb(self, msg):
		self.gps = msg
	
	def dualshock_control(self):
		if len(self.joy.buttons) != 0:
			if self.joy.buttons[5] == 1:
				self.forward()
			elif self.joy.buttons[7] == 1:
				self.reverse()
			else:
				self.pmotor_rotation_pub.publish(0.0)
		
		if len(self.joy.axes) != 0:
			self.smotor_rotation_pub.publish(self.joy.axes[0])

		self.backToMenu() 
		self.wait(1)

	def forward(self):
		self.pmotor_rotation_pub.publish(1)
	
	def reverse(self):
		self.pmotor_rotation_pub.publish(-1)

	def backToMenu(self):
		if len(self.joy.buttons) != 0:
			if self.joy.buttons[2]:
				self.state = 0
				self.pmotor_rotation_pub.publish(0.0)
		elif self.on_back == True:
			self.state = 0
			self.pmotor_rotation_pub.publish(0.0)

	def explore(self):
		self.backToMenu()
	
	def wait(self, time):
		for _ in range(time):
			self.rate.sleep()
	
	def shutdownhook(self):
		rospy.loginfo("======== Controller Shutting Down ========")

if __name__ == "__main__":
	controller = Controller()
