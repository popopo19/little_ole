#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from little_ole.srv import *

class Controller:
	def __init__(self):
		rospy.init_node("controller", anonymous=True)
		print("\nController Node Initialized\n")
		
		self.rate = rospy.Rate(20.0)
		
		rospy.on_shutdown(self.shutdownhook)

		while not rospy.is_shutdown():
			self.menu_mode()

	def menu_mode(self):
		choice = int(input("[1] - Manual Control\n[2] - Autonomous Navigation\nChoice: "))

		if choice == 1:
			print(self.manual_control(False))
		if choice == 2:
			pass

	def manual_control(self, using_keyboard):
		rospy.wait_for_service("manual_control")
		try:
			manual_control = rospy.ServiceProxy("manual_control", ManualControl)
			resp = manual_control(using_keyboard)
			return resp.response
		except rospy.ServiceException as e:
			print("Service call failed")
	
	def wait(self, time):
		for _ in range(time):
			self.rate.sleep()

	def shutdownhook(self):
		rospy.loginfo("======== Controller Shutting Down ========")

if __name__ == "__main__":
	controller = Controller()
