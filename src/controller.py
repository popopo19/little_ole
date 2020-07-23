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
		print("======== Menu ========")

		choice = int(input("[1] - Manual Control\n[2] - Autonomous Navigation\n[0] - Quit\nChoice: "))

		if choice == 1:
			print(self.manual_control(False))
		elif choice == 2:
			x = int(input("New local x position: "))
			y = int(input("New local y position: "))
			print(self.autonav(x, y))

	def manual_control(self, using_keyboard):
		rospy.wait_for_service("manual_control")
		try:
			manual_control = rospy.ServiceProxy("manual_control", ManualControl)
			resp = manual_control(using_keyboard)
			return resp.response
		except rospy.ServiceException as e:
			print("Service call failed")

	def autonav(self, x, y):
		rospy.wait_for_service("autonav")
		try:
			autonav = rospy.ServiceProxy("autonav", AutoNav)
			resp = autonav(x, y)
			return resp.response
		except rospy.ServiceException as e:
			print("Servce call failed")

	def wait(self, time):
		for _ in range(time):
			self.rate.sleep()

	def shutdownhook(self):
		rospy.loginfo("======== Controller Shutting Down ========")

if __name__ == "__main__":
	controller = Controller()
