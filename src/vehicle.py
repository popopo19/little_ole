#!/usr/bin/env python

import rospy, threading, math
from little_ole.srv import *
from std_msgs.msg import Float32, Float64, Bool
from sensor_msgs.msg import Joy, NavSatFix
from geometry_msgs.msg import Pose

class Vehicle:
	def __init__(self):
		rospy.init_node("vehicle", anonymous=False)
		rospy.loginfo("======== Vehicle Node initialized ========")

		self.joy = Joy()
		self.is_upright = Bool()
		self.sonic_dist = 0.0
		self.gps = NavSatFix()
		self.local_pose = Pose()
		self.rate = rospy.Rate(20.0)
		self.zoom_factor = 10000

		self.pmotor_pub = rospy.Publisher("/power_motor", Float32, queue_size=10)
		self.smotor_pub = rospy.Publisher("/steer_motor", Float32, queue_size=10)
		self.local_pose_pub = rospy.Publisher("/local_pose", Pose, queue_size=10)
		self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb)
		self.tilt_ball_sub = rospy.Subscriber("/tilt_ball_switch", Bool, self.tilt_ball_cb)
		self.sonic_dist_sub = rospy.Subscriber("/ultrasonic_sensor", Float64, self.sonic_dist_cb)
		self.gps_sub = rospy.Subscriber("/gps", NavSatFix, self.gps_cb)

		self.manual_srv = rospy.Service("manual_control", ManualControl, self.manual_control)
		self.autonav_srv = rospy.Service("autonav", AutoNav, self.autonav)

		rospy.on_shutdown(self.shutdownhook)

		self.wait(100)

		self.origin = Pose()
		self.origin.position.x = self.gps.longitude * self.zoom_factor
		self.origin.position.y = self.gps.latitude * self.zoom_factor
		rospy.loginfo("======== originX: " + str(self.origin.position.x) + ", originY: " + str(self.origin.position.y) + " ========")

		self.pose_thread = threading.Thread(target=self.calc_local_pose)
		self.pose_thread.daemon = True
		self.pose_thread.start()

		rospy.spin()

	def calc_local_pose(self):
		# x = longitude, y = latitude
		while not rospy.is_shutdown():
			self.local_pose.position.x = self.gps.longitude * self.zoom_factor - self.origin.position.x 
			self.local_pose.position.y = self.gps.latitude * self.zoom_factor - self.origin.position.y
			# rospy.loginfo("X: " + str(self.local_pose.position.x) + ", Y: " + str(self.local_pose.position.y))
			self.rate.sleep()

	def manual_control(self, req):

		rospy.loginfo("======== Manual Control Started ========")

		if not req.using_keyboard:
			while not rospy.is_shutdown():
				if len(self.joy.buttons) > 0:
					if self.joy.buttons[5] > 0:
						if not self.object_detected():
							self.forward()
					elif self.joy.buttons[4] > 0:
						# rospy.loginfo("======== Going backwards ========")
						self.backward()
					else:
						self.stop()

					if self.joy.axes[0] > 0:
						self.left()
					elif self.joy.axes[0] < 0:
						self.right()
					else:
						self.straight()

					if self.joy.buttons[0] or self.upside_down():
						break
		rospy.loginfo("======== Manual Control Ended ========")
		return ManualControlResponse("======== Manual Control Ended ========")

	def upside_down(self):
		if not self.is_upright:
			rospy.loginfo("======== Vehicle is upside down ========")
			return True
		else:
			return False

	def object_detected(self):
		if self.sonic_dist < 12:
			rospy.loginfo("======== Object detected in front of vehicle ========")
			return True
		else:
			return False

	def autonav(self, req):
		rospy.loginfo("======== Autonomous Navigation Started ========")

		while not rospy.is_shutdown():
			distance_to_goal = self.distance(req.x, req.y, self.local_pose.position.x, self.local_pose.position.y)
			rospy.loginfo("Distance To Goal: " + str(distance_to_goal))
			

			if self.upside_down() or self.distance(self.local_pose.position.x, self.local_pose.position.y, req.x, req.y) < 2:
				break

		return AutoNavResponse("======== Autonomous Navigation Ended ========")

	def left(self):
		self.smotor_pub.publish(1.0)

	def right(self):
		self.smotor_pub.publish(-1.0)

	def straight(self):
		self.smotor_pub.publish(0.0)

	def forward(self):
		self.pmotor_pub.publish(1.0)

	def backward(self):
		self.pmotor_pub.publish(-1.0)

	def stop(self):
		self.pmotor_pub.publish(0.0)

	def distance(self, x1, y1, x2, y2):
		return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

	def joy_cb(self, msg):
		self.joy = msg

	def tilt_ball_cb(self, msg):
		self.is_upright = msg

	def sonic_dist_cb(self, msg):
		self.sonic_dist = msg.data
		rospy.loginfo(self.sonic_dist)

	def gps_cb(self, msg):
		self.gps = msg

	def wait(self, time):
		for _ in range(time):
			self.rate.sleep()

	def shutdownhook(self):
		rospy.loginfo("======== Vehicle is shutting down ========")

if __name__ == "__main__":
	vehicle = Vehicle()
