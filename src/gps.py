#!/usr/bin/env python

import rospy
import time
import RPi.GPIO as GPIO
import serial
import pynmea2
import std_msgs.msg import NavSatFix

class GPS():
	def __init__(self):
		rospy.init_node("gps", anonymous=True)

		rospy.loginfo("======== GPS Node Initiated ========")

		self.gps_pub = rospy.Publisher("gps", NavSatFix, queue_size=10)
		self.ser = serial.Serial("/dev/ttyACM0", 9600, timeout=0.5)
		self.gps = NavSatFix()

		while not rospy.is_shutdown():
			data = ser.readline()
			if b'GPRM' in data:
				msg = pynmea2.parse(data.decode("utf-8"))
				self.gps.latitude = msg.latitude
				self.gps.longitude = msg.longitude
				self.gps_pub.publish(self.gps)
				

if __name__ == "__main__":
	u = GPS()
