#!/usr/bin/env python

import rospy
import time
import RPi.GPIO as GPIO
import serial
import pynmea2

class GPS():
	rospy.init_node("gps", anonymous=True)

	rospy.loginfo("======== GPS Node Initiated ========")

	

if __name__ == "__main__":
	u = UltrasonicSensor(13, 5)
