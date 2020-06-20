#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32
from motor import Motor

if __name__ == "__main__":
  power_motor = Motor("power_motor", 22, 17, 27)
