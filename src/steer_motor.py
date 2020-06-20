#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32
from motor import Motor

if __name__ == "__main__":
  steer_motor = Motor("steer_motor", 7, 8, 25)
