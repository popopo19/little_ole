#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from  std_msgs.msg import Bool

class TiltBallSwitch:
  def __init__(self, pin):
    rospy.init_node("tilt_ball_switch", anonymous=False)
    rospy.loginfo("======== Tilt Ball Switch Node Initaliized ========")

    self.tilt_ball_pub = rospy.Publisher("/tilt_ball_switch/", Bool, queue_size=10)

    self.pin = pin
    self.rate = rospy.Rate(20.0)
    self.is_upright = True

    # GPIO setup
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(self.pin, GPIO.IN)

    rospy.on_shutdown(self.shutdownhook)

    while not rospy.is_shutdown():
      if GPIO.input(self.pin):
        rospy.loginfo("======== Vehicle is upside down ========")
      self.wait(20)

  def wait(self, time):
    for _ in range(time):
      self.rate.sleep()

  def shutdownhook(self):
    rospy.loginfo("======== Tilt Ball Switch is shutting down ========")
    GPIO.cleanup()

if __name__ == "__main__":
  tiltball = TiltBallSwitch(16)
