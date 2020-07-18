#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from  std_msgs.msg import Bool

class TiltBallSwitch:
  def __init__(self, pin_in, pin_out):
    rospy.init_node("tilt_ball_switch", anonymous=False)
    rospy.loginfo("======== Tilt Ball Switch Node Initaliized ========")

    self.tilt_ball_pub = rospy.Publisher("/tilt_ball_switch/", Bool, queue_size=10)

    self.pin = pin_in
    self.pin_out = pin_out
    self.rate = rospy.Rate(20.0)
    self.is_upright = Bool()
    self.is_upright.data = False

    # GPIO setup
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(self.pin, GPIO.IN)
    GPIO.setup(self.pin_out, GPIO.OUT)
    GPIO.output(self.pin_out, True)

    rospy.on_shutdown(self.shutdownhook)

    while not rospy.is_shutdown():
      if GPIO.input(self.pin):
        self.is_upright.data = True
      else:
        self.is_upright.data = False
      self.tilt_ball_pub.publish(self.is_upright)

  def wait(self, time):
    for _ in range(time):
      self.rate.sleep()

  def shutdownhook(self):
    rospy.loginfo("======== Tilt Ball Switch is shutting down ========")
    GPIO.cleanup()

if __name__ == "__main__":
  tiltball = TiltBallSwitch(16, 23)
