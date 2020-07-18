#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32, Float64

class Motor:
  def __init__(self, name, en1, in1, in2):
    self.name = name

    rospy.init_node(self.name, anonymous=False)
    rospy.loginfo("======== " + self.name + "Node initialized ========")

    self.on = False
    self.distance = 0
    self.rotation = 0 # 1 => forward, -1 => backward
    self.rotation_sub = rospy.Subscriber("/" + self.name, Float32, self.rotation_cb)

    # pins
    self.en1 = en1
    self.in1 = in1
    self.in2 = in2

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(self.en1, GPIO.OUT)
    GPIO.setup(self.in1, GPIO.OUT)
    GPIO.setup(self.in2, GPIO.OUT)

    # Makes sure motor isn't starting at the beginning
    self.stop()

    rospy.on_shutdown(self.shutdownhook)

    while not rospy.is_shutdown():
      if self.rotation > 0:
        self.run()
        self.rotate_forward()
      elif self.rotation < 0:
        self.run()
        self.rotate_backward()
      elif self.on == True and self.rotation == 0:
        self.stop()

  def run(self):
    GPIO.output(self.en1, True)
    self.on = True
    # rospy.loginfo("======== " + self.name + " Is Running ========")

  def stop(self):
    GPIO.output(self.en1, False)
    self.on = False
    # rospy.loginfo("======== " + self.name + " Is Stoping ========")

  def rotate_forward(self):
    GPIO.output(self.in1, True)
    GPIO.output(self.in2, False)
    # rospy.loginfo("======== " + self.name + " rotating forward.")

  def rotate_backward(self):
    GPIO.output(self.in1, False)
    GPIO.output(self.in2, True)
    # rospy.loginfo("======== " + self.name + " rotating backward.")

  def rotation_cb(self, msg):
    self.rotation = msg.data

  def shutdownhook(self):
    rospy.loginfo("======== " + self.name + " is shutting down ========")
    GPIO.cleanup()

if __name__ == "__main__":
  # motor0 = Motor("motor", 22, 17, 27)
  motor1 = Motor("steer_motor", 7, 8, 25)
