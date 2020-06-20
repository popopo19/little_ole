#!/usr/bin/env python

import rospy
import time
import RPi.GPIO as GPIO
from std_msgs.msg import Float64

class UltrasonicSensor:
  def __init__(self, echo, trig):
    rospy.init_node("ultrasonic_sensor", anonymous=True)


    rospy.loginfo("======== Ultrasonic Sensor Node Initiated ========")
    
    self.distance_pub = rospy.Publisher("ultrasonic_sensor/distance", Float64, queue_size=10)

    self.echo = echo
    self.trig = trig
    self.rate = rospy.Rate(20.0)

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(self.echo, GPIO.IN)
    GPIO.setup(self.trig, GPIO.OUT)

    GPIO.output(self.trig, GPIO.LOW)
    rospy.loginfo("======== Waiting for sensor to get ready... ========")
    time.sleep(2)
    rospy.loginfo("======== Sensor is ready ========")

    rospy.on_shutdown(self.shutdownhook)

    while not rospy.is_shutdown():
      self.distance_pub.publish(self.get_distance())
      self.wait(10)

  def get_distance(self):
    GPIO.output(self.trig, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(self.trig, GPIO.LOW)
  
    pulse_start = 0
    pulse_end = 0
    
    while GPIO.input(self.echo) == 0:
      pulse_start = time.time()
    while GPIO.input(self.echo) == 1:
      pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150

    return round(distance, 2)

  def wait(self, time):
    for _ in range(time):
      self.rate.sleep()

  def shutdownhook(self):
    rospy.loginfo("======== Ultrasonic Sensor is shutting down ========")
    GPIO.cleanup()

if __name__ == "__main__":
  u = UltrasonicSensor(13, 5)
