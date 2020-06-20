#!/usr/bin/env python

import rospy, sys
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Int32

class Controller:
  def __init__(self):
    rospy.init_node("controller", anonymous=True)
    print("\nController Node Initialized\n")
   
    self.state_pub = rospy.Publisher("/controller/state", Int32, queue_size=10)
    self.pmotor_rotation_pub = rospy.Publisher("/power_motor/rotation", Float32, queue_size=10)
    self.smotor_rotation_pub = rospy.Publisher("/steer_motor/rotation", Float32, queue_size=10)
    self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb)

    self.state = 0
    self.in_state = False

    self.joy = Joy()
    self.rate = rospy.Rate(20.0)

    rospy.on_shutdown(self.shutdownhook)

  def menu_mode(self):
    while not rospy.is_shutdown():
      if not self.in_state:
        self.state = input("[0] - Manual Control: ")
        self.state_pub.publish(self.state)
        self.in_state = True
      if self.state == 0:
        self.dualshock_control()
 
  def joy_cb(self, msg):
    self.joy = msg

  def dualshock_control(self):
    if len(self.joy.buttons) != 0:
      if self.joy.buttons[5] == 1:
        self.pmotor_rotation_pub.publish(self.joy.buttons[5])
      elif self.joy.buttons[7] == 1:
        self.pmotor_rotation_pub.publish(self.joy.buttons[7] * -1)
      else:
        self.pmotor_rotation_pub.publish(0.0)
      if self.joy.buttons[2]:
        self.in_state = False

    if len(self.joy.axes) != 0:
      self.smotor_rotation_pub.publish(self.joy.axes[0])
    
    self.wait(1)

  def wait(self, time):
    for _ in range(time):
      self.rate.sleep()

  def shutdownhook(self):
    rospy.loginfo("======== Controller Shutting Down ========")

if __name__ == "__main__":
  controller = Controller()
  controller.menu_mode()
