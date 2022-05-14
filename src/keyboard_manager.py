#!/usr/bin/env python
import rospy
import time
import copy
from keyboard.msg import Key
from geometry_msgs.msg import Vector3
from freyja_msgs.msg import ReferenceState 

# Create a class which we will use to take keyboard commands and convert them to a ref state
class KeyboardManager():
  # On node initialization
  def __init__(self):
    # Create the publisher and subscriber
    self.position_pub = rospy.Publisher('/unverified_position', ReferenceState, queue_size=1)
    self.keyboard_sub = rospy.Subscriber('/keyboard/keydown', Key, self.get_key, queue_size = 1)
    
    # Create the position message we are going to be sending
    self.pos = ReferenceState()

    self.pos.pn = 0.0
    self.pos.pe = 0.0
    self.pos.pd = -1
    self.pos.vn = 0.0
    self.pos.ve = 0.0
    self.pos.vd = 0.0
    # self.pos.yaw = DEG2RAD(0.0)
    self.pos.an = 0.0
    self.pos.ae = 0.0
    self.pos.ad = 0.0
    self.pos.header.stamp = rospy.Time.now()
    
    # Create a variable we will use to hold the key code
    self.key_code = -1
    # Call the mainloop of our class
    self.mainloop()
  
  # Callback for the keyboard sub
  def get_key(self, msg):
    self.key_code = msg.code
  
  
  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(20)
    # While ROS is still running
    while not rospy.is_shutdown():
      # Publish the position
      self.position_pub.publish(self.pos)

      # West
      if self.key_code == Key.KEY_LEFT or self.key_code == Key.KEY_a:
        print("moved west 0.25")
        self.pos.pe -= 0.25
      # North
      if self.key_code == Key.KEY_UP or self.key_code == Key.KEY_w:
        print("moved north 0.25")
        self.pos.pn += 0.25
      # East
      if self.key_code == Key.KEY_RIGHT or self.key_code == Key.KEY_d:
        print("moved east 0.25")
        self.pos.pe += 0.25
      # South
      if self.key_code == Key.KEY_DOWN or self.key_code == Key.KEY_s:
        print("moved south 0.25")
        self.pos.pn -= 0.25
      # Up
      if self.key_code == Key.KEY_2:
        print("moved up 0.25")
        self.pos.pd -= 0.25
      # Down
      if self.key_code == Key.KEY_1:
        print("moved down 0.25")
        self.pos.pd += 0.25
      

      # Reset the code
      if self.key_code != -1:
        self.key_code = -1
      # Sleep for the remainder of the loop
      rate.sleep()

if __name__ == '__main__':
  rospy.init_node('keyboard_manager')
  try:
    ktp = KeyboardManager()
  except rospy.ROSInterruptException:
    pass