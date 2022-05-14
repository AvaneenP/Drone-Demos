#!/usr/bin/env python
import rospy
import time
import copy
from keyboard.msg import Key
from geometry_msgs.msg import Vector3
from freyja_msgs.msg import ReferenceState 
from geometry_msgs.msg import TransformStamped


# Create a class which we will use to take keyboard commands and convert them to a ref state
class TurtleManager():
  # On node initialization
  def __init__(self):
    # Create the publisher and subscriber
    self.position_pub = rospy.Publisher('/unverified_position', ReferenceState, queue_size=1)
    self.keyboard_sub = rospy.Subscriber('/vicon/BUGS/BUGS', TransformStamped, self.get_turtle_position, queue_size = 1)
    
    # Create the position message we are going to be sending
    self.pos = ReferenceState()

    self.turtle_position = TransformStamped()

    self.pos.pn = 0.0
    self.pos.pe = 0.0
    self.pos.pd = -1.5
    self.pos.vn = 0.0
    self.pos.ve = 0.0
    self.pos.vd = 0.0
    # self.pos.yaw = DEG2RAD(0.0)
    self.pos.an = 0.0
    self.pos.ae = 0.0
    self.pos.ad = 0.0
    self.pos.header.stamp = rospy.Time.now()
    
    # Call the mainloop of our class
    self.mainloop()
  
  # Callback for the turtle position
  def get_turtle_position(self, msg):
    self.turtle_position = msg
  
  
  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(20)

    self.pos.pn = 0.0
    self.pos.pe = 0.0
    self.pos.pd = -1.5
    self.position_pub.publish(self.pos)
    # 15 second lag before the drone begins to track the turtlebot's position
    rospy.sleep(15.)


    # While ROS is still running
    while not rospy.is_shutdown():

      # Update drone reference state to match up with turtlebot's x-y location
      self.pos.pn = self.turtle_position.transform.translation.y
      self.pos.pe = self.turtle_position.transform.translation.x
      self.pos.pd = -1.5

      # Publish the position
      self.position_pub.publish(self.pos)

      # Sleep for the remainder of the loop
      rate.sleep()

if __name__ == '__main__':
  rospy.init_node('turtle_manager_node')
  try:
    ktp = TurtleManager()
  except rospy.ROSInterruptException:
    pass