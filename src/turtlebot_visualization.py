#!/usr/bin/env python
import rospy
import time
import copy
import math 
from enum import Enum
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from freyja_msgs.msg import ReferenceState 

class TurtleViz():
  # Node initialization
  def __init__(self):
    # Create the publisher and subscriber
    self.position_drone_pub = rospy.Publisher('/visualization/drone', PointStamped, queue_size=1)
    self.position_turtle_pub = rospy.Publisher('visualization/turtle', PointStamped, queue_size=1)

    self.position_drone_sub = rospy.Subscriber('/vicon/JOZI/JOZI', TransformStamped, self.getDronePos, queue_size = 1)
    self.position_drone_sub = rospy.Subscriber('/vicon/BUGS/BUGS', TransformStamped, self.getTurtlePos, queue_size = 1)

    self.drone_pos = PointStamped()
    self.turtle_pos = PointStamped()
    self.drone_pos.header.frame_id = "map"
    self.turtle_pos.header.frame_id = "map"

    self.mainloop()

  # Callback 1
  def getDronePos(self, msg):
    self.drone_pos.point.x = msg.transform.translation.x
    self.drone_pos.point.y = msg.transform.translation.y
    self.drone_pos.point.z = msg.transform.translation.z
    self.position_drone_pub.publish(self.drone_pos)

  # Callback 2
  def getTurtlePos(self, msg):
    self.turtle_pos.point.x = msg.transform.translation.x
    self.turtle_pos.point.y = msg.transform.translation.y
    self.turtle_pos.point.z = msg.transform.translation.z
    self.position_turtle_pub.publish(self.turtle_pos)

  # The main loop of the function
  def mainloop(self):
    rate = rospy.Rate(20)

    # While ROS is still running
    while not rospy.is_shutdown():
      # Sleep for the remainder of the loop
      rate.sleep()

if __name__ == '__main__':
  rospy.init_node('turtle_viz_node')
  try:
    ktp = TurtleViz()
  except rospy.ROSInterruptException:
    pass