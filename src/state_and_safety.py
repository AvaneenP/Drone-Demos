#!/usr/bin/env python
import rospy
import time
import copy
import math 
from enum import Enum
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from freyja_msgs.msg import ReferenceState 

# A class to keep track of the quadrotors state
class DroneState(Enum):
  HOVERING = 1
  VERIFYING = 2

# Create a class which takes in a position and verifies it, keeps track of the state and publishes commands to a drone
class StateAndSafety():
  # Node initialization
  def __init__(self):
    # Create the publisher and subscriber
    self.position_pub = rospy.Publisher('/reference_state', ReferenceState, queue_size=1)
    self.keyboard_sub = rospy.Subscriber('/unverified_position', ReferenceState, self.getUnverifiedPosition, queue_size = 1)
        
    # Get the acceptance range
    self.acceptance_range = rospy.get_param("/state_safety_node/acceptance_range", 0.1)
    # Getting the virtual cage parameters
    cage_params = rospy.get_param('/state_safety_node/virtual_cage', {'x': 1.5, 'y': 1.5, 'z': -1.75})
    cn, ce, cd = cage_params['y'], cage_params['x'], cage_params['z']
    # Create the virtual cage
    self.cage_n = [-1 * cn, cn]
    self.cage_e = [-1 * ce, ce]
    self.cage_d = [cd, 0]
    # Display incoming parameters
    rospy.loginfo(str(rospy.get_name()) + ": Launching with the following parameters:")
    rospy.loginfo(str(rospy.get_name()) + ": Param: cage n - " + str(self.cage_n))
    rospy.loginfo(str(rospy.get_name()) + ": Param: cage e - " + str(self.cage_e))
    rospy.loginfo(str(rospy.get_name()) + ": Param: cage d - " + str(self.cage_d))
    rospy.loginfo(str(rospy.get_name()) + ": Param: acceptance range - " + str(self.acceptance_range))
    
    # Create the drones state as hovering
    self.state = DroneState.HOVERING
    rospy.loginfo(str(rospy.get_name()) + ": Current State: HOVERING")
    
    # Create the goal messages we are going to be sending
    self.goal_pos = ReferenceState()
    self.unverified_goal_pos = ReferenceState()
        
    # Start the drone a little bit off the ground
    self.goal_pos.pn = 0.0
    self.goal_pos.pe = 0.0
    self.goal_pos.pd = -1.5
    self.goal_pos.vn = 0.0
    self.goal_pos.ve = 0.0
    self.goal_pos.vd = 0.0
    # self.goal_pos.yaw = DEG2RAD(0.0)
    self.goal_pos.an = 0.0
    self.goal_pos.ae = 0.0
    self.goal_pos.ad = 0.0
    self.goal_pos.header.stamp = rospy.Time.now()    
    
    # Keeps track of whether the position was changed or not
    self.goal_changed = False
    # Call the mainloop of our class
    self.mainloop()

  # Callback for the keyboard manager
  def getUnverifiedPosition(self, msg):
    # Save the keyboard command
    if self.state == DroneState.HOVERING:
      self.goal_changed = True
      self.unverified_goal_pos = copy.deepcopy(msg)

  # Converts a position to string for printing
  def goalToString(self, msg):
    pos_str = "(" + str(msg.pn) 
    pos_str += ", " + str(msg.pe)
    pos_str += ", " + str(msg.pd) + ")"
    return pos_str

  def processVerifying(self):
    # Check if the new goal is inside the cage
    n_check = self.cage_n[0] <= self.unverified_goal_pos.pn <= self.cage_n[1]
    e_check = self.cage_e[0] <= self.unverified_goal_pos.pe <= self.cage_e[1]
    d_check = self.cage_d[0] <= self.unverified_goal_pos.pd <= self.cage_d[1]
    # If it is change state to moving
    if n_check and e_check and d_check:
      self.goal_pos = self.unverified_goal_pos

    # If it is not change to hovering
    else:
      print("Outside of cage - will continue hovering")
      rospy.loginfo(str(rospy.get_name()) + ": Current State: HOVERING")
    self.state = DroneState.HOVERING

  # This function is called when we are in the hovering state
  def processHovering(self):
    # Print the requested goal if the position changed
    if self.goal_changed:
      #rospy.loginfo(str(rospy.get_name()) + ": Requested Position: " + self.goalToString(self.unverified_goal_pos))
      #rospy.loginfo(str(rospy.get_name()) + ": Current State: VERIFYING")
      self.state = DroneState.VERIFYING

  # The main loop of the function
  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(20)
    # While ROS is still running
    while not rospy.is_shutdown():
      # Publish the position
      self.position_pub.publish(self.goal_pos)

      if self.state == DroneState.HOVERING:
        self.processHovering()
      elif self.state == DroneState.VERIFYING:
        self.processVerifying()
      # Sleep for the remainder of the loop
      rate.sleep()

if __name__ == '__main__':
  rospy.init_node('state_safety_node')
  try:
    ktp = StateAndSafety()
  except rospy.ROSInterruptException:
    pass