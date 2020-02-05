#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import pdb

pub = rospy.Publisher('pid_error', Float64, queue_size=10)

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = 0.1
MAX_DISTANCE = 30.0
MIN_ANGLE = -45.0
MAX_ANGLE = 225.0

# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, angle):
  # TODO: implement
  return 0.0

# data: single message from topic /scan
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the left wall.
def followLeft(data, desired_distance):
  # TODO: implement
  angle_min = data.angle_min
  angle_max = data.angle_max
  angle_increment = data.angle_increment

  range_max, range_min = data.range_max, data.range_min
  thetas = np.arange(angle_min, angle_max, angle_increment)
  #r = np.arange(606)
  r = np.clip(data.ranges, range_min, range_max)
  max_theta = 1.2217
  min_theta = np.pi/6

  if len(r) > 666:
     # gazebo
     min_theta = np.pi/2 - np.pi/8 # 35*np.pi/180
     max_theta = np.pi/2 + np.pi/8


  bools = thetas < max_theta
  bools = bools * (thetas > min_theta)
  r = r*bools
  if(r.mean() == 0):
	avg = 0;
  else:
  	r = np.ma.masked_equal(r,0)
 	avg = r.mean()
  d = desired_distance
  print("avg dist", avg)
  e = avg-d
  return e


# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance):
  # TODO: implement
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment

    range_max, range_min = data.range_max, data.range_min
    thetas = np.arange(angle_min, angle_max, angle_increment)
    #r = np.arange(606)
    r = np.clip(data.ranges, range_min, range_max)

    max_theta = 1.2217
    min_theta = np.pi/6

    if len(r) > 666:
     # gazebo
     min_theta = np.pi/2 - np.pi/8 # 35*np.pi/180
     max_theta = np.pi/2 + np.pi/8


    bools = thetas < max_theta
    bools = bools * (thetas > min_theta)
    r = r*bools # R=0 IF bools =0 (min<angle < max)
    if(r.mean() == 0):
    avg = 0;
    else:
    	r = np.ma.masked_equal(r,0)
    	avg = r.mean()
    d = desired_distance
    print("avg dist", avg)
    e = avg-d
      return e

  return 0.0

# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data):
  # TODO: implement
  return 0.0

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def scan_callback(data):
  Lerror = followLeft(data,0.5) # TODO: replace with followLeft, followRight, or followCenter
  Rerror = followRight(data,0.5)
  msg = Float64()
  msg.Ldata = Lerror
  msg.Rdata = Rerror
  pub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_error_node', anonymous = True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()
