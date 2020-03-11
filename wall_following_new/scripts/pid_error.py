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
#MIN_ANGLE = -45.0
#MAX_ANGLE = 225.0

# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right, 270 degree range
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, angle):
  distance = data.ranges
  dis = distance.mean()
  return distance

# data: single message from topic /scan, roughly 40 data per scan
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the left wall.
count =0
def followLeft(data, desired_distance):
  angle_min = data.angle_min
  angle_max = data.angle_max
  angle_increment = data.angle_increment

  range_max, range_min = data.range_max, data.range_min
  thetas = np.arange(angle_min, angle_max, angle_increment) #create array of theteas from min to max

  r = np.clip(data.ranges, range_min, range_max)  # clip the range between min and max, r is array
  
  min_theta = 0       #45 degree
  max_theta = np.pi/6       #75 degree

  #if len(r) > 666:
     # gazebo
  #   min_theta = np.pi/2 - np.pi/8 # left 22.5
  #   max_theta = np.pi/2 + np.pi/8 #right 22.5


  in_range = thetas < max_theta
  in_range = in_range * (thetas > min_theta)
  r = r*in_range
  if(r.mean() == 0):
      avg_distance = 0;
  else:
      r = np.ma.masked_equal(r,0)  #set all elements to 0
      avg_distance = r.mean()

  e = avg_distance-desired_distance
  global count 
  count +=1
  if np.mod(count,100):
    print("avg dist from left wall", avg_distance)
  return e


# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance):
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment

    range_max, range_min = data.range_max, data.range_min
    thetas = np.arange(angle_min, angle_max, angle_increment)

    r = np.clip(data.ranges, range_min, range_max)

    max_theta = -np.pi/2   #-75 degree
    min_theta = -np.pi/2 - np.pi/6 # -45 degree

    #if len(r) > 666:
    #   min_theta = -np.pi/2 - np.pi/8
    #   max_theta = -np.pi/2 + np.pi/8

    bools = thetas < max_theta
    bools = bools * (thetas > min_theta)
    r = r*bools # R=0 IF bools =0 (min<angle < max)
    if(r.mean() == 0):
        avg = 0;
    else:
    	r = np.ma.masked_equal(r,0)
    	avg = r.mean()
    d = desired_distance
    global count 
    count +=1
    if np.mod(count,100):
      print("avg dist from right wall", avg)
    e = avg-d
    e = -e
    return e

# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data):

  error_L = followLeft(data,0.5) #get the error from the left wall
  error_R = followRight(data,0.5) #get the error from the right wall
  error_F = 0 #ABS(data,1) #get the error from in front of you
  if (error_F < -0.5):
      print ("plz stop")
      e = ABS(data,1)
  elif (error_L >= 0 and error_R < 0 and error_L < -error_R): #if far enough away from both walls and closer to left wall 
      print ("plz steer right")
      e = followLeft(data,0.5)
  elif (error_L >= 0 and error_R < 0 and error_L >= -error_R): #if far enough away from both walls and closer to right wall
      print ("plz steer left")
      e = followRight(data,0.5)
  elif (error_L < 0 and error_R < 0): #if too close to left wall but not right wall
      print ("plz steer right") 
      e = followLeft(data,0.5)
  elif (error_L >= 0 and error_R >= 0): #if too close to right wall but not left wall
      print ("plz steer left")
      e = followRight(data,0.5)
  elif (error_L < 0 and error_R >= 0 and error_L < -error_R): #if too close to both walls and closer to left wall
      print ("plz steer right")
      e = followLeft(data,0.5)
  elif (error_L < 0 and error_R >= 0 and error_L >= -error_R): #if far enough away from both walls and closer to right wall
      print ("plz steer left")
      e = followRight(data,0.5)

  print ("error from left and right are", error_L, error_R)
  return e

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def scan_callback(data):
  error = followCenter(data)
  #error = followLeft(data,0.5) # TODO: replace with followLeft, followRight, or followCenter
  #error = followRight(data,0.5)
  #error = ABS(data,1)
  
  msg = Float64()
  msg.data = error
  pub.publish(msg)

def ABS(data, stop_distance):
  angle_min = data.angle_min
  angle_max = data.angle_max
  angle_increment = data.angle_increment

  range_max, range_min = data.range_max, data.range_min
  thetas = np.arange(angle_min, angle_max, angle_increment) #create array of theteas from min to max

  r = np.clip(data.ranges, range_min, range_max)  # clip the range between min and max, r is array
  max_theta = -np.pi/5        #30 degree
  min_theta = -np.pi/4      #-30 degree
  bools = thetas < max_theta
  bools = bools * (thetas > min_theta)
  r = r*bools # R=0 IF bools =0 (min<angle < max)
  if(r.mean() == 0):
      avg = 0;
  else:
      r = np.ma.masked_equal(r,0)
      avg = r.mean()
  s = stop_distance
  if(avg < s):
      e = avg-s
  else:
      e = 0
  print("avg dist from object in front", avg)
  #e = avg-d
  return e

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_error_node', anonymous = True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()
