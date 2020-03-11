#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
import numpy as np

# TODO: modify these constants to make the car follow walls smoothly.
KP = 40
KD = 1
KI= 0.1

KPV=4
KDV=0.3
err_i=0
count=0
#Ierrs= np.array([0.0,0.0])
errs = np.array([0.0,0.0])
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
err_i_sum=0
# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
def control_callback(data):
  # TODO: Based on the error (data.data), determine the car's required velocity and steering angle.
    global errs
    global err_i
    global err_i_sum
    e = data.data

    errs[0] = errs[1]  #shift data up in the array
    errs[1] = e

    err_p = e
    err_d = errs[1] - errs[0]

    global count
    
    if count ==100:
         err_i = err_i_sum/100
         count=0
    count +=1
    err_i_sum+=e
    
    steer_d = np.clip(KP*err_p + KD*err_d + KI*err_i, -30.0, 30.0)
    steer = steer_d/ 180.0 * np.pi   #convert in to radians
    
    
    velocity = np.clip(1 + KPV*err_p + KDV*err_d, 0, 1.0)
    
    if np.mod(count,10)==0:
        print("Errors", err_p, err_d,err_i, velocity,steer_d)

    msg = drive_param()
    msg.velocity = 1  #TODO: implement PID for velocity 0.3
    msg.angle = steer    # TODO: implement PID for steering angle
    pub.publish(msg)


# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_controller_node', anonymous=True)
	rospy.Subscriber("pid_error", Float64, control_callback)
	rospy.spin()
