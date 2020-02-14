#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
import numpy as np

# TODO: modify these constants to make the car follow walls smoothly.
KP = 40
KD = 8

errs = np.array([0.0,0.0])
#errs_L = np.array([0.0,0.0])
#errs_R = np.array([0.0,0.0])

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
def control_callback(data):
  # TODO: Based on the error (data.data), determine the car's required velocity
  # amd steering angle.
    #global errs_L
    #global errs_R
    global errs

	e = data.data
    #Le = data.Ldata
    #Re= data.Rdata
    #if Le> Re:
    	#errs_L[0] = errs_L[1]  #shift data up in the array
    	#errs_L[1] = Le
	errs[0] = errs[1]  #shift data up in the array
    	errs[1] = e

        #err_Lp = Le
        #err_Ld = errs_L[1] - errs_L[0]
	err_p = e
        err_d = errs[1] - errs[0]

        #steer_L = np.clip(KP*err_Lp + KD*err_Ld, -20.0, 20.0)
        #steer_L = steer_L / 180.0 * np.pi
        #steer = steer_L
	
	steer = np.clip(KP*err_p + KD*err_d, -40.0, 40.0)
        steer = steer / 180.0 * np.pi
        print("errs_L and steer", err_p, err_d, steer)

 """-------------------------------------------------------"""
<<<<<<< HEAD
    #if RE>LE:
     #   errs_R[0] = errs_R[1]  #shift data up in the array
     #   errs_R[1] = Re
=======
    if Re>Le:
        errs_R[0] = errs_R[1]  #shift data up in the array
        errs_R[1] = Re
>>>>>>> 5f0ba1286d7fc9e38e10fbe697d4bcc36c215eaf

      #  err_Rp = Re
      #  err_Rd = errs_R[1] - errs_R[0]

     #   steer_R = np.clip(KP*err_Rp + KD*err_Rd, -20.0, 20.0)
     #   steer_R = -steer_R / 180.0 * np.pi
     #   print("errs_R and steer", err_Rp, err_Rd, steer_R)
     #   steer = steer_R

#	msg = drive_param()

#	msg.velocity = 0.7  # TODO: implement PID for velocity
#	msg.angle = steer    # TODO: implement PID for steering angle
#	pub.publish(msg)


# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_controller_node', anonymous=True)
	rospy.Subscriber("pid_error", Float64, control_callback)
	rospy.spin()
