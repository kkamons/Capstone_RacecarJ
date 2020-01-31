#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
import numpy as np

# TODO: modify these constants to make the car follow walls smoothly.
KP = 40
KD = 8

errs = np.array([0.0,0.0])

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
def control_callback(data):
  # TODO: Based on the error (data.data), determine the car's required velocity
  # amd steering angle.
        global errs
        
	e = data.data
	errs[0] = errs[1]
	errs[1] = e
        err_p = e
        err_d = errs[1] - errs[0]
        steer = np.clip(KP*err_p + KD*err_d, -20.0, 20.0)
        steer = steer / 180.0 * np.pi
        print("errs and steer", err_p, err_d, steer)
	msg = drive_param()
	msg.velocity = 0.7  # TODO: implement PID for velocity
	msg.angle = steer    # TODO: implement PID for steering angle

	pub.publish(msg)


# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_controller_node', anonymous=True)
	rospy.Subscriber("pid_error", Float64, control_callback)
	rospy.spin()


