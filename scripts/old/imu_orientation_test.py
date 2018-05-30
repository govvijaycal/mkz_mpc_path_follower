#!/usr/bin/env python
import rospy
import numpy as np

from sensor_msgs.msg import Imu					# IMU (orientation)
import matplotlib.pyplot as plt
import pdb

from tf import transformations

arr_t = []
arr_r = []
arr_p = []
arr_y = []

def sub_imu(msg):
	global arr_r, arr_p, arr_y, arr_t
	ori = msg.orientation
	quaternion = (ori.x, ori.y, ori.z, ori.w)
	(roll_raw, pitch_raw, yaw_raw) = transformations.euler_from_quaternion(quaternion)

	tm = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs

	arr_t.append(tm)
	arr_r.append(roll_raw)
	arr_p.append(pitch_raw)
	arr_y.append(yaw_raw)

def end_callback():
	plt.figure()
	ts = [x - arr_t[0] for x in arr_t]
	plt.plot(ts, arr_y, 'r')
	plt.plot(ts, arr_p, 'g')
	plt.plot(ts, arr_r, 'b')
	plt.show()
	#pdb.set_trace()

if __name__=="__main__":
	rospy.init_node('imu_listener')
	#rospy.Subscriber('/xsens/imu/data', Imu, sub_imu, queue_size=10)
	rospy.Subscriber('/imu_raw', Imu, sub_imu, queue_size=10)
	rospy.on_shutdown(end_callback)
	rospy.spin()