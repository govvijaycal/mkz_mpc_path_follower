import rosbag
import math
import numpy as np

''' Set of helper functions to convert a ROS message to a simple tuple/array for data analysis '''
def parse_tf_message(msg):
	header_time = msg.transforms[0].header.stamp.secs + msg.transforms[0].header.stamp.nsecs*1e-9
	x = msg.transforms[0].transform.translation.x	
	y = msg.transforms[0].transform.translation.y
	qz = msg.transforms[0].transform.rotation.z
	qw = msg.transforms[0].transform.rotation.w
	psi = 2 * math.atan2(qz, qw)

	return (header_time, x,y,psi)

def parse_sr_message(msg):
	header_time = msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9
	swa = msg.steering_wheel_angle
	swa_cmd = msg.steering_wheel_angle_cmd
	speed = msg.speed
	return (header_time, swa, swa_cmd, speed)

def parse_path_message(msg):
	header_time = msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9

	if len(msg.poses) is 0:
		return None

	arr = np.ones( (len(msg.poses), 4) )

	for i in range(len(msg.poses)):
		x  = msg.poses[i].pose.position.x
		y  = msg.poses[i].pose.position.y
		qz = msg.poses[i].pose.orientation.z
		qw = msg.poses[i].pose.orientation.w
		psi = 2 * math.atan2(qz, qw)

		arr[i] = np.array([header_time, x, y, psi])

	return arr

def compute_acc_from_vel(t, speed):
	tdiff = np.diff(t,axis=0)
	spdiff = np.diff(speed, axis=0)

	tiny_denom = np.where(tdiff < 1e-4)[0]

	if np.size(tiny_denom) > 0:
		print 'Warning: small time difference detected in divide!'

	acc = np.divide(spdiff, tdiff)

	return acc

def interpolate_matrix(tm_inter, tm_actual, data_actual):
	# tm_inter: timepoints at which to interpolate
	# tm_actual: timepoints of the collected data
	# data_actual: collected data points corresponding to tm_actual

	data_inter = np.ones((len(tm_inter), data_actual.shape[1])) * np.nan

    # np.interp does linear interpolation but also handles values outside of interpolation range
    # so need to do a correction to set the values outside interpolation range to NaN after this.
    # TODO: maybe don't need this correction, can check this later.
	for i in range(data_actual.shape[1]):
		y_actual = data_actual[:,i]
		y_inter = np.interp(tm_inter, tm_actual, y_actual)

		tm_min = np.min(tm_inter)
		tm_max = np.max(tm_inter)

		invalid_tm_points = np.logical_or( (tm_inter < tm_min), (tm_inter > tm_max) )

		y_inter[invalid_tm_points] = np.NaN

		data_inter[:,i] = y_inter

	data_inter = np.column_stack((tm_inter, data_inter))

	return data_inter