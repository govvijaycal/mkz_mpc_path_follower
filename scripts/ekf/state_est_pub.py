#!/usr/bin/env python
import rospy
import numpy as np
import utm
	
from nmea_msgs.msg import Sentence				# GPS (latitude, longitude, altitude + covariances)
from dbw_mkz_msgs.msg import SteeringReport		# Velocity and Steering Wheel Angle
from sensor_msgs.msg import Imu					# IMU (orientation)
from std_msgs.msg import Float64

import ekf_state_estimation as ekf_se

import state_est_utils as se_utils

''' 
Global variables:
	entry 0 = mean, entry 1 = variance
	"o_" indicates an observation/measurement.
	"u_" indicates a control input.
'''
o_lat = np.ones(2) * np.nan 		# GPS latitude
o_lon = np.ones(2) * np.nan 		# GPS longitude

o_x   = np.ones(2) * np.nan 		# x from GPS conversion
o_y   = np.ones(2) * np.nan      # y from GPS conversion
o_psi = np.ones(2) * np.nan 		# heading from IMU
o_v   = np.ones(2) * np.nan 		# velocity from vehicle steering report

u_df = np.zeros(1)		 		# tire angle from vehicle steering report
u_a  = np.zeros(1)		 		# acceleration from differentiation of o_v

t_start = np.nan
t_imu = np.nan
t_gps = np.nan
t_steer = np.nan
t_acc   = np.nan

''' 
"CONSTANTS"
	defines the vehicle model reference system
	chosen from MPC Lab's stochastic LC code
	right before a stop sign next to the Transportation Sustainability Research Center
'''
LAT0 = 37.9152509
LON0 = -122.3329762
PSI0 = 0.0 # heading is the same as GPS heading (x = E, y = N)

def sub_nmea_sentence(msg):
	global o_lat, o_lon, o_x, o_y, t_gps
	# Three types of lines:
	# $GPGGA ...
	# <BESTPOS ...
	# < SOL_COMPUTED ...
	# We want to parse line 3.  I assume it always has the substring "< " in it, unlike the other two.

	if('< ' not in msg.sentence):
		return;

	tm_secs = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
	lat, lon, alt, lat_sd, lon_sd, alt_sd = se_utils.parse_sentence(msg.sentence)

	
	t_gps = tm_secs
	o_lat[0] = lat; o_lat[1] = lat_sd
	o_lon[0] = lon; o_lon[1] = lon_sd

	ox, oy = se_utils.convert_latlon_to_xy_spherical(o_lat, o_lon, LAT0, LON0)


def sub_steer_report(msg):
	global o_v, u_df, t_steer
	t_steer = msg.stamp.secs + 1e-9 * msg.stamp.nsecs
	u_df = msg.steering_wheel_angle/14.8 # steering ratio from steering wheel to tire angle
											# tenth of a degree resolution in steer wheel angle

	o_v[0] = msg.speed
	o_v[1] = [0.01] 						# TODO: not sure about speed errors...

def sub_filt_acceleration(msg):
	global t_acc, u_a
	tm_now = rospy.Time.now()
	t_acc = tm_now.secs + 1e-9*tm_now.nsecs

	u_a = msg.data

def sub_imu(msg):
	global o_psi, t_imu
	
	t_acc = msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs

	ori = msg.orientation
	quaternion = (ori.x, ori.y, ori.z, ori.w)
	(roll_raw, pitch_raw, yaw_raw) = transformations.euler_from_quaternion(quaternion)
	o_psi = yaw_raw

	

def state_pub_loop():
	state_update_rate = 10.0
	r = rospy.Rate(state_update_rate) # 2 Hz
	dt = 1.0/state_update_rate

    x_est = np.zeros(4)
    P_est = np.eye(4)

	EKF = ekf_se.MKZ_EKF(x_est, P_est, dt)

	Q = np.diag([2.1308**2, 0.9093**2, 0.0159**2, 0.0635**2]) # dynamics model covariance from simulation results

	tm_st = rospy.Time.now()
	t_start = tm_st.secs + 1e-9*tm_st.nsecs	

    while not rospy.is_shutdown():

    	# UPDATE EKF ...
    	u = [u_df, u_a] 
    	y = [o_x[0], o_y[0], o_psi[0], o_v[0]]
    	
    	R = np.diag([o_x[1], o_y[1], o_psi[1], o_v[1]]) 	  # measurement model covariance from sensor reports
    	x_est, P_est = EKF.update(u,y,Q,R)

    	# PUBLISH RESULTS ...
        t_msg = Twist()
        t_msg.linear.x = random.uniform(0.0, 1.0)
        t_msg.angular.z = random.uniform(-0.5, 0.5)

        pub.publish(t_msg)
        r.sleep()


if __name__=="__main__":
	rospy.init_node('state_estimation', anonymous=True)

	# setup up publishers and subscribers
	rospy.Subscriber('nmea_sentence', Sentence, sub_nmea_sentence, queue_size=10)
	rospy.Subscriber('steering_report', SteeringReport, sub_steer_report, queue_size=10)
	rospy.Subscriber('imu/data_raw', Imu, sub_imu, queue_size=10)
	rospy.Subscriber('filtered_accel', Float64, sub_filt_acceleration, queue_size=10)

	rospy.Publisher('state_estimate', StateEstimate, queue_size=2)
	# main publishing loop

	state_pub_loop()