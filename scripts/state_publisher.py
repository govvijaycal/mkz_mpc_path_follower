#!/usr/bin/env python
import rospy
from nmea_msgs.msg import Sentence
from dbw_mkz_msgs.msg import SteeringReport
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from mkz_mpc_path_follower.msg import state_est
from tf.transformations import euler_from_quaternion
import math as m

''' Global Variables for Callbacks '''
tm_gps = None; lat = None; lon = None
tm_vel = None; vel = None; acc_filt = None
tm_imu = None; psi = None
tm_df  = None;  df = None

def time_valid(ros_tm, tm_arr):
	tm_now = ros_tm.secs + 1e-9*ros_tm.nsecs
	for tm_rcvd in tm_arr:
		diff = m.fabs(tm_now - tm_rcvd)
		if diff > 0.05: # seconds
			return False
	return True

''' GPS Callback Functions '''
def parse_sentence(sent_str):
	spl = sent_str.split()

	sent_lat = float(spl[3])
	sent_lon = float(spl[4])
	sent_alt = float(spl[5])

	sent_lat_sd = float(spl[8])
	sent_lon_sd = float(spl[9])
	sent_alt_sd = float(spl[10])

	return sent_lat, sent_lon, sent_alt, sent_lat_sd, sent_lon_sd, sent_alt_sd

def parse_nmea(msg):
	# Three types of lines:
	# $GPGGA ...
	# <BESTPOS ...
	# < SOL_COMPUTED ...
	# We want to parse line 3.  I assume it always has the substring "< " in it, unlike the other two.

	if('< ' not in msg.sentence):
		return;

	global tm_gps, lat, lon
	tm_gps = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
	lat, lon, _, _, _, _ = parse_sentence(msg.sentence)

def latlon_to_XY(lat0, lon0, lat1, lon1):
	''' 
	Convert latitude and longitude to global X, Y coordinates,
	using an equirectangular projection.

	X = meters east of lon0
	Y = meters north of lat0

	Sources: http://www.movable-type.co.uk/scripts/latlong.html
		     https://github.com/MPC-Car/StochasticLC/blob/master/controller.py
	'''
	R_earth = 6371000 # meters
	delta_lat = m.radians(lat1 - lat0)
	delta_lon = m.radians(lon1 - lon0)

	lat_avg = 0.5 * ( m.radians(lat1) + m.radians(lat0) )
	X = R_earth * delta_lon * m.cos(lat_avg)
	Y = R_earth * delta_lat

	return X,Y

''' Steering/Velocity Callback Functions '''
def parse_steering_report(msg):	
	global tm_vel, vel, acc_filt
	global tm_df, df

	tm_df = tm_vel
	df = msg.steering_wheel_angle/14.8 # steering ratio from steering wheel to tire angle

	# Could use the filtered_accel topic instead...

	v_meas = msg.speed	
	if tm_vel is None:
		tm_vel = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
		acc_filt = 0.0

	else:
		dtm_vel = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs - tm_vel
		tm_vel = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
		acc_raw = (v_meas - vel)/dtm_vel
		acc_filt = 0.01 * acc_raw + 0.99 * acc_filt 
		# alpha = 0.01, alpha = dt/(Td + dt), dt~0.02 s -> first order time delay of 1.98 s
	vel = v_meas

''' Heading Callback Functions '''
def parse_imu(msg):
	global tm_imu, psi

	tm_imu = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
	ori = msg.orientation
	quat = (ori.x, ori.y, ori.z, ori.w)
	roll, pitch, yaw = euler_from_quaternion(quat)

	assert(-m.pi <= yaw)
	assert(m.pi >= yaw)

	psi = yaw

def pub_loop():
	rospy.init_node('state_publisher', anonymous=True)	
	rospy.Subscriber('/nmea_sentence', Sentence, parse_nmea, queue_size=1)
	rospy.Subscriber('/vehicle/steering_report', SteeringReport, parse_steering_report, queue_size=1)
	rospy.Subscriber('/xsens/imu/data', Imu, parse_imu, queue_size=1)
	#rospy.Subscriber('/vehicle/filtered_accel', Imu, parse_acc, queue_size=1)
	
	if not (rospy.has_param('lat0') and rospy.has_param('lon0') and rospy.has_param('yaw0')):
		raise ValueError('Invalid rosparam global origin provided!')

	if not rospy.has_param('is_heading_info'):
		raise ValueError('Invalid rosparam for if heading or yaw info provided!')

	if not rospy.has_param('time_check_on'):
		raise ValueError('Did not specify if time validity should be checked!')

	LAT0 = rospy.get_param('lat0')
	LON0 = rospy.get_param('lon0')
	YAW0 = rospy.get_param('yaw0')
	time_check_on = rospy.get_param('time_check_on')
	
	state_pub = rospy.Publisher('state_est', state_est, queue_size=1)

	r = rospy.Rate(100)
	while not rospy.is_shutdown():		
		
		if None in (lat, lon, psi, vel, acc_filt, df):			
			r.sleep()
			continue
		curr_state = state_est()
		curr_state.header.stamp = rospy.Time.now()
		
		# TODO: time validity check, only publish if data is fresh
		#if time_check_on and not time_valid(curr_state.header.stamp,[tm_vel, tm_df, tm_imu, tm_gps]):
		#	r.sleep()
		#	continue

		curr_state.lat = lat
		curr_state.lon = lon

		X,Y = latlon_to_XY(LAT0, LON0, lat, lon)

		curr_state.x   = X
		curr_state.y   = Y
		curr_state.psi = psi
		curr_state.v   = vel
		
		curr_state.a   = acc_filt
		curr_state.df  = df

		state_pub.publish(curr_state)
		r.sleep()

if __name__=='__main__':
	try:
		pub_loop()
	except rospy.ROSInterruptException:
		pass
