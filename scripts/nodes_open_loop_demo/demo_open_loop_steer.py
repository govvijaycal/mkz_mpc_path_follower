#!/usr/bin/env python
import rospy
from mkz_mpc_path_follower.msg import MPC_cmd
from std_msgs.msg import Empty

# Open loop tests to evaluate low-level control tracking of tire angle (d_f) input.
# NOTE: Assumes d_f in radians.  This may change with ECU reprogramming!

def pub_steer(mode='step'):
	enable_pub = rospy.Publisher("vehicle/enable", Empty, queue_size =10, latch=True)
	disable_pub = rospy.Publisher("vehicle/disable", Empty, queue_size =10, latch=True)
	pub = rospy.Publisher("vehicle/mpc_cmd", MPC_cmd, queue_size =10)
	rospy.init_node('pub_steer_ol', anonymous=True)
	r = rospy.Rate(10.0)

	while(enable_pub.get_num_connections == 0):
		print 'Waiting to enable steering!'
		r.sleep()

	enable_pub.publish(Empty()); # enable steering control.

	t_start = rospy.Time.now()
	rospy.loginfo('OL Steer Test Started at %f' % (t_start.secs + t_start.nsecs*1e-9))
	while not rospy.is_shutdown():
		# 14.5 steering ratio so 0.01 df -> 8.3 deg swa
		t_now = rospy.Time.now()

		dt = t_now - t_start
		dt_secs = dt.secs + 1e-9 * dt.nsecs

		if mode is 'step':
			st_des = step_response(dt_secs)
		elif mode is 'ramp':
			st_des = ramp_response(dt_secs)
		else:
			st_des = 0.0

		print('M: %s, T: %f, DF: %f' % (mode, dt_secs, st_des))
		
		m = MPC_cmd()
		m.accel_cmd = 0.0
		m.steer_angle_cmd = st_des
		pub.publish(m)
		
		r.sleep()

	disable_pub.publish(Empty()); # disable steering control.

def step_response(dt_secs):
	if dt_secs < 1.0:		# [0-1 s]
		st_des = 0.0
	elif dt_secs < 3.0:		# [1-3 s]
		st_des = 0.01
	elif dt_secs < 4.0:		# [3-4 s]
		st_des = 0.0
	elif dt_secs < 6.0:		# [4-6 s]
		st_des = -0.01
	else:					# [6+ s]
		st_des = 0.0			

	return st_des

def ramp_response(dt_secs):
	if dt_secs < 1.0:		# [0-1 s]
		st_des = 0.0
	elif dt_secs < 3.0:		# [1-3 s]
		st_des = 0.01 * (dt_secs - 1.0)/2.0
	elif dt_secs < 4.0:		# [3-4 s]
		st_des = 0.01 - 0.01 * (dt_secs - 3.0)/1.0
	elif dt_secs < 6.0:		# [4-6 s]
		st_des = -0.01 * (dt_secs - 4.0)/2.0
	else:					# [6+ s]
		st_des = 0.0

	return st_des

if __name__=="__main__":
	try:
		pub_steer(mode='step')
	except rospy.ROSInterruptException:
		pass # to handle node shutdown cleanly
