#!/usr/bin/env python
import rospy
from mkz_mpc_path_follower.msg import MPC_cmd
from std_msgs.msg import Empty

# Open loop tests to evaluate low-level control tracking of acceleration input.

def pub_acc(mode='step'):
	enable_pub = rospy.Publisher("vehicle/enable", Empty, queue_size =10, latch=True)
	disable_pub = rospy.Publisher("vehicle/disable", Empty, queue_size =10, latch=True)
	pub = rospy.Publisher("vehicle/mpc_cmd", MPC_cmd, queue_size =10)
	rospy.init_node('pub_accel_ol', anonymous=True)

	r = rospy.Rate(10.0)

	while(enable_pub.get_num_connections == 0):
		print 'Waiting to enable acceleration!'
		r.sleep()

	enable_pub.publish(Empty()); # enable acceleration control.

	t_start = rospy.Time.now()
	rospy.loginfo('OL Acc Test Started at %f' % (t_start.secs + t_start.nsecs*1e-9))
	while not rospy.is_shutdown():
		t_now = rospy.Time.now()

		dt = t_now - t_start
		dt_secs = dt.secs + 1e-9 * dt.nsecs
		
		if mode is 'step':
			a_des = step_response(dt_secs)
		elif mode is 'ramp':
			a_des = ramp_response(dt_secs)
		else:
			a_des = 0.0

		print('M: %s, T: %f, A: %f' % (mode, dt_secs, a_des))

		m = MPC_cmd()
		m.accel_cmd = a_des
		m.steer_angle_cmd = 0.0
		pub.publish(m)
		r.sleep()
		
	disable_pub.publish(Empty()); # disable acceleration control.

def step_response(dt_secs):
	if dt_secs < 1.0:		# [0-1 s]
		a_des = 0.0
	elif dt_secs < 6.0:		# [1-6 s]
		a_des = 1.0
	elif dt_secs < 7.0:		# [6-7 s]
		a_des = 0.0
	elif dt_secs < 12.0:	# [7-12 s]
		a_des = -1.0
	else:					# [12+ s]
		a_des = 0.0				
	return a_des


def ramp_response(dt_secs):
	if dt_secs < 1.0:		# [0-1 s]
		a_des = 0.0
	elif dt_secs < 6.0:		# [1-6 s]
		a_des = 1.0 * (dt_secs - 1.0)/5.0
	elif dt_secs < 7.0:		# [6-7 s]
		a_des = 1.0 - (dt_secs - 6.0)/1.0
	elif dt_secs < 12.0:	# [7-12 s]
		a_des = -1.0 * (dt_secs - 7.0)/5.0
	else:					# [12+ s]
		a_des = 0.0				
	return a_des

if __name__=="__main__":
	try:
		pub_acc(mode='step')
	except rospy.ROSInterruptException:
		pass # to handle node shutdown cleanly

