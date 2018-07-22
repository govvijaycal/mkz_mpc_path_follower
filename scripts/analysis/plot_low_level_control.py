import matplotlib.pyplot as plt
import rosbag
import argparse
import numpy as np

def make_llc_plot(bagfile):
	b = rosbag.Bag(bagfile)
	t_se = []; a_se = []; df_se = []	# /vehicle/state_est (EST)
	t_mpc = []; a_mpc = []; df_mpc = [] # /vehicle/mpc_cmd (MPC CMD)

	t_af = []; a_f = []					# /vehicle/filtered_accel (LL MEAS)
	t_ar = []; a_r = []					# /vehicle/req_accel (LL CMD)

	t_df = []; df_c = []; df_a = []		# cmd in /vehicle/steering_report

	#t_en = []; en = []
	t_en = None

	
	'''
	for topic, msg, t in b.read_messages(topics='/vehicle/state_est'):
		t_se.append(msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs)
		a_se.append(msg.a)
		df_se.append(msg.df)
	'''
	for topic, msg, t in b.read_messages(topics='/vehicle/mpc_cmd'):
		t_mpc.append(t.secs + 1e-9 * t.nsecs)
		a_mpc.append(msg.accel_cmd)
		df_mpc.append(msg.steer_angle_cmd)

	for topic, msg, t in b.read_messages(topics='/vehicle/filtered_accel'):
		t_af.append(msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs)
		a_f.append(msg.accel_value)

	
	for topic, msg, t in b.read_messages(topics='/vehicle/req_accel'):	
		t_ar.append(msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs)
		a_r.append(msg.accel_value)
	

	for topic, msg, t in b.read_messages(topics='/vehicle/steering_report'):	
		t_df.append(msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs)
		df_c.append(msg.steering_wheel_angle_cmd/14.8)
		df_a.append(msg.steering_wheel_angle/14.8)

	## ASSUMPTION: ENABLED IS ONLY TURNED ON ONCE!
	for topic, msg, t in b.read_messages(topics='/vehicle/dbw_enabled'):
		if msg.data == True:
			t_en = t.secs + 1e-9 * t.nsecs
			print 'DBW Enabled: %f' % t_en
			break
		'''
		t_en.append(t.secs + 1e-9 * t.nsecs)
		if msg.data == True:
			en.append(1)
		else:
			en.append(0)
		'''
	t_offset = t_en
	t_mpc = np.array(t_mpc) - t_offset
	t_af  = np.array(t_af) - t_offset
	t_df  = np.array(t_df) - t_offset

	plt.figure()
	plt.subplot(211)
	#plt.plot(t_se, a_se, 'b', label='SE')
	plt.plot(t_mpc, a_mpc, 'r', label='MPC')
	plt.plot(t_af, a_f, 'k', label='FILT')
	'''
	for i in range(len(en)):
		if en[i] == 1:
			plt.axvline(t_en[i], c='g')
		else:
			plt.axvline(t_en[i], c='m')
	'''
	#plt.plot(t_ar, a_r, 'g', label='REQ')
	plt.xlabel('t (s)')
	plt.ylabel('Accel (m/s^2)')
	plt.legend()

	plt.subplot(212)
	#plt.plot(t_se, df_se, 'b', label='SE')
	plt.plot(t_mpc, df_mpc, 'r', label='MPC')
	plt.plot(t_df, df_a, 'k', label='FILT')
	'''
	for i in range(len(en)):
		if en[i] == 1:
			plt.axvline(t_en[i], c='g')
		else:
			plt.axvline(t_en[i], c='m')
	'''
	#plt.plot(t_df, df_c, 'g', label='REQ')
	plt.xlabel('t (s)')
	plt.ylabel('D_f (rad)')
	

	plt.suptitle('Low Level Tracking Response')
	plt.show()

if __name__=='__main__':
	parser = argparse.ArgumentParser('Plots MPC command vs. actual low level acceleration + steering controller behavior')
	parser.add_argument('--bf',  type=str, required=True, help='Bag file for path followed.')
	args = parser.parse_args()
	make_llc_plot(args.bf)
