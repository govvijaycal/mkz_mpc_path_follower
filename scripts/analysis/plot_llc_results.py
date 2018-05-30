import pdb
import matplotlib.pyplot as plt
import numpy as np

def read_llc_log(filename):
	epoch_tm = [] 
	ros_tm = []
	th_cmd = []
	bk_cmd = []
	st_cmd = []
	a_act = []
	a_ref = []
	st_act = []
	st_ref = []

	with open(filename) as logfile:
		for line in logfile:
			if('ThCmd' in line):
				#print line
				res = line.split(',')
				
				epoch_tm.append( float(res[0].split('[')[-1]) )
				ros_tm.append( float(res[1].split(']')[0]) )
				
				th_cmd.append( float(res[1].split(':')[-1]) )
				bk_cmd.append( float(res[2].split(':')[-1]) )
				st_cmd.append( float(res[3].split(':')[-1]) )

				a_act.append( float(res[4].split(':')[-1]) )
				a_ref.append( float(res[5].split(':')[-1]) )
				st_act.append( float(res[6].split(':')[-1]) )
				st_ref.append( float(res[7].split(':')[-1]) )

				#print 'ETM: %f, RTM: %f' % (epoch_tm[-1], ros_tm[-1])
				#print 'TH: %f, BK: %f, ST: %f' % (th_cmd[-1], bk_cmd[-1], st_cmd[-1])
				#print 'A_ACT: %f,  A_REF: %f' % (a_act[-1], a_ref[-1])
				#print 'ST_ACT: %f,  ST_REF: %f\n' % (st_act[-1], st_ref[-1])

	print ros_tm[0]
	ros_tm = [x-ros_tm[0] for x in ros_tm]

	result_dict = {}
	for key in ['epoch_tm', 'ros_tm', 'th_cmd', 'bk_cmd', 'st_cmd', 'a_act', 'a_ref', 'st_act', 'st_ref']:
		result_dict[key] = locals()[key]

	return result_dict

def lpf_data(data, alpha):
	lpf_arr = [data[0]]

	for i in range(1, len(data)):
		if np.isnan(lpf_arr[-1]):
			next_val = data[i]
		elif np.isnan(data[i]):
			next_val = lpf_arr[-1]
		else:
			next_val = lpf_arr[-1]*alpha + data[i]*(1-alpha)

		lpf_arr.append(next_val)

	#print len(data), len(lpf_arr)
	return lpf_arr


def plot_llc_results(res_dict):

	d_ros_tm = np.diff(np.array(res_dict['ros_tm']))
	d_st_act = np.diff(np.array(res_dict['st_act']))
	
	st_act_dot = lpf_data( np.divide(d_st_act, d_ros_tm), 0.8) # not robust against division by 0, but just sets those entries to 0.
	st_act_ddot = lpf_data( np.divide(np.diff(st_act_dot), d_ros_tm[:-1]), 0.8)

	#pdb.set_trace()

	fig = plt.figure()
	plt.subplot(411)
	plt.plot(res_dict['ros_tm'], res_dict['a_ref'], color='k', label='A_REF')
	plt.plot(res_dict['ros_tm'][::10], res_dict['a_act'][::10], marker='x' , color='r', label='A_ACT')
	plt.ylabel('Acc (m/s^2)')
	#plt.legend()
	
	plt.subplot(412)
	plt.plot(res_dict['ros_tm'], res_dict['st_ref'], color='k', label='ST_REF')
	plt.plot(res_dict['ros_tm'][::10], res_dict['st_act'][::10], marker='x' , color='r', label='ST_ACT')
	plt.ylabel('SWA (rad)')
	#plt.legend()

	plt.subplot(413)
	ax1 = plt.gca()
	ax2 = ax1.twinx()
	ax1.plot(res_dict['ros_tm'], res_dict['th_cmd'], color='r', label='TH')
	ax2.plot(res_dict['ros_tm'], res_dict['bk_cmd'], color='b', label='BK')
	ax1.set_ylabel('Th (%)', color='r')
	ax2.set_ylabel('Bk (N*m)', color='b')

	
	plt.subplot(414)
	#res_str = 'AVG = %.3f, MIN = %.3f, MAX = %.3f' % (np.mean(st_act_dot), np.min(st_act_dot), np.max(st_act_dot))
	plt.plot(res_dict['ros_tm'][:-1], st_act_dot, color='r', label='ST_ACT_DOT')
	#plt.plot(res_dict['ros_tm'][:-2], st_act_ddot, color='b', label='ST_ACT_DDOT')
	ax = plt.gca()
	ax.axhline(8.7, color='k', ls='dashed')
	ax.axhline(-8.7, color='k', ls='dashed')
	#plt.text(res_dict['ros_tm'][0], 30, res_str)
	#plt.legend()
	plt.ylabel('SWA_dot (rad/s)')
	plt.xlabel('ROS Time (s)')

	plt.tight_layout()
	plt.show()

if __name__=="__main__":
	logdir = "/home/govvijay/catkin_ws/src/mkz_mpc_path_follower/data/tests_3_24/test2/"
	#logdir = "/home/govvijay/.ros/log/latest/"
	fname = "vehicle-low_level_controller-15-stdout.log"
	log_results = read_llc_log(logdir+fname)
	plot_llc_results(log_results)
