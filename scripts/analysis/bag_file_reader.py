import rosbag
import pdb
import math
import numpy as np
import matplotlib.pyplot as plt
import topic_parser

''' Function to load TF, Steering Report, and Acceleration data from a rosbag '''
def parse_and_interpolate_bag(bagfile='../../data/sample_data_2.bag', plot=False, save=False):

	bag = rosbag.Bag(bagfile)

	# Data to save
	tfd = np.array([]) # TF Topic raw data: x,y,psi,
	srd = np.array([]) # Steering Report Topic raw data: steering wheel angle, "" cmd, speed.
	acc = np.array([]) # Acceleration generated from Steering Report Data.
	paths = np.array([])

	# Read data from the bag file.
	print 'Extracting TF data'
	N_reserved = bag.get_message_count('/tf')
	N_actual = 0
	tfd = np.ones((N_reserved, 4)) * np.nan
	for topic, msg, t in bag.read_messages(topics=['/tf']):
		if msg.transforms[0].child_frame_id != 'vehicle/base_footprint':
			continue
		
		(ht, x, y, psi) = topic_parser.parse_tf_message(msg)
		tfd[N_actual] = np.array([ht, x, y, psi])
		N_actual = N_actual + 1

	tfd = tfd[~np.isnan(tfd).any(axis=1)]
	assert tfd.shape[0] == N_actual

	print 'Extracting SteeringReport data'
	N_reserved = bag.get_message_count('/vehicle/steering_report')
	N_actual = 0
	srd = np.ones((N_reserved, 4)) * np.nan
	for topic, msg, t in bag.read_messages(topics=['/vehicle/steering_report']):
		(ht, swa, swa_cmd, speed) = topic_parser.parse_sr_message(msg)
		srd[N_actual] = np.array([ht,swa,swa_cmd,speed])
		N_actual = N_actual + 1
	srd = srd[~np.isnan(srd).any(axis=1)]
	assert srd.shape[0] == N_actual

	print 'Extracting Target Path data'
	N_reserved = bag.get_message_count('/vehicle/target_path')
	N_actual = 0
	paths = np.ones((N_reserved, 50, 4)) * np.nan
	for topics, msg, t in bag.read_messages(topics=['/vehicle/target_path']):
		path_arr = topic_parser.parse_path_message(msg)

		if path_arr is None:
			continue

		paths[N_actual, :,:] = path_arr
		N_actual = N_actual + 1	
	paths = paths[~np.isnan(paths).any(axis=(1,2))]
	assert paths.shape[0] == N_actual
	bag.close()

	print 'Computing Acceleration'
	a = topic_parser.compute_acc_from_vel(srd[:,0], srd[:,3])
	acc = np.column_stack((srd[:-1,0], a))

	print 'Running Interpolation'
	# Interpolation of raw data.
	tm_min = min( np.min(tfd[:,0]), np.min(srd[:,0]) )
	tm_max = max( np.max(tfd[:,0]), np.max(srd[:,0]) )

	tm_interp = np.arange(tm_min, tm_max, 0.1) # 10 Hz

	tfd_interp = topic_parser.interpolate_matrix(tm_interp, tfd[:,0], tfd[:,1:])
	srd_interp = topic_parser.interpolate_matrix(tm_interp, srd[:,0], srd[:,1:])
	acc_interp = topic_parser.interpolate_matrix(tm_interp, acc[:,0], acc[:,1:])


	if plot:	
		''' Plotting '''
		plt.figure()

		plt.subplot(511)
		plt.plot(tfd[:,1], tfd[:,2], 'k')
		plt.plot(tfd_interp[:,1], tfd_interp[:,2], 'ro')
		plt.xlabel('X')
		plt.ylabel('Y')

		plt.subplot(512)
		plt.plot(srd[:,0], srd[:,3], 'k')
		plt.plot(srd_interp[:,0], srd_interp[:,3], 'ro')
		plt.ylim([0,20])
		plt.ylabel('V')

		plt.subplot(513)
		plt.plot(tfd[:,0], tfd[:,3], 'k')
		plt.plot(tfd_interp[:,0], tfd_interp[:,3], 'ro')
		plt.ylabel('Psi')

		plt.subplot(514)
		plt.plot(srd[:,0], srd[:,1], color='k', label='Actual')
		plt.plot(srd_interp[:,0], srd_interp[:,1], 'ro')
		#plt.plot(srd[:,0], srd[:,2], color='r', label='Desired')
		plt.ylabel('St Angle')

		plt.subplot(515)
		plt.plot(acc[:,0],acc[:,1], 'k')
		plt.plot(acc_interp[:,0],acc_interp[:,1], 'ro')
		plt.ylabel('Acc')

		plt.tight_layout()
		plt.show()

	if save:
		np.save('tfd_interp.npy', tfd_interp)
		np.save('srd_interp.npy', srd_interp)
		np.save('acc_interp.npy', acc_interp)

	return (tfd, srd, acc, paths, tfd_interp, srd_interp, acc_interp)
    
if(__name__=="__main__"):
	parse_and_interpolate_bag(plot=True)