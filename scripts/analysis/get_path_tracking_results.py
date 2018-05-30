import bag_file_reader as bfr
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import pdb
import scipy.spatial.distance as ssd
import time

def load_tf_information(bagfile=None):
	if bagfile is None:
		ValueError('Please provide a valid bag file name to parse!')

	return bfr.parse_and_interpolate_bag(bagfile=bagfile)[0]

def compute_path_errors(xy_ref_track, xy_actual):
	# Input: m x n X1 and p x n X2 matrices.
	# Output: m x p matrix D with D[i,j] = dist(X1[i,:], X2[j,:])
	errors = np.ones(xy_actual.shape[0]) * np.nan
	closest_pt_ind = np.ones(xy_actual.shape[0]) * np.nan

	for st_ind in np.arange(0, xy_actual.shape[0], 1000):
		print '%d of %d' % (st_ind, xy_actual.shape[0])
		end_ind = min(xy_actual.shape[0], st_ind + 1000)
		dist = ssd.cdist(xy_ref_track, xy_actual[st_ind:end_ind])
		closest_pt_ind[st_ind:end_ind] = np.argmin(dist, axis=0)
		errors[st_ind:end_ind] = dist[ closest_pt_ind[st_ind:end_ind].astype(int) , np.arange(dist.shape[1]) ]

	return errors, closest_pt_ind

def compute_histogram(errors, inds, pre_string='', ignore_ind_min=158000, ignore_ind_max=161000):
	# ignore_ind_min and ignore_ind_max are to remove parts of the track from the histogram results
	# you can use this to remove the initial condition region, where the initial position error is ~ 3 m (lane change)
	keep_inds = np.logical_or( inds<ignore_ind_min, inds>ignore_ind_max )
	error_processed = errors[keep_inds]
	plt.figure()
	plt.hist(error_processed, color='k')
	plt.xlabel('Path Error (m)')
	plt.ylabel('Frequency')
	plt.xlim([0.0, 2.0])
	plt.title('%s: Path Error Histogram' % pre_string)

	res_tuple = np.mean(error_processed), np.std(error_processed), np.median(error_processed), np.min(error_processed), np.max(error_processed)
	plt.axvline(x=res_tuple[0], c='r')
	plt.axvline(x=res_tuple[0] - res_tuple[1], c='b', ls=':')
	plt.axvline(x=res_tuple[0] + res_tuple[1], c='b', ls=':')
	plt.axvline(x=res_tuple[2], c='g')

	res_str = 'Mean, Std: %.3f, %.3f\nMedian: %.3f\nMin, Max: %.3f, %.3f' % res_tuple
	plt.text(0.7, 0.7, res_str, horizontalalignment='center', verticalalignment='center', transform=plt.gca().transAxes)
	return res_tuple

def convert_track_index_to_color(ind_vec, ind_min = int(0), ind_max = int(196041)):
	assert(np.ndim(ind_vec) == 1)
	assert(np.min(ind_vec) >= ind_min)
	assert(np.max(ind_vec) <= ind_max)
	return np.divide(ind_vec, float(ind_max+1))


def plot_track(track_ref, cmap_name='gist_rainbow'):
	plt.figure()
	plt.ion()
	ax = plt.gca()

	cmap = matplotlib.cm.get_cmap(cmap_name)

	inds = np.arange(0, track_ref.shape[0], 500)
	cols = convert_track_index_to_color(inds)
	xs = track_ref[inds,1]
	ys = track_ref[inds,2]
	ax.scatter(xs,ys, c=cols, cmap=cmap)
	plt.title('Track Reference')
	plt.show()

def plot_path_error(rostms, errors, closest_ind, track_ref, pre_string='', cmap_name='gist_rainbow', res_tuple=None):
	plt.figure()
	plt.ion()
	plt.plot(rostms - rostms[0], errors, 'k')
	plt.ylim([0,4])
	plt.xlabel('Ros Time Elapsed (s)')
	plt.ylabel('Path Error (m)')
	plt.title('%s: Path Error vs. Time' % pre_string)
	
	if res_tuple is not None:
		plt.axhline(y=res_tuple[0], c='r')
		plt.axhline(y=res_tuple[0] - res_tuple[1], c='b', ls=':')
		plt.axhline(y=res_tuple[0] + res_tuple[1], c='b', ls=':')
		plt.axhline(y=res_tuple[2], c='g')

	plt.figure()
	plt.ion()
	plt.subplot(311)
	ax = plt.gca()
	ax.scatter(closest_ind.astype(int), errors, c='k')
	plt.xlim([0,200000])
	plt.ylim([0,4])
	plt.xlabel('Track Index')
	plt.ylabel('Path Error (m)')

	if res_tuple is not None:
		plt.axhline(y=res_tuple[0], c='r')
		plt.axhline(y=res_tuple[0] - res_tuple[1], c='b', ls=':')
		plt.axhline(y=res_tuple[0] + res_tuple[1], c='b', ls=':')
		plt.axhline(y=res_tuple[2], c='g')

	cmap = matplotlib.cm.get_cmap(cmap_name)
	track_inds = np.arange(0,track_ref.shape[0],1000).astype(int)
	track_cols = convert_track_index_to_color(track_inds)

	plt.subplot(312)
	ax = plt.gca()
	ax.scatter(track_inds, track_ref[track_inds,1], c=track_cols, cmap=cmap)
	plt.xlim([0,200000])
	plt.xlabel('Track Index')
	plt.ylabel('X of Track (m)')
	
	plt.subplot(313)
	ax = plt.gca()
	ax.scatter(track_inds, track_ref[track_inds,2], c=track_cols, cmap=cmap)
	plt.xlim([0,200000])
	plt.xlabel('Track Index')
	plt.ylabel('Y of Track (m)')
	
	plt.suptitle('%s: Path Error Over the Track' % pre_string)
	plt.show()
 
if __name__ == '__main__':
	tf_track = np.load('../../data/full_track/track_txy_full.npy')

	f_track_v2 = np.load('../../data/frenet_v2/frenet_v2_txy.npy')
	errors_f2 = np.load('../../data/frenet_v2/frenet_v2_errors.npy')
	closest_f2 = np.load('../../data/frenet_v2/closest_v2.npy')

	path_error_stats_f2 = compute_histogram(errors_f2, closest_f2, pre_string='Frenet2')
	plot_path_error(f_track_v2[:,0], errors_f2, closest_f2, tf_track, pre_string='Frenet2', res_tuple=path_error_stats_f2)


	'''
	f_track = np.load('../../data/frenet/frenet_txy.npy')
	xy_track = np.load('../../data/xy/xy_txy.npy')
	
	#errors_f, closest_f = compute_path_errors(  tf_track[:,1:], f_track[:,1:3] )
	#np.save('../../data/frenet/frenet_errors.npy', errors_f)
	#np.save('../../data/frenet/frenet_indices.npy', closest_f)

	#errors_xy, closest_xy = compute_path_errors( tf_track[:,1:], xy_track[:,1:3] )
	#np.save('../../data/xy/xy_errors.npy', errors_xy)
	#np.save('../../data/xy/xy_indices.npy', closest_xy)
	

	f_errors = np.load('../../data/frenet/frenet_errors.npy')
	f_closest = np.load('../../data/frenet/frenet_indices.npy')
	xy_errors = np.load('../../data/xy/xy_errors.npy')
	xy_closest = np.load('../../data/xy/xy_indices.npy')

	path_error_stats_f = compute_histogram(f_errors, f_closest, pre_string='Frenet')
	plot_path_error(f_track[:,0], f_errors, f_closest, tf_track, pre_string='Frenet', res_tuple=path_error_stats_f)
	
	path_error_stats_xy = compute_histogram(xy_errors, xy_closest, pre_string='XY')
	plot_path_error(xy_track[:,0], xy_errors, xy_closest, tf_track, pre_string='XY', res_tuple=path_error_stats_xy)
	
	plot_track(tf_track)
	'''


	pdb.set_trace()

	
	
