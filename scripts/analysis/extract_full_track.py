import bag_file_reader as bfr
import numpy as np
import pdb
import matplotlib.pyplot as plt

def load_and_save_track(bag_dir='../../data', save_dir='../../data/full_track'):
	bfile = '%s/full_data.bag' % bag_dir
	tfd, _, _, paths, _, _, _ = bfr.parse_and_interpolate_bag(bagfile=b_file)
	
	start_index = 44330 # this is set to start the track when the vehicle is roughly aligned with centerline
	
	track_txy = tfd[44330:,0:3] # t,x,y
	
	np.save('%s/track_txy_full.npy' % save_dir, track_txy) 		# TRACK CLEANED UP
	np.save('%s/tf_full_track.npy' % save_dir, tfd)				# FULL BAG DATA (UNPROCESSED)
	np.save('%s/target_path_full_track.npy' % save_dir, paths)	# CENTERLINE DETECTION RESULTS.

if __name__=='__main__':
	#load_and_save_track()
	
#############################################################################################################
# MISC CODE SNIPPETS
#############################################################################################################
''' 
	#Unused: meant to convert a path in local frame to global frame by aligining
	#with vehicle's reference frame at time the path is generated
def convert_local_path_to_global_frame(path, vehicle_frame_origin_global):
	x_offset = vehicle_frame_origin_global['x']			# x_0 (global frame)
	y_offset = vehicle_frame_origin_global['y']			# y_0 (global frame)
	psi_offset = vehicle_frame_origin_global['psi']		# psi_0 (global frame)

	xy_path_local_frame = path[:,0:2] # matrix with rows [x', y'] of path points in vehicle local frame
	R_loc_to_glob = np.array([ [np.cos(psi_offset), -np.sin(psi_offset)], \
									 [np.sin(psi_offset),   np.cos(psi_offset)] ])

	xy_path_global_frame = np.dot( xy_path_local_frame, R_loc_to_glob.T ) + np.array([ x_offset, y_offset ])

	psi_global_frame = path[:,2] + psi_offset

	#pdb.set_trace()

	return xy_path_global_frame, psi_global_frame
'''
#############################################################################################################
'''
	# Get the distance profile along the track.  Not sure if this is needed, unused for now.
	s = 0.0
	s_arr = [0.0]
	for i in range(start_index, tfd.shape[0] - 1):
		s = s + np.linalg.norm( tfd[i+1,1:3] - tfd[i,1:3] )
		s_arr.append(s)
	'''
	# This is to get the paths shifted to global coordinates.  Not used.
	'''
	paths = np.load('../../data/full_track/target_path_full_track.npy')

	shifted_paths = np.ones((paths.shape[0], paths.shape[1], 3)) * np.nan

	for p_num in range(paths.shape[0]):
		t_stamp = paths[p_num,0,0]

		vehicle_origin = {}
		vehicle_origin['x'] = np.interp( t_stamp, tfd[:,0], tfd[:,1] ) 	# x_0
		vehicle_origin['y'] = np.interp( t_stamp, tfd[:,0], tfd[:,2] )		# y_0
		vehicle_origin['psi'] = np.interp( t_stamp, tfd[:,0], tfd[:,3] )	# psi_0

		xy_global, psi_global = convert_local_path_to_global_frame(paths[p_num,:,1:], vehicle_origin)

		shifted_paths[p_num,:,:] = np.column_stack((xy_global, psi_global))

	plt.figure()
	plt.ion()
	plt.plot(tfd[:,1], tfd[:,2], 'r:')
	plt.show()
	plt.pause(0.0001)
	for i in range(shifted_paths.shape[0]):
		plt.plot(shifted_paths[i,:,0], shifted_paths[i,:,1], 'bx')
		plt.show()
		plt.pause(0.0001)
	'''
#############################################################################################################