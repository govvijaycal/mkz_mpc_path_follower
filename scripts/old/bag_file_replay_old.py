import rosbag
import pdb
import math
import numpy as np

bag = rosbag.Bag('../sample_data.bag')

'''
# Data from the Steering Report topic.
arr_t_sr = []      # steering report header time (s)
arr_swa_sr = []    # steering wheel angle (rad?)
arr_swacmd_sr = [] # desired steering wheel angle
arr_speed_sr = []     # velocity


# Data from the Target Path topic.
for topic, msg, t in bag.read_messages(topics=['/vehicle/target_path']): 
	#'/vehicle/steering_report', '/tf', 

    print msg.poses[0]
    pdb.set_trace()

	print msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9
	#print msg

    
	for i in range(1, len(msg.poses)):
		position_curr = msg.poses[i].pose.position
		position_prev = msg.poses[i-1].pose.position
		dist = math.sqrt( (position_curr.x - position_prev.x)**2 + \
    	       (position_curr.y - position_prev.y)**2 )
		print '%d: %f' % (i, dist)
'''

# Test transforming waypoints into time indices.
v = 15.0 # m/s
for topic, msg, t in bag.read_messages(topics=['/vehicle/target_path']):
    cumulative_dist = 0.0
    t_arr = []
    pos_arr = []
    for i in range(0, len(msg.poses)):
        if i is 0:
            position_curr = msg.poses[i].pose.position
            cumulative_dist = cumulative_dist + math.sqrt(position_curr.x**2 + position_curr.y**2)
            
            t_arr.append(cumulative_dist/v)
            pos_arr.append(position_curr)
        else:
            position_curr = msg.poses[i].pose.position
            position_prev = msg.poses[i-1].pose.position
            cumulative_dist = cumulative_dist + \
                math.sqrt( (position_curr.x - position_prev.x)**2 + \
                (position_curr.y - position_prev.y)**2 )

            t_arr.append(cumulative_dist/v)
            pos_arr.append(position_curr)

    print "%d Poses, %d Times, %d Positions\n" % (len(msg.poses), len(t_arr), len(pos_arr))

    t_ref = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6]
    x_ref = []
    y_ref = []

    for t_des in t_ref:
        diff_t = np.square(np.array(t_arr) - t_des)
        diff_min_ind = np.argmin(diff_t)
        t_closest = t_arr[diff_min_ind]

        if t_closest < t_des:

            if diff_min_ind == len(t_arr): 
                # if t_des is greater than all times in t_arr, use endpoint of trajectory.
                x_des = pos_arr[-1].x
                y_des = pos_arr[-1].y
            else:
                # t_closest = t_m, t_m < t_des < t_m+1
                #x_des = x_m + (x_m+1 - x_m)/(t_m+1 - t_m) * (t_des - t_m), linear interpolation between closest point m and next point m+1
                interval_diff = t_arr[diff_min_ind + 1] - t_closest
                interp_diff = t_des - t_closest            

                x_des = pos_arr[diff_min_ind].x + \
                        (pos_arr[diff_min_ind + 1].x - pos_arr[diff_min_ind].x)/interval_diff * interp_diff
                y_des = pos_arr[diff_min_ind].y + \
                        (pos_arr[diff_min_ind + 1].y - pos_arr[diff_min_ind].y)/interval_diff * interp_diff
        else:
            if diff_min_ind == 0:
                # if t_des is less than all times in t_arr, interpolate between (0,0) and first point.
                x_des = pos_arr[0].x * t_des/t_closest
                y_des = pos_arr[0].y * t_des/t_closest
            else:
                # t_closest = t_m, t_m-1 < t_des < t_m
                # x_des = x_m-1 + (x_m - x_m-1)/(t_m - t_m-1) * (t_des - t_m-1), linear interpolation between prev point m-1 and closest point m
                interval_diff = t_closest - t_arr[diff_min_ind - 1]
                interp_diff = t_des - t_arr[diff_min_ind - 1]            

                x_des = pos_arr[diff_min_ind-1].x + \
                        (pos_arr[diff_min_ind].x - pos_arr[diff_min_ind-1].x)/interval_diff * interp_diff
                y_des = pos_arr[diff_min_ind-1].y + \
                        (pos_arr[diff_min_ind].y - pos_arr[diff_min_ind-1].y)/interval_diff * interp_diff

        x_ref.append(x_des)
        y_ref.append(y_des)

    for j in range(0, len(t_arr)):
        print "t: %f\tX: %f\ty: %f\n" % (t_arr[j], pos_arr[j].x, pos_arr[j].y)

    for k in range(0, len(t_ref)):
        print "td: %f\tXd: %f\tyd: %f\n" % (t_ref[k], x_ref[k], y_ref[k])

    pdb.set_trace()

bag.close()

