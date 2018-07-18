#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import math as m
import numpy as np
import pickle as pkl
import rosbag

# PATH PARSING FUNCTIONS.
''' Read in a dictionary of prerecorded paths from a saved .pkl file '''
def parse_pkl_data(filename= '/home/govvijay/catkin_ws/src/mkz_mpc_path_follower/data/sample_data_2_paths.pkl'):
    data = pkl.load( open(filename, 'rb'))
    return data # an array of dictionaries, each dictionary is a path.

def parse_bag_and_save(bag_filename = None, save_filename = None):
    if bag_filename is None or save_filename is None:
        raise ValueError("Please specify valid bag and save locations!")

    bag = rosbag.Bag(bag_filename)
    paths = []
    for topic, msg, t in bag.read_messages(topics=['/vehicle/target_path']):
        if len(msg.poses) is 0: # empty message
            continue
        paths.append(parse_msg(msg))        
    bag.close()    
    pkl.dump(ps, open(save_filename, "wb"))
    return paths

''' Process a nav_msgs/Path to get a path dictionary for further processing. '''
# This is replicated in mpc_cmd_pub.jl since I had issues converting the Julia representation
# into a Python nav_msgs/Path object.
def parse_msg(msg):
    if len(msg.poses) is 0: # empty message
        return None

    s_arr = [0.0]              # cumulative distance along the path
    x_arr = [0.0]              # x coord along path
    y_arr = [0.0]              # y coord along path
    psi_arr = [0.0]            # heading (wrt x) along path
    cumulative_dist = 0.0      # s

    for i in range(0, len(msg.poses)):
        position_curr = msg.poses[i].pose.position
        orientation_curr = msg.poses[i].pose.orientation
        if i is 0:
            cumulative_dist = cumulative_dist + m.sqrt(position_curr.x**2 + position_curr.y**2)
        else:
            position_prev = msg.poses[i-1].pose.position
            cumulative_dist = cumulative_dist + \
                m.sqrt( (position_curr.x - position_prev.x)**2 + \
                (position_curr.y - position_prev.y)**2 )

        s_arr.append(cumulative_dist)
        x_arr.append(position_curr.x)
        y_arr.append(position_curr.y)
        psi_arr.append(2.0*m.atan2(orientation_curr.z,orientation_curr.w)) # quaternion to heading angle conversion

    path = {}
    path['x'] = x_arr
    path['y'] = y_arr
    path['s'] = s_arr
    path['psi'] = psi_arr
    return path