#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import math as m
import numpy as np
import pdb
import pickle as pkl
import rosbag
from nav_msgs_common import *

''' THIS VERSION TAKES IN WAYPOINTS AND FITS TRAJECTORY X(t), Y(t), Psi(t), t \in t_ref '''
##################################################################################################################
# MAIN FUNCTION FOR FITTING TRAJECTORY FROM PATH USING T_REF.
def get_reference_using_t(path, t_ref, v_ref):
    t_arr = np.array(path['s']) / v_ref
    x_interp = np.interp(t_ref, t_arr, path['x'])      # x_des   = f_interp(t_des, t_actual, x_actual)
    y_interp = np.interp(t_ref, t_arr, path['y'])      # y_des   = f_interp(t_des, t_actual, y_actual)
    p_interp = np.interp(t_ref, t_arr, path['psi'])    # psi_des = f_interp(t_des, t_actual, psi_actual)
    return x_interp, y_interp, p_interp
##################################################################################################################
# FUNCTIONS FOR DEBUGGING/VERIFICATION.
def plot_ref_traj(x_arr, y_arr, x_interp, y_interp, f_handle = None, plt_delay=0.05):
    # Handle figure creation/clearing existing figures.
    if f_handle is None:
        f_handle = plt.figure()
    else:
        plt.figure( f_handle.number )
    
    plt.ion()
    plt.clf()
    
    plt.plot(x_arr, y_arr, 'k', lw = 2, label='ACT')                # Actual path from message.
    plt.plot(x_interp, y_interp, 'rx', markersize = 5, label='REF') # Reference trajectory generated from message using t_ref.
    
    plt.xlim([0,50]); plt.ylim([-25,25])
    plt.axis('equal')
    plt.legend()        
    plt.show()
    plt.pause(plt_delay)

    return f_handle

if __name__ == "__main__":
    paths = parse_pkl_data()
    t_ref = np.arange(0.0, 1.61, 0.2)
    v_ref = 15.0

    f = None                                                                        # figure handle for plotting
    for path in paths:
        x_interp, y_interp, _ = get_reference_using_t(path, t_ref, v_ref)
        f = plot_ref_traj(path['x'], path['y'], x_interp, y_interp, f_handle=f)     # plot original path and generated trajectory.