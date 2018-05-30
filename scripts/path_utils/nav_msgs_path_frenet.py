#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import math as m
import numpy as np
from nav_msgs_common import *

''' THIS VERSION TAKES IN WAYPOINTS AND FITS FRENET FRAME TRAJECTORY s, K_poly(s) '''
##################################################################################################################
# HELPER FUNCTIONS FOR FITTING FRENET FRAME TRAJECTORY.
''' Polynomial Fit with Derivatives '''
def cubic_func(t,a3,a2,a1,a0):              # f(t)
    return a0 + a1*t + a2*t**2 + a3*t**3
def dcubic_func(t,a3,a2,a1,a0):             # df/dt
    return a1 + 2*a2*t  + 3*a3*t**2
def ddcubic_func(t,a3,a2,a1,a0):            # d2f/dt2
    return 2*a2 +  6*a3*t

''' Build a Path Using the Frenet Frame System and Initial Conditions '''
def reconstruct_frenet(x0,y0,p0,s_max, K_coeff):
    xr = [x0] # initial x (s=0)
    yr = [y0] # initial y (s=0)
    pr = [p0] # initial heading (atan2(dy, dx) evaluated at s = 0)

    s_interp = np.arange(0.0, s_max, 0.25) # s interpolation range

    for i in range(0, len(sa_interp) - 1):  # integrate ds to get the trajectory
        # Update position
        ds = s_interp[i+1] - s_interp[i]                
        xn = xr[-1] + ds * m.cos(pr[-1])    
        yn = yr[-1] + ds * m.sin(pr[-1])

        # Update heading
        k = cubic_func(sa_interp[i], *K_coeff) # compute curvature at s
        pn = pr[-1] + ds * k

        xr.append(xn)
        yr.append(yn)
        pr.append(pn)
    return xr, yr, pr, s_interp # return reconstructed path with heading

''' From Polynomials X(s) and Y(s), compute curvature K(s) '''
def compute_curvature_poly(s_interp, x_coeffs, y_coeffs):
    dx_interp = [dcubic_func(s,*x_coeffs) for s in s_interp]    # dx/ds
    dy_interp = [dcubic_func(s,*y_coeffs) for s in s_interp]    # dy/ds

    ddx_interp = [ddcubic_func(s,*x_coeffs) for s in s_interp]  # d2x/ds2
    ddy_interp = [ddcubic_func(s,*y_coeffs) for s in s_interp]  # d2y/ds2
        
    # Use the derivation of K = dtheta/dt, where t = s in our case from
    # http://mathworld.wolfram.com/Curvature.html (Eqn 12).
    # Note that X and Y are already parametrized by s, so we don't use Eqn 13.
    num = np.multiply(dx_interp, ddy_interp) - np.multiply(dy_interp, ddx_interp) # x'*y'' - y'*x''
    den = np.square(dx_interp) + np.square(dy_interp)                             # x'**2 + y'**2

    K_meas = np.divide(num,den)                                                   # K actual
    K_coeffs = np.polyfit(s_interp, K_meas, 3)                                    # K polynomial fit, highest degree first
    return K_coeffs

''' Fit X(s), Y(s) polynomials. '''
def fit_XY_s(x_arr, y_arr, s_arr):
    s_min = s_arr[0]
    s_max = s_arr[-1]
    s_interp = np.arange(s_min, s_max, 0.5)

    x_interp = np.interp(s_interp, s_arr, x_arr) # x_des   = f_interp(s_des, s_actual, x_actual)
    y_interp = np.interp(s_interp, s_arr, y_arr) # x_des   = f_interp(s_des, s_actual, x_actual)


    x_coeffs = np.polyfit(s_interp, x_interp, 3)  # X(s)
    y_coeffs = np.polyfit(s_interp, y_interp, 3)  # Y(s)
    return x_coeffs, y_coeffs
##################################################################################################################
# MAIN FUNCTION FOR FITTING FRENET FRAME TRAJECTORY REPRESENTATION FROM PATH.
def get_reference_frenet(path):
	x_coeffs, y_coeffs = fit_XY_s( path['x'], path['y'], path['s'] )  				# Fit X(s), Y(s) polynomials

	s_interp = np.arange(0.0, path['s'][-1], 0.25)                  				# interpolate fitted polynomials X(s), Y(s)
	x_interp = [cubic_func(s,*x_coeffs) for s in s_interp]
	y_interp = [cubic_func(s,*y_coeffs) for s in s_interp]

	K_coeffs = compute_curvature_poly(s_interp,x_coeffs,y_coeffs)   				# fit curvature polynomial from X(s), Y(s)        
	psi_start = m.atan2( dcubic_func(0.0,*y_coeffs) , dcubic_func(0.0,*x_coeffs) )  # desired initial path heading = atan2(dy/dx) at s = 0
	
	return K_coeffs, psi_start, x_interp, y_interp        
##################################################################################################################
# TEST FUNCTIONS FOR DEBUGGING/VERIFICATION.
def plot_frenet_path_fit(x_arr, y_arr, x_interp, y_interp, x_recon, y_recon, f_handle = None, plt_delay=0.05):
    # Handle figure creation/clearing existing figures.
    if f_handle is None:
        f_handle = plt.figure()
    else:
        plt.figure( f_handle.number )
    
    plt.ion()
    plt.clf()
    
    plt.plot(x_arr, y_arr, 'ko', markersize = 5, label='ACT') 	# Actual path from message.
    plt.plot(x_interp, y_interp, 'r', lw = 2, label='POLY')    	# Fitted path from polynomial X(s), Y(s)
    plt.plot(x_recon,y_recon, 'b', lw = 2, label='FRENET')      # Frenet-frame reconstructed path (s, K(s)).
    
    plt.xlim([0,50]); plt.ylim([-25,25])
    plt.axis('equal')
    plt.legend()        
    plt.show()
    plt.pause(plt_delay)

    return f_handle

if __name__ == "__main__":
    paths = parse_pkl_data()

    f = None # plotting figure handle
    for path in paths:
    	
    	K_coeffs, psi_start, x_interp, y_interp = get_reference_frenet(path)
        x_recon, y_recon, psi_recon = reconstruct_frenet(0.0, 0.0, psi_start, 40, K_coeffs) # reconstructed path using Frenet Frame
        f = plot_frenet_path_fit(path['x'], path['y'], x_interp, y_interp, x_recon, y_recon, f_handle=f)    # plot original and fitted paths to ensure consistency.