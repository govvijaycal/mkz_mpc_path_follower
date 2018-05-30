#!/usr/bin/env python

# A modified version of state_estimation_SensorKinematicModels.py, observers.py
# and system_models.py from the BARC workspace tailored for the Lincoln MKZ.
# Edited by Vijay Govindarajan (govvijay@berkeley.edu).
# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for 
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link 
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was 
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------

from numpy import array, dot, eye, copy
from numpy import dot, zeros
from scipy.linalg import inv
import rospy

class MKZ_EKF:
	''' Public API '''
	def __init__(self, z_0, P_0, dt, l_f = 1.108, l_r = 1.742):
		self.z_est = z_0
		self.P_est = P_0
		
		self.l_f = l_f
		self.l_r = l_r
		self.dt = dt

	def update(self, u, y, Q, R):
		"""
	    Extended Kalman Filter Update for Lincoln MKZ:
	    ekf_update(self, u, y, Q, R) returns state estimate, z and state covariance, P:
	    	z_k+1 = _f_MKZ(z_k, u_k) + w_k
			y_k   = _h_MKZ(z_k, u_k) + v_k
	    where  w ~ N(0,Q): w is Gaussian Noise with mean 0, covariance Q
	           v ~ N(0,R): v is Gaussian Noise with mean 0, covariance R
	    Inputs:    u_k:	  current input
	               y_kp1: current measurement
	               Q_k: process noise covariance 
	               R_k: measurement noise covariance

	               mz_k: previous state estimate is a class variable (z_est)
	               P_k : previous state covariance is a class variable (P_est)
	    Output:    mz_kp1: "a posteriori" state estimate
	               P_kp1: "a posteriori" state covariance
	               
	    Notation: mz_k = E[z_k] and my_k = E[y_k], where m = mean.
	    """
	    mz_k = self.z_est
	    P_k  = self.P_est
	    
	    zDim    = mz_k.size                         # dimension of the state
	    mz_kp1  = self._f_MKZ(mz_k, u)              # predict next state a priori
	    A       = self._jacobian_f_MKZ(mz_k,u)      # linearize process model about current state
	    P_kp1   = dot(dot(A,P_k),A.T) + Q           # propagate variance
	    my_kp1  = self._h_MKZ(mz_kp1)               # predict future output
	    H       = self._jacobian_h_MKZ(mz_kp1)      # linearize measurement model about predicted next state
	    P12     = dot(P_kp1, H.T)                   # cross covariance
	    K       = dot(P12, inv( dot(H,P12) + R))    # Kalman filter gain
	    mz_kp1  = mz_kp1 + dot(K,(y_kp1 - my_kp1))  # state estimate a posteriori
	    P_kp1   = dot(dot(K,R),K.T) + dot( dot( (eye(zDim) - dot(K,H)) , P_kp1)  ,  (eye(zDim) - dot(K,H)).T ) 

	    # Update class variable estimates and then return the same information.
	    self.z_est = mz_kp1
	    self.P_est = P_kp1
	    return (mz_kp1, P_kp1)		

	''' "Private" helper functions '''
	def _f_MKZ(self, z, u):
	''' 
	    Process Model without Drift Compensation
	    Input: state z[k] and control u[k] at current time step
	    Output: state at next time step z[k+1]
	    '''
	    # get states / inputs
	    x       = z[0] # position 
	    y       = z[1]  
	    psi     = z[2] # heading
	    v       = z[3] # velocity

	    d_f     = u[0] # tire angle
	    a       = u[1] # acceleration

	    # extract parameters
	    L_a = self.l_f
	    L_b = self.l_r
	    dt = self.dt

	    # compute slip angle
	    bta         = arctan( L_a / (L_a + L_b) * tan(d_f) )

	    # compute next state
	    x_next      = x + dt*( v*cos(psi + bta) )
	    y_next      = y + dt*( v*sin(psi + bta) )
	    psi_next    = psi + dt*v/L_b*sin(bta)
	    v_next      = v + dt*(a)

	    return array([x_next, y_next, psi_next, v_next])

	def _h_MKZ(self, z):
		 ''' 
		 Measurement Model
		 Input and Output: current state estimate z[k]
		 '''
		return z

	def _jacobian_f_MKZ(z,u):
		"""
	    Function to compute the numerical jacobian of a vector valued function 
	    using final differences
	    """
	    # numerical gradient and diagonal hessian
	    y = self._f_MKZ(z, u)
	    
	    jac = zeros( (y.size,x.size) )
	    eps = 1e-5
	    xp = copy(x)
	    
	    for i in range(x.size):
	        xp[i] = x[i] + eps/2.0
	        yhi = self._f_MKZ(xp, u)
	        xp[i] = x[i] - eps/2.0
	        ylo = self._f_MKZ(xp, u)
	        xp[i] = x[i]
	        jac[:,i] = (yhi - ylo) / eps
	    return jac

	def _jacobian_h_MKZ(z):
		return np.eye(z.size) # h(z) = z, so jacobian should be identity matrix

'''
Original EKF Code used for reference:
directory: https://github.com/MPC-Berkeley/barc/tree/LMPC/workspace/src/barc/src
(1) system_models.py
(2) observers.py
(3) state_estimation_SesnorKinematicModel.py
'''