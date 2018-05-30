import bag_file_reader
import math
import matplotlib.pyplot as plt
import numpy as np
import pdb

def vehicle_model(x0, y0, psi0, v0, a, d_f, dt, lr = 1.1076, lf = 1.7422):
	# Vehicle Model and Parameters
	L  = lr + lf
	steer_ratio = 14.8
	beta = math.atan2(lr*math.tan(d_f), L)
	x1 = x0 + dt *( v0 * math.cos(psi0 + beta) )
	y1 = y0 + dt *( v0 * math.sin(psi0 + beta) )
	psi1 = psi0 + dt * ( v0/lr * math.sin(beta) )
	v1 = v0 + dt * a

	return (x1, y1, psi1, v1)

def eval_model(tfd, srd, acc, tfd_interp, srd_interp, acc_interp, lr = 1.1076, lf = 1.7422, plot_on=False):
	steer_ratio = 14.8

	t   = tfd_interp[:,0]
	x   = tfd_interp[:,1]
	y   = tfd_interp[:,2]
	psi = tfd_interp[:,3]
	vel = srd_interp[:,3]
	
	swa = srd_interp[:,1]
	d_f = swa/steer_ratio
	acc = acc_interp[:,1] 
	x_pred = []
	y_pred = []
	psi_pred = []
	vel_pred = []

	for i in range(len(t)):
		if i == 0:
			x_pred.append(x[0])
			y_pred.append(y[0])
			psi_pred.append(psi[0])
			vel_pred.append(vel[0])

		else:
			xcurr   = x_pred[-1]
			ycurr   = y_pred[-1]
			psicurr = psi_pred[-1]
			velcurr = vel_pred[-1]
			dt = t[i] - t[i-1]

			(xp,yp,psip,vp) = \
				vehicle_model(xcurr, ycurr, psicurr, velcurr, acc[i-1], d_f[i-1], dt, lr, lf)

			x_pred.append(xp)
			y_pred.append(yp)
			psi_pred.append(psip)
			vel_pred.append(vp)

	error_x = np.linalg.norm(x-x_pred, ord=2)
	error_y = np.linalg.norm(y-y_pred, ord=2)
	error_psi = np.linalg.norm(psi-psi_pred, ord=2)
	error_vel = np.linalg.norm(vel-vel_pred, ord=2)
	
	#pdb.set_trace()

	if plot_on:
		plt.figure()

		n_pts = len(x_pred)

		plt.subplot(611)
		plt.plot(t, x, 'k')
		plt.plot(t[0:n_pts:10], x_pred[0:n_pts:10], 'ro')
		#plt.plot(t, x-x_pred)
		plt.ylabel('X')

		plt.subplot(612)
		plt.plot(t, y, 'k')
		plt.plot(t[0:n_pts:10], y_pred[0:n_pts:10], 'ro')
		#plt.plot(t, y-y_pred)
		plt.ylabel('Y')

		plt.subplot(613)
		plt.plot(t, psi, 'k')
		plt.plot(t[0:n_pts:10], psi_pred[0:n_pts:10], 'ro')
		#plt.plot(t, psi-psi_pred)
		plt.ylabel('Psi')

		plt.subplot(614)
		plt.plot(t, vel, 'k')
		plt.plot(t[0:n_pts:10], vel_pred[0:n_pts:10], 'ro')
		#plt.plot(t, vel-vel_pred)
		plt.ylabel('Vel')

		plt.subplot(615)
		plt.plot(t, acc, 'b')
		plt.ylabel('Acc')

		plt.subplot(616)
		plt.plot(t, d_f, 'g')
		plt.ylabel('D_f')

		plt.tight_layout()
		plt.show()
	
	return error_x, error_y, error_psi, error_vel

if(__name__=="__main__"):
	bag_file = '/home/govvijay/catkin_ws/src/mkz_mpc_path_follower/data/sample_data_2.bag'

	(tfd, srd, acc, paths, tfd_interp, srd_interp, acc_interp) = \
		bag_file_reader.parse_and_interpolate_bag(bagfile=bag_file, plot=False)

	# Just test the model.
	ex,ey,epsi,evel = eval_model(tfd, srd, acc, tfd_interp, srd_interp, acc_interp, plot_on=True)

	# Sweep lr:
	'''
	L = 2.8498

	lr_cands = np.arange(0.0, L, 0.1)
	lf_cands = L - lr_cands

	exs = []
	eys = []
	epsis = []
	evels = []

	for i in range(len(lr_cands)):
		ex,ey,epsi,evel = eval_model(tfd, srd, acc, tfd_interp, srd_interp, acc_interp, lr=lr_cands[i], lf=lf_cands[i])
		exs.append(ex)
		eys.append(ey)
		epsis.append(epsi)
		evels.append(evel)

	plt.figure()

	plt.subplot(411)
	plt.plot(lr_cands, exs)
	plt.ylabel('EX')

	plt.subplot(412)
	plt.plot(lr_cands, eys)
	plt.ylabel('EY')

	plt.subplot(413)
	plt.plot(lr_cands, epsis)
	plt.ylabel('EPsi')

	plt.subplot(414)
	plt.plot(lr_cands, evels)
	plt.ylabel('EVel')

	plt.tight_layout()
	plt.show()
	'''



