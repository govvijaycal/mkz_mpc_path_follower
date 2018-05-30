#!/usr/bin/env julia

#=
 This is a modified version of controller_MPC.jl from the barc project ROS codebase.

 Licensing Information: You are free to use or extend these projects for 
 education or reserach purposes provided that (1) you retain this notice
 and (2) you provide clear attribution to UC Berkeley, including a link 
 to http://barc-project.com

 Attibution Information: The barc project ROS code-base was developed
 at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
 (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
 by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was 
 based on an open source project by Bruce Wootton
=# 

module KinMPCPathFollower

    using JuMP
    using Ipopt

    #### (1) Set up model and model parameters ####
    mdl = Model(solver = IpoptSolver(print_level=0, max_cpu_time = 0.1))

	L_a     = 1.108 		# meters
	L_b     = 1.742 		# meters
	dt      = 0.20			# seconds for discretization (td)                
	N       = 8				# horizon

	dt_control = 0.10		# control period (ts)
   
	v_ref = 10.0

    steer_max = 0.5 		# tire angle bound, rad
    steer_dmax = 0.5		# tire angle rate bound, rad/s

	a_max = 1.0				# acceleration and deceleration bound, m/s^2
	a_dmax = 1.5			# jerk bound, m/s^3
	
    # Cost factors:
    C_y = 10.0
    C_psi = 500.0
    C_v   = 1.0

	C_dacc	 = 1.0
	C_ddf	 = 1.0
	C_acc	 = 0.0
	C_df	 = 1000.0

	x_ref = zeros(N+1)
	y_ref = zeros(N+1)
	psi_ref = zeros(N+1)

	#### (2) Define State/Input Variables and Constraints ####
	# states: position (x,y), velocity (v), yaw angle (psi)
	# inputs: steering_angle and acceleration.
	println("Creating kinematic bicycle model ....")
	@variable( mdl, x[1:(N+1)], start=0.0)
	@variable( mdl, y[1:(N+1)], start=0.0)
	@variable( mdl, v[1:(N+1)], start=0.0)
	@variable( mdl, psi[1:(N+1)], start=0.0)

	# Input Constraints
	@variable( mdl, -a_max <= acc[1:N] <= a_max, start=0.0)
	@variable( mdl, -steer_max <= d_f[1:N] <= steer_max, start=0.0)

	# Input Steering Rate Constraints
	@NLparameter(mdl, d_f_current == 0.0)
	@NLconstraint(mdl, -steer_dmax*dt_control <= d_f[1]  - d_f_current <= steer_dmax*dt_control)
    for i in 2:(N-1)
        @constraint(mdl, -steer_dmax*dt <= d_f[i+1] - d_f[i] <= steer_dmax*dt)
    end

	# Input Acceleration Rate Constraints
	@NLparameter(mdl, acc_current == 0.0)
	@NLconstraint(mdl, -a_dmax*dt_control <= acc[1]  - acc_current <= a_dmax*dt_control)
    for i in 2:(N-1)
        @constraint(mdl, -a_dmax*dt <= acc[i+1] - acc[i] <= a_dmax*dt)
    end	

	#### (3) Define Objective ####

	@NLparameter(mdl, ypoly[i=1:4] == zeros(4)[i] ) # Polynomial(X) reference trajectory
	@NLparameter(mdl, psipoly[i=1:4] == zeros(4)[i] )
	@NLexpression(mdl, yp[i=1:(N+1)], ypoly[1] + ypoly[2]*x[i] + ypoly[3]*x[i]^2 + ypoly[4]*x[i]^3)
	@NLexpression(mdl, psip[i=1:(N+1)], psipoly[1] + psipoly[2]*x[i] + psipoly[3]*x[i]^2 + psipoly[4]*x[i]^3)
	@NLparameter(mdl, v_target == v_ref) # target speed

    @NLobjective(mdl, Min, C_y * sum{(y[i] - yp[i])^2, i=2:(N+1)} + 
						   C_psi * sum{(psi[i] - psip[i])^2, i=2:(N+1)} +
                           C_v *sum{ (v[i] - v_target)^2, i = 2:N} + 
                           C_acc*sum{(acc[i])^2, i=1:N} +
                           C_df*sum{(d_f[i])^2, i=1:N} +
						   C_dacc*sum{(acc[i+1] - acc[i])^2, i=1:(N-1)} +
                           C_ddf*sum{(d_f[i+1] - d_f[i])^2, i=1:(N-1)}
				)

	#### (4) Define System Dynamics Constraints ####
	# Reference: R.Rajamani, Vehicle Dynamics and Control, set. Mechanical Engineering Series,
	#               Spring, 2011, page 26

    # Initial condition and velocity can be updated.
	@NLparameter(mdl, x0     == 0.0); @NLconstraint(mdl, x[1]     == x0);    
	@NLparameter(mdl, y0     == 0.0); @NLconstraint(mdl, y[1]     == y0);    
	@NLparameter(mdl, psi0   == 0.0); @NLconstraint(mdl, psi[1]   == psi0);
    @NLparameter(mdl, v0     == 0.0); @NLconstraint(mdl, v[1]     == v0);

	@NLexpression(mdl, bta[i = 1:N], atan( L_a / (L_a + L_b) * tan(d_f[i]) ) )

	for i in 1:N
        # equations of motion wrt CoG
		@NLconstraint(mdl, x[i+1]   == x[i]   + dt*( v[i]*cos(psi[i] + bta[i]) ) )
		@NLconstraint(mdl, y[i+1]   == y[i]   + dt*( v[i]*sin(psi[i] + bta[i]) ) )
		@NLconstraint(mdl, psi[i+1] == psi[i] + dt*( v[i]/L_b*sin(bta[i]) ) )

        # equations of motion wrt base link
		#@NLconstraint(mdl, x[i+1]   == x[i]   + dt*( v[i]*cos(psi[i]+ bta[i])  + v[i]*sin(psi[i])*sin(bta[i]) ) )
		#@NLconstraint(mdl, y[i+1]   == y[i]   + dt*( v[i]*sin(psi[i] + bta[i]) - v[i]*cos(psi[i])*sin(bta[i]) ) )
		#@NLconstraint(mdl, psi[i+1] == psi[i] + dt*( v[i]/L_b * sin(bta[i]) ) )
        @NLconstraint(mdl, v[i+1]   == v[i]   + dt*( acc[i] ) )

	end

    #### (5) Initialize Solver ####
	println("initial solve ...")
	status = solve(mdl)
	println("finished initial solve: ", status)

    function update_init_cond(x::Float64, y::Float64, psi::Float64, vel::Float64)
        # update mpc initial condition 
        setvalue(x0,    x)
        setvalue(y0,    y)
        setvalue(psi0,  psi)
        setvalue(v0,    vel)
    end

    function update_reference(xr::Array{Float64,1}, yr::Array{Float64,1}, pr::Array{Float64,1}, yp::Array{Float64,1}, pp::Array{Float64,1})
    	setvalue(ypoly[i=1:4], yp) # Reference trajectory can be updated.
    	setvalue(psipoly[i=1:4], pp) # Reference trajectory can be updated.

		global x_ref, y_ref, psi_ref
		x_ref = xr
		y_ref = yr
		psi_ref = pr
    end

	function update_current_input(c_swa::Float64, c_acc::Float64)
		setvalue(d_f_current, c_swa)
		setvalue(acc_current, c_acc)
	end

    function solve_model()
        # Solve the model.
		#global last_swa_command, last_acc_command
        #setvalue(d_f_current, last_swa_command)
	    #setvalue(acc_current, last_acc_command)

        status = solve(mdl)

        # get optimal solutions
        d_f_opt = getvalue(d_f[1:N])
        acc_opt = getvalue(acc[1:N])

        return acc_opt[1], d_f_opt[1], status
    end

	function get_solver_results()
		# State Variables and Reference
		x_mpc   = getvalue(x[1:(N+1)])
		y_mpc   = getvalue(y[1:(N+1)])
		v_mpc   = getvalue(v[1:(N+1)])
		psi_mpc = getvalue(psi[1:(N+1)])

		y_poly = getvalue(ypoly[1:4])
		psi_poly = getvalue(psipoly[1:4])

		# Optimal Solution
        d_f_opt = getvalue(d_f[1:N])
        acc_opt = getvalue(acc[1:N])

		global x_ref, y_ref, psi_ref

		return x_mpc, y_mpc, v_mpc, psi_mpc, x_ref, y_ref, psi_ref, y_poly, psi_poly, d_f_opt, acc_opt	
	end

end
