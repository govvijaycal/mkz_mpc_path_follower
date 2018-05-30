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
	dt      = 0.20			# seconds                     
	N       = 8				# horizon
   
	x_ref   = 15.0*collect(0.0:dt:N*dt)			# x(0.0 s), ..., x(N*dt s)
	y_ref   = zeros(N+1)			# y(0.0 s), ..., y(N*dt s)
    psi_ref = zeros(N+1)			# psi(0.0 s), ..., psi(N*dt s)

    steer_max = 0.1 		# radians (steering angle bound, i.e. on d_f)
    steer_dmax = 0.1		# radians/sec (steering angle rate bound, i.e. on d_f derivative)

    a_max = 3.0				# m/s*2 (acceleration bound)

    # Cost factors:
    C_x = 1
    C_y = 1
    C_psi = 0.1
    C_v   = 0.1
    C_acc = 0.1
    C_df  = 0.5

	#### (2) Define State/Input Variables and Constraints ####
	# states: position (x,y), velocity (v), yaw angle (psi)
	# inputs: steering_angle and acceleration.
	println("Creating kinematic bicycle model ....")
	@variable( mdl, x[1:(N+1)], start=0.0)
	@variable( mdl, y[1:(N+1)], start=0.0)
	@variable( mdl, v[1:(N+1)], start=0.0)
	@variable( mdl, psi[1:(N+1)], start=0.0) # TODO: may need to constrain to +/- pi...

	@variable( mdl, -a_max <= acc[1:N] <= a_max, start=0.0)
	@variable( mdl, -steer_max <= d_f[1:N] <= steer_max, start=0.0)

    for i in 1:(N-1)
        @constraint(mdl, -steer_dmax*dt <= d_f[i+1] - d_f[i] <= steer_dmax*dt)
    end

	#### (3) Define Objective ####

	@NLparameter(mdl, x_r[i=1:(N+1)] == x_ref[i]) # Reference trajectory can be updated.
	@NLparameter(mdl, y_r[i=1:(N+1)] == y_ref[i])
	@NLparameter(mdl, psi_r[i=1:(N+1)] == psi_ref[i])
	@NLparameter(mdl, v_target == 15.0)

    @NLobjective(mdl, Min, sum{C_x*(x[i] - x_r[i])^2 + C_y*(y[i] - y_r[i])^2 + C_psi*(psi[i] - psi_r[i])^2 , i=2:(N+1)} + 
                           C_v *sum{ (v[i] - v_target)^2, i = 1:N} + 
                           C_acc*sum{acc[i]^2, i=1:N} +
                           C_df*sum{d_f[i]^2, i=1:N}
                )
    #@NLobjective(mdl, Min, (x[N+1] - x_r[N+1])^2 + (y[N+1] - y_r[N+1])^2 ) # terminal cost only

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

    function update_reference(x_ref::Array{Float64,1}, y_ref::Array{Float64,1}, psi_ref::Array{Float64,1})
    	setvalue(x_r[i=1:(N+1)], x_ref) # Reference trajectory can be updated.
    	setvalue(y_r[i=1:(N+1)], y_ref) # Reference trajectory can be updated.
    	setvalue(psi_r[i=1:(N+1)], psi_ref) # Reference trajectory can be updated.
    end

    function solve_model_with_ref(x_ref::Array{Float64,1}, y_ref::Array{Float64,1}, psi_ref::Array{Float64,1})
        update_reference(x_ref, y_ref, psi_ref)
        return solve_model()
    end

    function solve_model()
        # Solve the model.
        status = solve(mdl)

        # get optimal solutions
        d_f_opt = getvalue(d_f[1:N])
        acc_opt = getvalue(acc[1:N])

        #= 
        # For debugging: print out solution.
        x_act = getvalue(x[1:(N+1)])
        y_act = getvalue(y[1:(N+1)])
        psi_act = getvalue(psi[1:(N+1)])

        t_ref = collect(0.0:dt:(N*dt))
        println(@sprintf("Solve Status: %s", status))
        for i in range(1, length(t_ref))
            println(@sprintf("t: %.3f", t_ref[i]))
            println(@sprintf("\tX: %.3f\tX_des: %.3f", x_act[i], x_ref[i]))
            println(@sprintf("\tY: %.3f\tY_des: %.3f", y_act[i], y_ref[i]))
            println(@sprintf("\tPsi: %.3f", psi_act[i]))

            if i < length(t_ref)
                println(@sprintf("\td_f: %.3f\n", d_f_opt[i]))
                println(@sprintf("\tacc: %.3f\n", acc_opt[i]))
            else
                println("")
            end

        end
        =#

        return acc_opt[1], d_f_opt[1], status
    end

	function get_solver_results()
		# State Variables and Reference
		x_mpc   = getvalue(x[1:(N+1)])
		y_mpc   = getvalue(y[1:(N+1)])
		v_mpc   = getvalue(v[1:(N+1)])
		psi_mpc = getvalue(psi[1:(N+1)])

		x_ref   = getvalue(x_r[1:(N+1)])
		y_ref   = getvalue(y_r[1:(N+1)])
		psi_ref = getvalue(psi_r[1:(N+1)])

		# Optimal Solution
        d_f_opt = getvalue(d_f[1:N])
        acc_opt = getvalue(acc[1:N])

		return x_mpc, y_mpc, v_mpc, psi_mpc, x_ref, y_ref, psi_ref, d_f_opt, acc_opt	
	end

end
