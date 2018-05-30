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
    vel = 15.0				# meters/second (longitudinal velocity)
   
	x_ref   = vel*collect(0:dt:(N*dt)) # x(0.0 s), ..., x(N*dt s)
	y_ref   = zeros(N+1)			# y(0.0 s), ..., y(N*dt s)
    psi_ref = zeros(N+1)			# psi(0.0 s), ..., psi(N*dt s)

    yaw_rate_bound = 0.5

	#### (2) Define State/Input Variables and Constraints ####
	# states: position (x,y), yaw angle (psi); we assume fixed velocity.
	# inputs: yaw rate (ang_z)
	println("Creating kinematic bicycle model ....")
	@variable( mdl, x[1:(N+1)], start=0.0)
	@variable( mdl, y[1:(N+1)], start=0.0)
	@variable( mdl, psi[1:(N+1)], start=0.0) # TODO: may need to constrain to +/- pi...
	@variable( mdl, -yaw_rate_bound <= ang_z[1:N] <= yaw_rate_bound, start=0.0)

	#### (3) Define Objective ####

	@NLparameter(mdl, x_r[i=1:(N+1)] == x_ref[i]) # Reference trajectory can be updated.
	@NLparameter(mdl, y_r[i=1:(N+1)] == y_ref[i])

    @NLobjective(mdl, Min, sum{(x[i] - x_r[i])^2 + (y[i] - y_r[i])^2, i=1:(N+1)})
    #@NLobjective(mdl, Min, (x[N+1] - x_r[N+1])^2 + (y[N+1] - y_r[N+1])^2 ) # final state matters

	#### (4) Define System Dynamics Constraints ####
	# Reference: R.Rajamani, Vehicle Dynamics and Control, set. Mechanical Engineering Series,
	#               Spring, 2011, page 26

    # Initial condition and velocity can be updated.
	@NLparameter(mdl, x0     == 0.0); @NLconstraint(mdl, x[1]     == x0);    
	@NLparameter(mdl, y0     == 0.0); @NLconstraint(mdl, y[1]     == y0);    
	@NLparameter(mdl, psi0   == 0.0); @NLconstraint(mdl, psi[1]   == psi0);
    @NLparameter(mdl, v == vel);

	for i in 1:N
        # simple kinematic model
		@NLconstraint(mdl, x[i+1]   == x[i]   + dt*( v*cos(psi[i]) ) )
		@NLconstraint(mdl, y[i+1]   == y[i]   + dt*( v*sin(psi[i]) ) )
		@NLconstraint(mdl, psi[i+1] == psi[i] + dt*( ang_z[i] ) )
	end


    #### (5) Initialize Solver ####
	println("initial solve ...")
	status = solve(mdl)
	println("finished initial solve: ", status)

    function getSpeed()
        return getvalue(v)
    end

    function update_init_cond(x::Float64, y::Float64, psi::Float64, vel::Float64)
        # update mpc initial condition 
        setvalue(x0,    x)
        setvalue(y0,    y)
        setvalue(psi0,  psi)
        setvalue(v,    vel)
    end

    function update_reference(x_ref::Array{Float64,1}, y_ref::Array{Float64,1})
    	setvalue(x_r[i=1:(N+1)], x_ref) # Reference trajectory can be updated.
    	setvalue(y_r[i=1:(N+1)], y_ref) # Reference trajectory can be updated.
    end

    function solve_model_with_ref(x_ref::Array{Float64,1}, y_ref::Array{Float64,1})
        update_reference(x_ref, y_ref)
        solve_model()
    end

    function solve_model()
        # Solve the model.
        status = solve(mdl)

        # get optimal solutions
        ang_z_opt = getvalue(ang_z[1:N])
        psi_act = getvalue(psi[1:(N+1)])

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
                println(@sprintf("\tang_z: %.3f\n", ang_z[i]))
            else
                println("")
            end

        end
        =#

        return ang_z_opt[1]
    end

end
