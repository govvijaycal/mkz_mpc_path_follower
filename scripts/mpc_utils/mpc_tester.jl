#!/usr/bin/env julia

include("kinematic_mpc_module.jl")
import KinMPCPathFollower

using PyPlot
using NPZ

function state_update_fncn(x0, y0, psi0, v0, a, df0, df_desired)

	L_a     = 1.108 		# meters
	L_b     = 1.742 		# meters
	dt      = 0.20			# seconds                     

    steer_max = 7.0 		# radians (steering wheel angle bound)
    steer_dmax = 7.0		# radians/sec (steering wheel angle rate)

    x = x0
    y = y0
    v = v0
    psi = psi0
    df = df0

    disturbance_norm = 0.0

    for i in 1:10
        #println(@sprintf("\t%f %f %f %f", x,y, psi,df))
        dt_sim = dt/10
        bta = atan( L_a / (L_a + L_b) * tan(df))

        dist = (rand(4) - 0.5) * 2.0 * disturbance_norm
        x = x + dt_sim * (v*cos(psi+ bta) + v*sin(psi)*sin(bta) + dist[1])
        y = y + dt_sim * (v*sin(psi + bta) - v*cos(psi)*sin(bta) + dist[2])
        psi = psi + dt_sim * (v/L_b * sin(bta) + dist[3]) 
        v = v + dt_sim * (a + dist[4])

        if df < df_desired
            df = (df_desired - df)/dt_sim > steer_dmax ?
                (df + dt_sim * steer_dmax) : df_desired
        elseif df > df_desired
            df = (df - df_desired)/dt_sim > steer_dmax ?
                (df - dt_sim * steer_dmax) : df_desired
        end

        if(df > steer_max)
            df = steer_max
        elseif(df < -steer_max)
            df = -steer_max
        end
    end

    return x, y, psi, v, df
end

function test_mpc_module()
    x     = 0.0
    y     = 0.0
    psi   = 0.0
    df    = 0.0

    v = 14.8				# meters/second (longitudinal velocity)

    # make a path
    t_ref = collect(0.0:0.2:10.0)
    x_ref = v * t_ref
    y_ref = sin(t_ref)#sin(t_ref) #zeros(length(t_ref))#t_ref

    # store history of path taken
    x_hist = [x]
    y_hist = [y]
    psi_hist = [psi]
    df_hist = [df]
    df_des_hist = []
     
    # for loop: 
    for i in range(1, 42)
        println(@sprintf("Iteration %d", i))
        KinMPCPathFollower.update_init_cond(x,y,psi,v)
        df_desired = KinMPCPathFollower.solve_model_with_ref(x_ref[i:i+8], y_ref[i:i+8])
        println(@sprintf("\t%f %f %f %f %f", x, y, psi, df, df_desired))
        (x,y,psi,df) = state_update_fncn(v, x, y, psi, df, df_desired)

        push!(x_hist, x)
        push!(y_hist, y)
        push!(psi_hist, psi)
        push!(df_hist, df)
        push!(df_des_hist, df_desired)
    end

	# plot the results
    #subplot(511)
    #plot(t_ref[1:length(x_hist)], x_hist, color="r", label="Actual")
    #plot(t_ref, x_ref, color="k", label="Desired")
    #legend()
    #xlabel("Time (s)")
    #ylabel("X (m)")

    subplot(511)
    plot(t_ref[1:length(x_hist)], x_ref[1:length(x_hist)] - x_hist, color="r", label="Actual")
    #plot(t_ref, x_ref, color="k", label="Desired")
    #legend()
    xlabel("Time (s)")
    ylabel("Error in X (m)")

    subplot(512)
    plot(t_ref[1:length(x_hist)], y_ref[1:length(y_hist)] - y_hist, color="r", label="Actual")
    #plot(t_ref, x_ref, color="k", label="Desired")
    #legend()
    xlabel("Time (s)")
    ylabel("Error in Y (m)")
    
    #subplot(512)
    #plot(t_ref[1:length(y_hist)], y_hist, color="r", label="Actual")
    #plot(t_ref, y_ref, color="k", label="Desired")
    #legend()
    #xlabel("Time (s)")
    #ylabel("Y (m)")

    subplot(513)
    plot(t_ref[1:length(psi_hist)], psi_hist, color="r", label="Actual")
    legend()
    xlabel("Time (s)")
    ylabel("Psi (rad)")

    subplot(514)
    plot(t_ref[1:length(df_hist)], df_hist, color="r", label="Actual")
    plot(t_ref[1:length(df_des_hist)], df_des_hist, color="k", label="Desired")
    legend()
    xlabel("Time (s)")
    ylabel("Steering Angle (rad)")

    subplot(515)
    plot(x_hist, y_hist, color="r", label="Actual")
    plot(x_ref, y_ref, color="k", label="Reference")
    legend()
    xlabel("X (m)")
    ylabel("Y (m)")

    #tight_layout()

    show()
end

function test_mpc_module_with_data()
	tfd_intp = npzread("../data/tfd_interp.npy") # t, x, y, psi
	srd_intp = npzread("../data/srd_interp.npy") # t, swa, swa_cmd, speed
	acc_intp = npzread("../data/acc_interp.npy") # t, acc

    # Data downsampled to 10 Hz.
	t_arr   = tfd_intp[:,1] - tfd_intp[1,1]
    x_arr   = tfd_intp[:,2]
    y_arr   = tfd_intp[:,3]
    psi_arr = tfd_intp[:,4]

    swa_arr     = srd_intp[:,2]
	swa_cmd_arr = srd_intp[:,3]
    speed_arr   = srd_intp[:,4]

    acc_arr     = acc_intp[:,2]
   
    
    # make a path at 5 Hz
    t_ref = [t_arr[i] for i in 1:2:length(t_arr)]
    x_ref = [x_arr[i] for i in 1:2:length(t_arr)]
    y_ref = [y_arr[i] for i in 1:2:length(t_arr)]
    psi_ref = [psi_arr[i] for i in 1:2:length(t_arr)]

    t_ref = convert(Array{Float64,1}, t_ref)
    x_ref = convert(Array{Float64,1}, x_ref)
    y_ref = convert(Array{Float64,1}, y_ref)
    psi_ref = convert(Array{Float64,1}, psi_ref)

    # IC and store history of path taken
    x   = x_arr[1]
    y   = y_arr[1]
    psi = psi_arr[1]
    v   = speed_arr[1]
    df  = swa_arr[1]/14.8

    x_hist = [x]
    y_hist = [y]
    psi_hist = [psi]
    v_hist = []
    df_hist = [df]
    a_hist = []
     
    # for loop: 
    st_ind = 1
    end_ind = length(t_ref) - 8
    for i in range(st_ind, end_ind)        
        println(@sprintf("Iteration %d", i))
        KinMPCPathFollower.update_init_cond(x,y,psi,v)
        a_des, df_des, solve_status = KinMPCPathFollower.solve_model_with_ref(x_ref[i:i+8], y_ref[i:i+8], psi_ref[i:i+8])
        KinMPCPathFollower.update_current_input(df_des, a_des)
        println(@sprintf("\t%d: %s x:%.3f y:%.3f psi:%.3f v:%.3f a_des:%.3f, df:%.3f df_des:%.3f", i, solve_status, x, y, psi, v, a_des, df, df_des))
        (x,y,psi, v, df) = state_update_fncn(x, y, psi, v, a_des, df, df_des)

        push!(x_hist, x)
        push!(y_hist, y)
        push!(psi_hist, psi)
        push!(v_hist, v)

        push!(df_hist, df)		# assume steering wheel angle can't immediately be reached, so pass actual swa, not the command!
		push!(a_hist, a_des)
    end

	# plot the results
    subplot(711)
    l1 = plot(t_ref[1:5:length(x_hist)], x_hist[1:5:length(x_hist)], color="r", marker="o", label="MPC")
    l2 = plot(t_ref, x_ref, color="k", label="Desired")
    #legend()
    xlabel("Time (s)")
    ylabel("X (m)")

 
    subplot(712)
    plot(t_ref[1:5:length(y_hist)], y_hist[1:5:length(y_hist)], color="r", marker="o", label="MPC")
    plot(t_ref, y_ref, color="k", label="Desired")
    #legend()
    xlabel("Time (s)")
    ylabel("Y (m)")

    subplot(713)
    plot(t_ref[1:5:length(psi_hist)], psi_hist[1:5:length(psi_hist)], color="r", marker="o", label="MPC")
    plot(t_arr, psi_arr, color="k", label="Desired")
	#legend()
    xlabel("Time (s)")
    ylabel("Psi (rad)")

    subplot(714)
    plot(t_ref[1:5:length(v_hist)], v_hist[1:5:length(v_hist)], color="r", marker="o", label="MPC")
    plot(t_arr, speed_arr, color="k", label="Desired")
	#legend()
    xlabel("Time (s)")
    ylabel("V (m/s)")

    subplot(715)
    plot(t_ref[1:5:length(a_hist)], a_hist[1:5:length(a_hist)], color="r", marker="o", label="MPC")
    plot(t_arr, acc_arr, color="k", label="Desired")
	#legend()
    xlabel("Time (s)")
    ylabel("A (m/s^2)")

    subplot(716)
    plot(t_ref[1:5:length(df_hist)], df_hist[1:5:length(df_hist)], color="r", marker="o", label="MPC")
    plot(t_arr, swa_cmd_arr/14.8, color="k", label="Actual")
    #legend()
    xlabel("Time (s)")
    ylabel("Steering Angle (rad)")

    subplot(717)
    plot(x_hist[1:5:end], y_hist[1:5:end], color="r", marker="o", label="MPC")
    plot(x_arr, y_arr, color="k", label="Reference")
    #legend()
    xlabel("X (m)")
    ylabel("Y (m)")

    tight_layout()
    #figlegend((l1,l2), ("MPC", "Actual"), loc="upper right")
    suptitle("MPC (red) vs Reference (black) Trajectory")
    show()

end


if ! isinteractive()
    test_mpc_module_with_data()
end
