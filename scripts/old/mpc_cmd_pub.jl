#!/usr/bin/env julia

include("helper_code/kinematic_mpc_module.jl")
include("helper_code/nav_msgs_path_interpolation.jl") # All ROS imports here.
import KinMPCPathFollower
using JLD

#=
TODO: Description.
=#

dt = KinMPCPathFollower.dt
N = KinMPCPathFollower.N
L_b = KinMPCPathFollower.L_b

t_ref = collect(0:dt:N*dt)
x_ref = zeros(length(t_ref))
y_ref = zeros(length(t_ref))
psi_ref = zeros(length(t_ref))

received_reference = false 		#TODO: better to use a time-based check to see if input valid.

speed = 0.0
int_speed = 15.0
acc_filt = 0.0
curr_wheel_angle = 0.0


ref_lock = false

function steer_callback(msg::SteeringReport)
	if ref_lock == false
		global speed, curr_wheel_angle

	    speed = Float64(msg.speed)
		curr_wheel_angle = Float64(msg.steering_wheel_angle/14.8) # divide by steering ratio
	end
end

function acc_filt_callback(msg::Float64Msg)
	if ref_lock == false
		global acc_filt
		acc_filt = Float64(msg.data)
	end
end

function path_callback(msg::Path)
    time = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs

	if(length(msg.poses) == 0)
		logwarn("Invalid Path")
		return
	end

  	if ref_lock == false
		loginfo(@sprintf("Path received at: %.3f", time))
		global x_ref, y_ref, psi_ref, speed, int_speed

		if speed < int_speed
			#sp = speed + 1.0
			sp = int_speed
		else
			sp = int_speed
		end

	    x_ref, y_ref, psi_ref = interpolate_path(sp, msg, t_ref)
	    #KinMPCPathFollower.update_reference(x_ref,y_ref, psi_ref)

		global received_reference
    	received_reference = true
    end
 
    #x_first = msg.poses[1].pose.position.x
    #y_first = msg.poses[1].pose.position.y
    #x_end   = msg.poses[end].pose.position.x
    #y_end   = msg.poses[end].pose.position.y
    
    #log_str = @sprintf("PATH START: (%f, %f)\tPATH END: (%f, %f)", x_first, y_first, x_end, y_end)
    #loginfo(log_str)
	#loginfo(@sprintf("Path received at: %.3f", time))
end

function pub_loop(pub_obj)
    loop_rate = Rate(10.0)
    count = 0

	filename = @sprintf("mpc_np_%s.jld", Dates.DateTime(now()) )
	jldopen(filename, "w") do file

	    while ! is_shutdown()
    	    if ! received_reference
    	        rossleep(loop_rate)
    	        continue
    	    end
	
    	    global ref_lock
    	    ref_lock = true
	
			global speed, curr_wheel_angle, acc_filt
			global x_ref, y_ref, psi_ref

			# Update Model
			KinMPCPathFollower.update_init_cond(0.0, 0.0, 0.0, speed)
			#KinMPCPathFollower.update_current_input(curr_wheel_angle, acc_filt)
			KinMPCPathFollower.update_reference(x_ref, y_ref, psi_ref)
	
    	    ref_lock = false
	
    	    a_opt, df_opt, is_opt = KinMPCPathFollower.solve_model()

			rostm = get_rostime()
			tm_secs = rostm.secs + 1e-9 * rostm.nsecs
	
    	    log_str = @sprintf("Solve Status: %s, Acc: %.3f, SA: %.3f", is_opt, a_opt, df_opt)
    	    loginfo(log_str)
	
    	    u_msg = MPC_cmd()
    	    u_msg.accel_cmd = a_opt
    	    u_msg.steer_angle_cmd = df_opt
    	     
			#if is_opt == :Optimal
		        publish(pub_obj, u_msg)
			#end

			KinMPCPathFollower.update_current_input(df_opt, a_opt)
			res = KinMPCPathFollower.get_solver_results()
			save_name = @sprintf("iter_%d", count)
			write(file, save_name, (tm_secs, res))
			count = count + 1
    	    rossleep(loop_rate)
    	end
	end
end	

function start_mpc_node()
    init_node("dbw_mpc_pf")
    pub = Publisher("mpc_cmd",MPC_cmd, queue_size=2)
    sub_steer = Subscriber("steering_report", SteeringReport, steer_callback, queue_size=2)    
    sub_path = Subscriber("target_path", Path, path_callback, queue_size=2)
	sub_acc  = Subscriber("filtered_accel", Float64Msg, acc_filt_callback, queue_size=2)    

    pub_loop(pub)    
end

if ! isinteractive()
    start_mpc_node()
end

