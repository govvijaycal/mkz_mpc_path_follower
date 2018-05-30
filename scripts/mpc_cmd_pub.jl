#!/usr/bin/env julia

# ROS Imports
using RobotOS
@rosimport geometry_msgs.msg: Twist
@rosimport dbw_mkz_msgs.msg: SteeringReport
@rosimport nav_msgs.msg: Path
@rosimport mkz_mpc_path_follower.msg: MPC_cmd
@rosimport std_msgs.msg: Float64
rostypegen()
using geometry_msgs.msg
using dbw_mkz_msgs.msg
using nav_msgs.msg
using mkz_mpc_path_follower.msg
using std_msgs.msg

# Access Python modules for path processing.  Ugly way of doing it, can seek to clean this up in the future.
using PyCall
const path_utils_loc = "/home/govvijay/catkin_ws/src/mkz_mpc_path_follower/scripts/path_utils"
unshift!(PyVector(pyimport("sys")["path"]), path_utils_loc) # append the current directory to Python path
@pyimport nav_msgs_path_xy as nmp

# Access MPC Controller.
include("mpc_utils/kinematic_mpc_module.jl")
import KinMPCPathFollower
const kmpc = KinMPCPathFollower

# For Data Logging
using JLD

const t_ref = collect(0:kmpc.dt:kmpc.N*kmpc.dt)
x_ref = zeros(length(t_ref))
y_ref = zeros(length(t_ref))
psi_ref = zeros(length(t_ref))

received_reference = false 		#TODO: can use time from last reading to see if data is fresh for MPC update.

des_speed = 15.0
curr_speed = 0.0
curr_acc_filt = 0.0
curr_wheel_angle = 0.0
ref_lock = false

function steer_callback(msg::SteeringReport)
	if ref_lock == false
		global curr_speed, curr_wheel_angle
	    curr_speed = Float64(msg.speed)
		curr_wheel_angle = Float64(msg.steering_wheel_angle/14.8) # divide by steering ratio
	end
end

function acc_filt_callback(msg::Float64Msg)
	if ref_lock == false
		global curr_acc_filt
		curr_acc_filt = Float64(msg.data)
	end
end

function convert_msg_to_path_dict(msg::Path)
    cumulative_dist = 0.0 # distance taken between by the vehicle to get to current waypoint
    s_arr = zeros(0)      # estimated time the vehicle will reach waypoint i.
    x_arr = zeros(0)      # planned x coordinate wrt vehicle local coordinate system (base_link)
    y_arr = zeros(0)      # planned y coordinate wrt vehicle local coordinate system (base_link)
    psi_arr = zeros(0)    # planned heading wrt vehicle local x-axis (base_link)
    
    # Extract poses and estimated time reached for the array of poses (msg.poses). 
    for i in range(1, length(msg.poses))
    	position_curr = msg.poses[i].pose.position
		orientation_curr = msg.poses[i].pose.orientation        
        if i == 1
            cumulative_dist = cumulative_dist + sqrt(position_curr.x^2 + position_curr.y^2)
        else
            position_prev = msg.poses[i-1].pose.position
            cumulative_dist = cumulative_dist +
                sqrt( (position_curr.x - position_prev.x)^2 + 
                (position_curr.y - position_prev.y)^2 )
        end        
        push!(s_arr, cumulative_dist)
        push!(x_arr, position_curr.x)
        push!(y_arr, position_curr.y)
        push!(psi_arr, 2.0*atan2(orientation_curr.z,orientation_curr.w) ) # quaternion to angle
    end

    path = Dict()
    path["x"] = x_arr
    path["y"] = y_arr
    path["s"] = s_arr
    path["psi"] = psi_arr
    return path
end


function path_callback(msg::Path)
    time = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs

	if(length(msg.poses) == 0)
		logwarn("Invalid Path")
		return
	end

  	if ref_lock == false
		loginfo(@sprintf("Path received at: %.3f", time))
		global x_ref, y_ref, psi_ref, curr_speed, des_speed

		#= Ramp up speed slowly
		if curr_speed < (des_speed - 1.0) 
			sp = curr_speed + 1.0
		else
			sp = des_speed
		end
		=#
		sp = des_speed

		#Could do the msg_to_path conversion in Python using below method, but seems to be slow:
		#https://github.com/jdlangs/RobotOS.jl/issues/33
		path = convert_msg_to_path_dict(msg)
			    
	    x_ref, y_ref, psi_ref = nmp.get_reference_using_t(path, t_ref, sp)
		global received_reference
    	received_reference = true
    end
end

function pub_loop(pub_obj)
    loop_rate = Rate(10.0)
    count = 0

	filename = @sprintf("mpc_xy_%s.jld", Dates.DateTime(now()) )
	jldopen(filename, "w") do file

	    while ! is_shutdown()
    	    if ! received_reference
    	        rossleep(loop_rate)
    	        continue
    	    end
	
    	    global ref_lock
    	    ref_lock = true
	
			global curr_speed, curr_wheel_angle, curr_acc_filt
			global x_ref, y_ref, psi_ref

			# Update Model
			kmpc.update_init_cond(0.0, 0.0, 0.0, curr_speed)
			kmpc.update_reference(x_ref, y_ref, psi_ref)
	
    	    ref_lock = false
	
    	    a_opt, df_opt, is_opt = kmpc.solve_model()

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

			kmpc.update_current_input(df_opt, a_opt)
			res = kmpc.get_solver_results()
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