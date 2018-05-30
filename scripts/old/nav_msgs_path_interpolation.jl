#!/usr/bin/env julia
#= Code to take a set of waypoints and produce a trajectory (path + time information) given current velocity. =#

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

function interpolate_path(target_vel::Float64, msg::Path, t_ref::Array{Float64,1})
    #= This function transforms waypoints into a trajectory.  The following is assumed:
       (1) first waypoint is (0,0,0) which is not in the msg.poses array
       (2) the vehicle travels along the line segments between the positions in msg.poses
       (3) the vehicle travels at a fixed velocity given by target_vel
       (4) linear interpolation is used to determine (x,y,theta) for a set of desired times

       Input:
       target_vel is the assumed longitudinal velocity of the vehicle.
       msg is a ROS nav_msgs/Path type, consisting of desired poses not including the origin.
       t_ref is an array consisting of the timepoints needed to define the trajectory.
   
       Output:
       x_ref, y_ref, psi_ref: arrays containing pose information corresponding to times in t_ref
    =#

    cumulative_dist = 0.0 # distance taken between by the vehicle to get to current waypoint
    t_arr = zeros(0)      # estimated time the vehicle will reach waypoint i.
    x_arr = zeros(0)      # planned x coordinate wrt vehicle local coordinate system (base_link)
    y_arr = zeros(0)      # planned y coordinate wrt vehicle local coordinate system (base_link)
    psi_arr = zeros(0)    # planned heading wrt vehicle local x-axis (base_link)
    
    # Extract poses and estimated time reached for the array of poses (msg.poses). 
    for i in range(1, length(msg.poses))
        if i == 1
            position_curr = msg.poses[i].pose.position
            orientation_curr = msg.poses[i].pose.orientation
            cumulative_dist = cumulative_dist + sqrt(position_curr.x^2 + position_curr.y^2)
        else
            position_curr = msg.poses[i].pose.position
            orientation_curr = msg.poses[i].pose.orientation
            position_prev = msg.poses[i-1].pose.position
            cumulative_dist = cumulative_dist +
                sqrt( (position_curr.x - position_prev.x)^2 + 
                (position_curr.y - position_prev.y)^2 )
        end
        

        push!(t_arr, cumulative_dist/target_vel)
        push!(x_arr, position_curr.x)
        push!(y_arr, position_curr.y)
        push!(psi_arr, 2.0*atan2(orientation_curr.z,orientation_curr.w) ) # quaternion to angle
    end

    #log_str = @sprintf("%d Poses, %d Times, %d Positions\n" ,length(msg.poses), length(t_arr), length(x_arr))
    #println(log_str)

    x_ref = zeros(0)            # computed target x coordinate wrt vehicle local coordinate system (base_link) at time given by t_ref        
    y_ref = zeros(0)            # computed target y coordinate wrt vehicle local coordinate system (base_link) at time given by  t_ref        
    psi_ref = zeros(0)          # computed target heading wrt vehicle local coordinate system (base_link) at time given by  t_ref        

    # Interpolate given trajectory to get pose at reference timepoints (t_ref).
    for t_des in t_ref
        diff_t = [(tm-t_des)^2 for tm in t_arr]
        diff_min_ind = findmin(diff_t)[2] #return tuple index 1 is min value, index2 is min index
        t_closest = t_arr[diff_min_ind]

        if t_closest < t_des

            if diff_min_ind == length(t_arr) 
                # if t_des is greater than all times in t_arr, use endpoint of trajectory.
                x_des = x_arr[end]
                y_des = y_arr[end]
                psi_des = psi_arr[end]
            else
                # t_closest = t_m, t_m < t_des < t_m+1
                #x_des = x_m + (x_m+1 - x_m)/(t_m+1 - t_m) * (t_des - t_m), linear interpolation between closest point m and next point m+1
                interval_diff = t_arr[diff_min_ind + 1] - t_closest
                interp_diff = t_des - t_closest            

                x_des = x_arr[diff_min_ind] + 
                        (x_arr[diff_min_ind + 1] - x_arr[diff_min_ind])/interval_diff * interp_diff
                y_des = y_arr[diff_min_ind] + 
                        (y_arr[diff_min_ind + 1] - y_arr[diff_min_ind])/interval_diff * interp_diff
                psi_des = psi_arr[diff_min_ind] + 
                        (psi_arr[diff_min_ind + 1] - psi_arr[diff_min_ind])/interval_diff * interp_diff
            end
        else
            if diff_min_ind == 1
                # if t_des is less than all times in t_arr, interpolate between (0,0) and first point.
                x_des = x_arr[1] * t_des/t_closest
                y_des = y_arr[1] * t_des/t_closest
                psi_des = psi_arr[1] * t_des/t_closest
            else
                # t_closest = t_m, t_m-1 < t_des < t_m
                # x_des = x_m-1 + (x_m - x_m-1)/(t_m - t_m-1) * (t_des - t_m-1), linear interpolation between prev point m-1 and closest point m
                interval_diff = t_closest - t_arr[diff_min_ind - 1]
                interp_diff = t_des - t_arr[diff_min_ind - 1]            

                x_des = x_arr[diff_min_ind-1] + 
                        (x_arr[diff_min_ind] - x_arr[diff_min_ind-1])/interval_diff * interp_diff
                y_des = y_arr[diff_min_ind-1] + 
                        (y_arr[diff_min_ind] - y_arr[diff_min_ind-1])/interval_diff * interp_diff
                psi_des = psi_arr[diff_min_ind-1] + 
                        (psi_arr[diff_min_ind] - psi_arr[diff_min_ind-1])/interval_diff * interp_diff
            end
        end
        push!(x_ref, x_des)
        push!(y_ref, y_des)
        push!(psi_ref, psi_des)
    end

    return x_ref, y_ref, psi_ref # extract by using return_variable[1,2, or 3].
    
    # For debugging:
    #println("\nTarget Path\n")
    #for j in range(1, length(t_arr))
    #    log_str = @sprintf("t: %f\tX: %f\ty: %f\tpsi: %f\n", t_arr[j], x_arr[j], y_arr[j], psi_arr[j])
    #    println(log_str) 
    #end
    #
    #println("\nReference Trajectory\n")
    #for k in range(1, length(t_ref))
    #    log_str = @sprintf("tr: %f\tXr: %f\tyr: %f\tpsir: %f\n", t_ref[k], x_ref[k], y_ref[k], psi_ref[k])
    #    println(log_str) 
    #end 
    #
end
