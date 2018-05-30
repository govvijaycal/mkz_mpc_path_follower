#!/usr/bin/env julia

using RobotOS
@rosimport geometry_msgs.msg: Twist
@rosimport dbw_mkz_msgs.msg: SteeringReport
@rosimport nav_msgs.msg: Path
rostypegen()
using geometry_msgs.msg
using dbw_mkz_msgs.msg
using nav_msgs.msg

#=
This code simply prints out the steering information and path information from the lane detector + path planner.
Assume that v = 15 m/s and focus on steering wheel angle planner.

Encapsulate MPC implementation into a class and just include it here.  Need to figure out safe way to encapsulate updates
so MPC doesn't clash with new messages received by subscribers.
=#

function steer_callback(msg::SteeringReport)
    time = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
    speed = msg.speed
    swa = msg.steering_wheel_angle

    log_str = @sprintf("TIME: %s\tSPEED: %f\tSTEER_WHEEL: %f", time, speed, swa)
    loginfo(log_str)
end

function path_callback(msg::Path)
    time = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
    x_first = msg.poses[1].pose.position.x
    y_first = msg.poses[1].pose.position.y
    x_end   = msg.poses[end].pose.position.x
    y_end   = msg.poses[end].pose.position.y
    
    log_str = @sprintf("PATH START: (%f, %f)\tPATH END: (%f, %f)", x_first, y_first, x_end, y_end)
    loginfo(log_str)
end

function start_subscribers()
    init_node("dbw_mpc_pf")
    sub_steer = Subscriber("vehicle/steering_report", SteeringReport, steer_callback, queue_size=10)
    sub_path = Subscriber("vehicle/target_path", Path, path_callback, queue_size=10)    
    spin()
end

if ! isinteractive()
    start_subscribers()
end


