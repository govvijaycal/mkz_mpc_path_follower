/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/* Originally TwistControllerNode.cpp.  Modified by Vijay Govindarajan 
 * to enable acceleration and steering inputs directly 
 */

#include "LowLevelController.h"

#define VERBOSE 1

namespace dbw_mkz_low_level_controller {

LowLevelControllerNode::LowLevelControllerNode(ros::NodeHandle &n, ros::NodeHandle &pn)
{
  lpf_fuel_.setParams(60.0, 0.1); //tau, ts
  lpf_accel_.setParams(0.5, 0.02);

  accel_pid_.setGains(0.8, 0.4, 0.0); //kp,ki,kd
  accel_pid_.setRange(0.0, 1.0);      //min/max throttle %

  // Control rate parameter
  double control_rate;
  pn.param("control_rate", control_rate, 50.0);
  control_period_ = 1.0 / control_rate;

  // Ackermann steering parameters
  acker_wheelbase_ = 2.8498; // 112.2 inches
  acker_track_ = 1.5824; // 62.3 inches
  steering_ratio_ = 14.8;
  pn.getParam("ackermann_wheelbase", acker_wheelbase_);
  pn.getParam("ackermann_track", acker_track_);
  pn.getParam("steering_ratio", steering_ratio_);

  pn.param("accel_max", accel_max_, 1.0);
  pn.param("decel_max", decel_max_, 1.0);

  if(accel_max_ < 0 || decel_max_ < 0) {
    pedals_enable_ = false;
    steer_enable_ = false;
  	ROS_INFO_STREAM("ERROR: Invalid Acceleration Limits Given!  Disabling Pedals and Steering Control\n");
  }

  pn.param("pub_pedals", pedals_enable_, false);
  pn.param("pub_steering", steer_enable_, false);
#if VERBOSE
  ROS_INFO_STREAM("Pedals Enable: " << pedals_enable_ << "\n");
  ROS_INFO_STREAM("Steer Enable: "  << steer_enable_  << "\n");
  ROS_INFO_STREAM("Acceleration Limits: [" << -decel_max_ << ", " << accel_max_ << "]\n");
  ROS_INFO_STREAM("Steering Ratio: " << steering_ratio_ << "\n");
#endif

  // Subscribers
  sub_steering_ = n.subscribe("steering_report", 1, &LowLevelControllerNode::recvSteeringReport, this);
  sub_enable_ = n.subscribe("dbw_enabled", 1, &LowLevelControllerNode::recvEnable, this);
  sub_fuel_level_ = n.subscribe("fuel_level_report", 1, &LowLevelControllerNode::recvFuel, this);
  sub_cmd_ = n.subscribe("mpc_cmd", 1, &LowLevelControllerNode::recvCommand, this);

  // Publishers
  pub_throttle_ = n.advertise<dbw_mkz_msgs::ThrottleCmd>("throttle_cmd", 1);
  pub_brake_ = n.advertise<dbw_mkz_msgs::BrakeCmd>("brake_cmd", 1);
  pub_steering_ = n.advertise<dbw_mkz_msgs::SteeringCmd>("steering_cmd", 1);

  // Debug
  //pub_accel_ = n.advertise<std_msgs::Float64>("filtered_accel", 1);
  //pub_req_accel_ = n.advertise<std_msgs::Float64>("req_accel", 1);
  pub_accel_ = n.advertise<mkz_mpc_path_follower::acc_stamped>("filtered_accel", 1);
  pub_req_accel_ = n.advertise<mkz_mpc_path_follower::acc_stamped>("req_accel", 1);


  // Timers
  control_timer_ = n.createTimer(ros::Duration(control_period_), &LowLevelControllerNode::controlCallback, this);
}

void LowLevelControllerNode::controlCallback(const ros::TimerEvent& event)
{
  if ((event.current_real - cmd_stamp_).toSec() > (10.0 * control_period_)) {
    accel_pid_.resetIntegrator();
    return;
  }

  double curr_vehicle_mass = BASE_VEHICLE_MASS + lpf_fuel_.get() / 100.0 * FUEL_CAPACITY * GAS_DENSITY;

  double accel_cmd = mpc_cmd_.accel_cmd;
  double curr_accel = 0;

  if(accel_cmd > accel_max_){
	accel_cmd = accel_max_;
  }
  else if(accel_cmd < -decel_max_) {
	accel_cmd = -decel_max_;
  }

  //std_msgs::Float64 accel_cmd_msg;
  //accel_cmd_msg.data = accel_cmd;
  mkz_mpc_path_follower::acc_stamped accel_cmd_msg;
  accel_cmd_msg.accel_value = accel_cmd;
  accel_cmd_msg.header.stamp = ros::Time::now();
  pub_req_accel_.publish(accel_cmd_msg);

  if (sys_enable_) {
    dbw_mkz_msgs::ThrottleCmd throttle_cmd;
    dbw_mkz_msgs::BrakeCmd brake_cmd;
    dbw_mkz_msgs::SteeringCmd steering_cmd;

    throttle_cmd.enable = true;
    throttle_cmd.pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT;

  	curr_accel = lpf_accel_.get();	

    if (accel_cmd >= 0) {
      throttle_cmd.pedal_cmd = accel_pid_.step(accel_cmd - curr_accel, control_period_);
    } else {
      accel_pid_.resetIntegrator();
      throttle_cmd.pedal_cmd = 0;
    }

    brake_cmd.enable = true;
    brake_cmd.pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_TORQUE;
    if (accel_cmd < -BRAKE_DEADBAND) {
      brake_cmd.pedal_cmd = -accel_cmd * curr_vehicle_mass * WHEEL_RADIUS;
    } else {
      brake_cmd.pedal_cmd = 0;
    }

    steering_cmd.enable = true;

    double target_wheel_angle = mpc_cmd_.steer_angle_cmd * steering_ratio_;

    /*
    if ( (target_wheel_angle - steer_wheel_angle) > (STEER_MAX_DERIV *control_period_) ) {
      target_wheel_angle = steer_wheel_angle + STEER_MAX_DERIV*control_period_;
    }
    else if ( (steer_wheel_angle - target_wheel_angle) > (STEER_MAX_DERIV*control_period_) ) {
      target_wheel_angle = steer_wheel_angle - STEER_MAX_DERIV*control_period_;
    }
    */

	if(fabs(target_wheel_angle) > STEER_MAX) {
		target_wheel_angle = target_wheel_angle > 0.0 ? STEER_MAX : -STEER_MAX;
	}

    steering_cmd.steering_wheel_angle_cmd  = target_wheel_angle;

    if (pedals_enable_) {
      pub_throttle_.publish(throttle_cmd);
      pub_brake_.publish(brake_cmd);
    }

    if (steer_enable_) {
      pub_steering_.publish(steering_cmd);
    }

#if VERBOSE
	ROS_INFO("ThCmd: %.3f, BkCmd: %.3f, StCmd: %.3f, ActA: %.3f, RefA: %.3f, ActSt: %.3f, RefSt: %.3f\n", throttle_cmd.pedal_cmd,
		brake_cmd.pedal_cmd, target_wheel_angle, curr_accel, mpc_cmd_.accel_cmd, steer_wheel_angle, mpc_cmd_.steer_angle_cmd*steering_ratio_);
#endif

  } else {
    accel_pid_.resetIntegrator();
  }
}

void LowLevelControllerNode::recvFuel(const dbw_mkz_msgs::FuelLevelReport::ConstPtr& msg)
{
  lpf_fuel_.filt(msg->fuel_level);
}

void LowLevelControllerNode::recvSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg)
{
  double raw_accel = 50.0 * (msg->speed - actual_.linear.x);
  lpf_accel_.filt(raw_accel);

  //std_msgs::Float64 accel_msg;
  //accel_msg.data = lpf_accel_.get();

  mkz_mpc_path_follower::acc_stamped accel_msg;
  accel_msg.accel_value = lpf_accel_.get();
  accel_msg.header.stamp = ros::Time::now();
  pub_accel_.publish(accel_msg);
  

  actual_.linear.x = msg->speed;
  steer_wheel_angle = msg->steering_wheel_angle;

/*#if verbose
  ROS_INFO("LLC SR Received - Speed: %.3f, FiltA: %.3f, SWA: %.3f", actual_.linear.x, accel_msg.accel_value, steer_wheel_angle);
#endif
*/
}

void LowLevelControllerNode::recvEnable(const std_msgs::Bool::ConstPtr& msg)
{
  //ROS_INFO_STREAM("HERE: SYS_ENABLE: " << msg->data << "\n");
  sys_enable_ = msg->data;
}

void LowLevelControllerNode::recvCommand(const mkz_mpc_path_follower::MPC_cmd::ConstPtr& msg) {
  mpc_cmd_ = *msg;
/*#if verbose
  ROS_INFO("LLC MPC Received - RefA: %.3f, RefSt: %.3f", mpc_cmd_.accel_cmd, mpc_cmd_.steer_angle_cmd);
#endif
*/
  cmd_stamp_ = ros::Time::now();
}


}

