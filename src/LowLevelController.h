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

/* Originally TwistControllerNode.h.  Modified by Vijay Govindarajan 
 * to enable acceleration and steering inputs directly 
 */

#ifndef LOWLEVELCONTROL_H_
#define LOWLEVELCONTROL_H_

// ROS and messages
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <dbw_mkz_msgs/ThrottleCmd.h>
#include <dbw_mkz_msgs/ThrottleReport.h>
#include <dbw_mkz_msgs/BrakeCmd.h>
#include <dbw_mkz_msgs/BrakeReport.h>
#include <dbw_mkz_msgs/SteeringCmd.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/FuelLevelReport.h>
#include <dbw_mkz_msgs/TwistCmd.h>
#include <geometry_msgs/Twist.h>
#include <mkz_mpc_path_follower/MPC_cmd.h>

// Debug message
#include <std_msgs/Float64.h>
#include <mkz_mpc_path_follower/acc_stamped.h>

// Controllers
#include "PidControl.h"
#include "LowPass.h"

namespace dbw_mkz_low_level_controller {

class LowLevelControllerNode{
public:
  LowLevelControllerNode(ros::NodeHandle &n, ros::NodeHandle &pn);
private:
  void controlCallback(const ros::TimerEvent& event);
  void recvFuel(const dbw_mkz_msgs::FuelLevelReport::ConstPtr& msg);
  void recvSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg);
  void recvEnable(const std_msgs::Bool::ConstPtr& msg);
  void recvCommand(const mkz_mpc_path_follower::MPC_cmd::ConstPtr& msg);

  ros::Publisher pub_throttle_;
  ros::Publisher pub_brake_;
  ros::Publisher pub_steering_;
  ros::Publisher pub_accel_;
  ros::Publisher pub_req_accel_;

  ros::Subscriber sub_steering_;
  ros::Subscriber sub_enable_;
  ros::Subscriber sub_fuel_level_;
  ros::Subscriber sub_cmd_;
  
  geometry_msgs::Twist actual_;
  double steer_wheel_angle;

  mkz_mpc_path_follower::MPC_cmd mpc_cmd_;

  ros::Timer control_timer_;
  ros::Time cmd_stamp_;

  PidControl accel_pid_;
  LowPass lpf_accel_;
  LowPass lpf_fuel_;
  bool sys_enable_, pedals_enable_, steer_enable_;

  // Parameters
  double control_period_;
  double acker_wheelbase_;
  double acker_track_;
  double steering_ratio_;
  double accel_max_, decel_max_;
  double kp_, ki_;

  static const double GAS_DENSITY = 2.858; // kg/gal
  static double mphToMps(double mph) { return mph * 0.44704; }
  
  // Added Parameters (previously from cfg file):
  static const double FUEL_CAPACITY       = 13.5;    // gal
  static const double BRAKE_DEADBAND      = 0.1;     // m/s*s
  static const double WHEEL_RADIUS        = 0.33;	 // m
  static const double BASE_VEHICLE_MASS   = 1847.78; // kg
  static const double STEER_MAX		      = 8.2;     // rad, +/- 470 deg
  static const double STEER_MAX_DERIV     = 8.0;    // rad/s, 500 deg/s = 8.7 rad/s
  // VEHICLE_MASS = 1736.35 - GAS_DENSITY*FUEL_CAPACITY + 150.0 (passenger allowance)

};

}

#endif /* LOWLEVELCONTROL_H_ */


