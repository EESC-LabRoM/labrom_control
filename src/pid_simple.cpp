/*************************************************************************
*   Implementation of a simplified pid  controller 
*   This file is part of labrom_control generic controls package
*   
*   labrom_control is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_control is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_control.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

// labrom_control libraries
#include <labrom_control/pid_simple.h>

// ros libraries
#include "ros/ros.h"

namespace controllers{
namespace pid{
/**
* Empty constructor
*/
Simple::Simple(void){};


/**
* Simplified constructor
* @param[in] name_id name of the controller for identification  
*/
Simple::Simple(std::string name_id): Controller("PID", name_id), _kp(0), _ki(0), _kd(0){
  // Initializing variables
  _windup_thresh = 0;
  _max_sampling_time = 1;
  state_ = IDLE;
}


/**
* Construct with controllers parameters
* @param[in] name_id name of the controller for identification
* @param[in] kp proportional gain
* @param[in] ki integrative gain
* @param[in] kd derivative gain
* @param[in] windup_thresh threshold for anti-windup filter
*/
Simple::Simple(std::string name_id, double kp, double ki, double kd, double windup_thresh): Controller("PID", name_id), _kp(kp), _ki(ki), _kd(kd){
  // Initializing variables
  _windup_thresh = windup_thresh;
  _max_sampling_time = 1;
  state_ = IDLE;
}


/**
* Empty destructor
*/
Simple::~Simple(){};

/**
* Set PID controller parameters
* @param[in] kp proportional gain
* @param[in] ki integrative gain
* @param[in] kd derivative gain
* @param[in] windup_thresh threshold for anti-windup filter
*/
void Simple::SetParams(double kp, double ki, double kd, double windup_thresh){
  // Modifying PID controller parameters..
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _windup_thresh = std::fabs(windup_thresh);

  // Log
  ROS_INFO("[%s][%s] New parameters uploaded [kp, ki, kd] <- [%f, %f, %f], [Anti wind-up value] <- [%f]", 
              name_type_.c_str(), name_id_.c_str(), 
              _kp, _ki, _kd,
              _windup_thresh);
}

/**
* Force a a change PID state to flush variables
*/
void Simple::Reset(void){
  state_ = RESET;
}

/**
* Loop once: computes pid controller iteration
* @param[in] ref reference value to be followed
* @param[in] feedback state value measured
* @param[in] d_ref speed value to be followed [DEFAULT = 0]
* @return The output vlaue computed by the pid controller
*/
double Simple::LoopOnce(double ref, double feedback, double d_ref){
  double P, I, D;
  double dt = ros::Time::now().toSec() - previous_time_;

  if (dt > _max_sampling_time)
    state_ = RESET;

  switch (state_){

    // Nothing to do
    case IDLE: 
      output_ = 0;
      state_ = RESET;

    // Reset controller initial parameters
    case RESET:
      error_sum_ = 0;
      output_ = 0;
      previous_state_ = feedback;
      state_ = ACTIVE;
      break;

    // Iterate control law itself
    case ACTIVE:
     /// Proportional
     P = _kp * (ref - feedback);
     /// Integrative sumation and anti-windup
     double increment = (ref - feedback)*dt ;
     I = (error_sum_ + increment) * _ki;
     if (std::fabs(I) > _windup_thresh){
       I = _windup_thresh * I/std::fabs(I);
       ROS_WARN("[%s][%s]: Anti-windup threshold reached (value: %.2f)", name_type_.c_str(), name_id_.c_str(), I);
     } else{
       error_sum_ += increment;
     }
     /// Derivative
     D  = (d_ref - (feedback - previous_state_) / dt) * _kd; 
     /// PID controller output
     output_    = P + I + D;
     break;
  }
  // Saving values for next iteration
  previous_time_ = ros::Time::now().toSec();
  previous_state_ = feedback;

  return output_;
}


/**
* Loop once: computes pid controller iteration
* @param[in] ref reference value to be followed
* @param[in] feedback state value measured
* @param[in] d_ref speed value to be followed 
* @param[in  speed value measured 
* @return The output vlaue computed by the pid controller
*/
double Simple::LoopOnce(double ref, double feedback, double d_ref, double d_feedback){
  double P, I, D;
  double dt = ros::Time::now().toSec() - previous_time_;

  if (dt > _max_sampling_time)
    state_ = RESET;

  switch (state_){

    // Nothing to do
    case IDLE: 
      output_ = 0;
      state_ = RESET;

    // Reset controller initial parameters
    case RESET:
      error_sum_ = 0;
      output_ = 0;
      state_ = ACTIVE;
      break;

    // Iterate control law itself
    case ACTIVE:
     /// Proportional
     P = _kp * (ref - feedback);
     /// Integrative sumation and anti-windup
     double increment = (ref - feedback)*dt ;
     I = (error_sum_ + increment) * _ki;
     if (std::fabs(I) > _windup_thresh){
       I = _windup_thresh * I/std::fabs(I);
       ROS_WARN("[%s][%s]: Anti-windup threshold reached (value: %.2f)", name_type_.c_str(), name_id_.c_str(), I);
     } else{
       error_sum_ += increment;
     }
     /// Derivative
     D  = (d_ref - d_feedback) * _kd; 
     /// PID controller output
     output_    = P + I + D;
     break;
  }
  // Saving values for next iteration
  previous_time_ = ros::Time::now().toSec();

  return output_;
}

} // pid namespace
} // controllers namespace
