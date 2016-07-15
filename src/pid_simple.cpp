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
* Simplified constructor
* @param name_id name of the controller for identification  
*/
Simple::Simple(std::string name_id): Controller(name_id), kp_(0), ki_(0), kd_(0){
  // Initializing variables
  name_type_ = "PID";
  windup_thresh_ = 0;
  error_sum_ = 0;
  state_ant_ = 0;
  output_val_ = 0;  
  active_ = false;
  // Dynamic reconfigure callback
  dynconfig_callback_ = boost::bind(&Simple::DynamicReconfigureCallback, this, _1, _2);
  dynconfig_server_.setCallback(dynconfig_callback_);
}


/**
* Construct with controllers parameters
* @param name_id name of the controller for identification
* @param kp proportional gain
* @param ki integrative gain
* @param kd derivative gain
* @param windup_thresh threshold for anti-windup filter
*/
Simple::Simple(std::string name_id, double kp, double ki, double kd, double windup_thresh): Controller(name_id), kp_(kp), ki_(ki), kd_(kd){
  // Initializing variables
  name_type_ = "PID";
  windup_thresh_ = windup_thresh;
  error_sum_ = 0;
  state_ant_ = 0;  
  output_val_ = 0;
  active_ = false;
  // Dynamic reconfigure callback
  dynconfig_callback_ = boost::bind(&Simple::DynamicReconfigureCallback, this, _1, _2);
  dynconfig_server_.setCallback(dynconfig_callback_);
}


/**
* Empty destructor
*/
Simple::~Simple(){};

/**
* Set PID controller parameters
* @param kp proportional gain
* @param ki integrative gain
* @param kd derivative gain
* @param windup_thresh threshold for anti-windup filter
*/
bool Simple::SetParams(double kp, double ki, double kd, double windup_thresh){
  // Modifying PID controller parameters..
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  windup_thresh_ = std::fabs(windup_thresh);

  // Log
  ROS_INFO("[%s][%s] New parameters uploaded [kp, ki, kd] <- [%f, %f, %f], [Anti wind-up value] <- [%f]", 
              name_type_.c_str(), name_id_.c_str(), 
              kp_, ki_, kd_,
              windup_thresh_);

  return true;
}

/**
* Flush integrative component stored in variable error_sum_
*/
bool Simple::Flush(void){
  // Flushing..
  error_sum_ = 0;
  return true;
}

/**
* Loop once: computes pid controller iteration
* @param ref reference value to be followed
* @param feedback state value measured
* @param dt coontroller sampling time
* @param d_ref speed value to be followed [DEFAULT = 0]
* @return The output vlaue computed by the pid controller
*/
double Simple::LoopOnce(double ref, double feedback, double dt, double d_ref){
  // Proportional
  double P = kp_ * (ref - feedback);

  // Integrative sumation and anti-windup
  error_sum_  += (ref - feedback)*dt ;
  double I = error_sum_ * ki_;
  if (I > windup_thresh_){
    I = windup_thresh_;
    std::clog << "Anti-windup upper threshold reached" << std::endl;
  }
  else if ( I < -windup_thresh_ ){
    I = -windup_thresh_;
    std::clog << "Anti-windup lower threshol reached" << std::endl;
  }  
 
  // Derivative
  double D  = 0;
  if (active_){   // Avoid computing derivative for the first iteration
    double d_p  = (feedback - state_ant_) / dt;
    D = kp_ * (d_ref - d_p); 
  }else
    active_ = true;
  state_ant_ = feedback; // Save for next iteration  

  // PID controller output
  output_val_    = P + I + D;

  return output_val_;

}

/**
* Get output value
* @return last computed controller output
*/
double Simple::GetOutput(void){
  return output_val_;
}

/**
* Dynamic reconfigure callback. 
* This function is called for updating the controller parameters
*/

void Simple::DynamicReconfigureCallback(labrom_control::PID_simpleConfig &config, uint32_t level){
  if (config.send){
    // Upload parameters
    this->SetParams(config.kp, config.ki, config.kd, config.windup_thresh);
  }

}  

} // pid namespace
} // controllers namespace
