/*************************************************************************
*   Simplified pid  controller header files 
*   This file is part of labrom_control generic controls package
*   
*   labrom_control is a free sotfware: you can redistribute it and/or modify
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


#ifndef PID_SIMPLE_H_
#define PID_SIMPLE_H_
// labrom_control libraries
#include<labrom_control/controllers.h>
// C++ libriares
#include <cmath>
// ROS libraries
#include "ros/ros.h"

namespace controllers{
namespace pid{
// PID state machine
enum PIDState{IDLE=0, RESET, ACTIVE};
//! Simplified PID controller
class Simple : public Controller{
  public:
    //! Empty Constructor
    Simple(void);
    //! Constructor
    Simple(std::string name_id);
    //! Constructor with parameters
    Simple(std::string name_id, double kp, double ki, double kd, double windup_thresh);
    //! Empty Destructor 
    ~Simple(void);
    //! Set parameters handle
    void SetParams(double kp=0, double ki=0, double kd=0, double windup_thresh=0);
    //! Flush integrative sumation
    void Reset(void);
    //! Loop a controller iteration
    double LoopOnce(double ref, double feedback, double d_ref=0);


  private:
    double error_sum_;                    //!< Integrative sumation
    double previous_state_;                    //!< Last measured feedback value
    double previous_time_;
    PIDState state_;

    double _kp, _ki, _kd;                 //!< PID controller parameters
    double _windup_thresh;                //!< Anti-windup threshold
    double _max_sampling_time;

}; 

} // pid
} // controllers

#endif // SIMPLE_PID_H
