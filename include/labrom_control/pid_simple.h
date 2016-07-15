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
// dynamic reconfigure libraries
#include <dynamic_reconfigure/server.h>
#include <labrom_control/PID_simpleConfig.h>

namespace controllers{
namespace pid{
//! Simplified PID controller
class Simple : public Controller{
  public:
    //! Constructor
    Simple(std::string name_id);
    //! Constructor with parameters
    Simple(std::string name_id, double kp, double ki, double kd, double windup_thresh);
    //! Empty Destructor 
    ~Simple(void);
    //! Set parameters handle
    bool SetParams(double kp=0, double ki=0, double kd=0, double windup_thresh=0);
    //! Flush integrative sumation
    bool Flush(void);
    //! Loop a controller iteration
    double LoopOnce(double ref, double feedback, double dt, double d_ref=0);
    //! Get controller output
    double GetOutput(void);
    // Dynamic reconfigure callback
    void DynamicReconfigureCallback(labrom_control::PID_simpleConfig &config, uint32_t level);

  private:
    double kp_, ki_, kd_;                 //!< PID controller parameters
    double windup_thresh_;                //!< Anti-windup threshold
    double error_sum_;                    //!< Integrative sumation
    double state_ant_;                    //!< Last measured feedback value
    double output_val_;                   //!< Computed controller output
    bool active_;                         //!< Indicates wheter PID is active or not

   dynamic_reconfigure::Server<labrom_control::PID_simpleConfig> dynconfig_server_;
   dynamic_reconfigure::Server<labrom_control::PID_simpleConfig>::CallbackType dynconfig_callback_;
}; 

} // pid
} // controllers

#endif // SIMPLE_PID_H
